/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <TMIV/Renderer/ViewWeightingSynthesizer.h>

#include <TMIV/Common/Graph.h>
#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <tuple>

namespace TMIV::Renderer {
namespace {
template <typename MAT>
auto textureGather(const MAT &m, const Common::Vec2f &p)
    -> Common::stack::Vec4<typename MAT::value_type> {
  const auto w_last = static_cast<int32_t>(m.width()) - 1;
  const auto h_last = static_cast<int32_t>(m.height()) - 1;

  const auto x0 = std::clamp(static_cast<int32_t>(std::floor(p.x() - 0.5F)), 0, w_last);
  const auto y0 = std::clamp(static_cast<int32_t>(std::floor(p.y() - 0.5F)), 0, h_last);

  const auto x1 = std::min(x0 + 1, w_last);
  const auto y1 = std::min(y0 + 1, h_last);

  return {m(y1, x0), m(y1, x1), m(y0, x1), m(y0, x0)};
}

void insertWeightedDepthInStack(std::vector<Common::Vec2f> &stack, float weight, float z,
                                float blendingFactor) {
  if (0.F < weight) {
    for (size_t i = 0; i <= stack.size(); i++) {
      const auto zAfter =
          (i < stack.size()) ? stack[i].x() / stack[i].y() : std::numeric_limits<float>::max();

      if (z < zAfter) {
        const auto zBefore = (0 < i) ? (stack[i - 1].x() / stack[i - 1].y()) : -1.F;

        const auto mergeBefore = (z - zBefore) < (zBefore * blendingFactor);
        const auto mergeAfter = (zAfter - z) < (z * blendingFactor);

        if (mergeBefore && mergeAfter) {
          stack[i - 1] += (weight * Common::Vec2f({z, 1.F}) + stack[i]);

          for (size_t j = i; j < stack.size() - 1; j++) {
            stack[j] = stack[j + 1];
          }

          stack.pop_back();
        } else if (mergeBefore) {
          stack[i - 1] += weight * Common::Vec2f({z, 1.F});
        } else if (mergeAfter) {
          stack[i] += weight * Common::Vec2f({z, 1.F});
        } else {
          stack.push_back(Common::Vec2f({0.F, 0.F}));

          for (size_t j = (stack.size() - 1); i < j; j--) {
            stack[j] = stack[j - 1];
          }

          stack[i] = weight * Common::Vec2f({z, 1.F});
        }

        break;
      }
    }
  }
}

auto getEnabledIdList(const std::vector<bool> &inputList) -> std::vector<size_t> {
  auto outputList = std::vector<size_t>{};

  for (size_t id = 0; id < inputList.size(); id++) {
    if (inputList[id]) {
      outputList.emplace_back(id);
    }
  }

  return outputList;
}

template <int32_t N>
auto computeMatrixM(const std::array<Common::Vec2i, N> &offsetList,
                    const std::array<std::pair<Common::Vec2f, float>, N> &Q,
                    const std::array<float, N> &W, const Common::Vec2f &C) -> Common::Mat2x2f {
  auto M = Common::Mat2x2f{0.F, 0.F, 0.F, 0.F};

  for (size_t i = 0U; i < offsetList.size(); i++) {
    if (isValidDepth(Common::at(Q, i).second)) {
      const auto dp = Common::at(Q, i).first - C;
      M += Common::at(W, i) * Common::stack::Mat2x2<float>{dp.x() * dp.x(), dp.x() * dp.y(),
                                                           dp.x() * dp.y(), dp.y() * dp.y()};
    }
  }
  return M;
}
} // namespace

class ViewWeightingSynthesizer::Impl {
private:
  struct Splat {
    Common::Vec2f center{};
    Common::Vec2f firstAxis{};
    Common::Vec2f secondAxis{};
    float pointSize{};
  };

  struct BoundingBox {
    int32_t x0;
    int32_t x1;
    int32_t y0;
    int32_t y1;
  };

  struct FilterReprojectedPrunedDepthMapsParams {
    int32_t erodeCount{};
    int32_t dilateCount{};
  };

  std::vector<float> m_cameraWeight;
  std::vector<bool> m_cameraVisibility;
  std::vector<float> m_cameraDistortion;
  std::vector<Common::Mat<Common::Vec3f>> m_sourceColor;
  std::vector<Common::Mat<float>> m_sourceDepth;
  std::vector<Common::Mat<Common::Vec3f>> m_sourceUnprojection;
  std::vector<Common::Mat<std::pair<Common::Vec2f, float>>> m_sourceReprojection;
  std::vector<Common::Mat<Common::Vec3f>> m_sourceRayDirection;
  std::vector<Common::Mat<Common::Vec3f>> m_viewportUnprojection;
  std::vector<Common::Mat<float>> m_viewportDepth;
  std::vector<Common::Mat<float>> m_viewportWeight;
  Common::Mat<float> m_viewportVisibility;
  Common::Mat<Common::Vec3f> m_viewportColor;
  std::set<size_t> m_inpaintedViews;

  float m_angularScaling;
  float m_minimalWeight;
  float m_stretchFactor;
  float m_blendingFactor;
  float m_overloadFactor;
  int32_t m_filteringPass;
  std::optional<FilterReprojectedPrunedDepthMapsParams> m_filterReprojectedPrunedDepthMaps;

public:
  explicit Impl(const Common::Json &componentNode)
      : m_angularScaling{componentNode.require("angularScaling").as<float>()}
      , m_minimalWeight{componentNode.require("minimalWeight").as<float>()}
      , m_stretchFactor{componentNode.require("stretchFactor").as<float>()}
      , m_blendingFactor{componentNode.require("blendingFactor").as<float>()}
      , m_overloadFactor{componentNode.require("overloadFactor").as<float>()}
      , m_filteringPass{componentNode.require("filteringPass").as<int32_t>()} {
    if (const auto &node = componentNode.optional("filterReprojectedPrunedDepthMaps")) {
      m_filterReprojectedPrunedDepthMaps = FilterReprojectedPrunedDepthMapsParams{
          node.require("erodeCount").as<int32_t>(), node.require("dilateCount").as<int32_t>()};
    }
    Common::logVerbose("[VT prep] filterReprojectedPrunedDepthMaps is {}",
                       m_filterReprojectedPrunedDepthMaps.has_value());
  }

  auto renderFrame(const MivBitstream::AccessUnit &frame,
                   const MivBitstream::CameraConfig &cameraConfig) -> Common::RendererFrame {
    const auto &viewParamsList = frame.viewParamsList;
    const auto sourceHelperList = ProjectionHelperList{viewParamsList};
    const auto targetHelper = ProjectionHelper{cameraConfig.viewParams};

    // 0) Initialization
    findInpaintedView(frame);
    computeCameraWeight(sourceHelperList, targetHelper);

    // 1) Deconstruction
    const auto depthBitDepth = recoverPrunedSource(frame, sourceHelperList);

    // 0) Initialization (cont'd)
    computeCameraVisibility(sourceHelperList, targetHelper, depthBitDepth);
    computeAngularDistortionPerSource(sourceHelperList);

    // 2) Reprojection
    reprojectPrunedSource(frame, sourceHelperList, targetHelper);

    // 3) Warping
    warpPrunedSource(frame, targetHelper);

    // 3.5) Morphological filtering
    if (m_filterReprojectedPrunedDepthMaps) {
      filterReprojectedPrunedDepthMaps(frame);
    }

    // 4) Weight recovery
    recoverPrunedWeight(sourceHelperList, targetHelper);

    // 5) Selection
    VERIFY(frame.casps);
    selectViewportDepth(!frame.casps->casps_miv_extension().casme_depth_low_quality_flag(),
                        targetHelper);

    // 6) Filtering
    filterVisibilityMap();

    // 7) Shading
    computeShadingMap(sourceHelperList, targetHelper);

    // 8) Output
    for (size_t i = 0U; i < m_viewportColor.size(); i++) {
      if (isValidDepth(m_viewportVisibility[i])) {
        if (m_viewportColor[i].x() < 0.F) {
          m_viewportVisibility[i] = NAN;
          m_viewportColor[i] = Common::Vec3f{};
        } else {
          m_viewportVisibility[i] = std::clamp(1.F / m_viewportVisibility[i],
                                               cameraConfig.viewParams.dq.dq_norm_disp_low(),
                                               cameraConfig.viewParams.dq.dq_norm_disp_high());
        }
      }
    }

    auto viewport = Common::RendererFrame{
        quantizeTexture(m_viewportColor, cameraConfig.bitDepthTexture),
        MivBitstream::DepthTransform{cameraConfig.viewParams.dq, cameraConfig.bitDepthGeometry}
            .quantizeNormDisp(m_viewportVisibility, 1)};
    viewport.texture.fillInvalidWithNeutral(viewport.geometry);
    return viewport;
  }

private:
  [[nodiscard]] auto isViewInpainted(size_t viewIdx) const -> bool {
    return m_inpaintedViews.count(viewIdx) != 0;
  }

  [[nodiscard]] auto hasInpaintedViews() const -> bool { return !m_inpaintedViews.empty(); }

  void findInpaintedView(const MivBitstream::AccessUnit &frame) {
    m_inpaintedViews.clear();

    for (size_t viewIdx = 0; viewIdx < frame.viewParamsList.size(); ++viewIdx) {
      if (frame.viewParamsList[viewIdx].viewInpaintFlag) {
        m_inpaintedViews.insert(viewIdx);
      }
    }
  }

  void filterReprojectedPrunedDepthMaps(const MivBitstream::AccessUnit &frame) {
    const auto &viewParamsList = frame.viewParamsList;

    const auto W = Common::downCast<int32_t>(m_viewportDepth[0].width());
    const auto H = Common::downCast<int32_t>(m_viewportDepth[0].height());

    Common::Mat<float> tmpmat;
    Common::Mat<float> tmpmat2;
    tmpmat.resize(H, W);
    tmpmat2.resize(H, W);

    for (size_t v = 0; v < viewParamsList.size() - 1; v++) {
      if (!m_cameraVisibility[v]) {
        continue;
      }

      filterReprojectedPrunedDepthMapsCopyInput(tmpmat, v);
      filterReprojectedPrunedDepthMapsInplaceErode(tmpmat, tmpmat2);
      filterReprojectedPrunedDepthMapsInplaceDilate(tmpmat, tmpmat2, v);
      filterReprojectedPrunedDepthMapsCopyOutput(tmpmat, v);
    }
  }

  void filterReprojectedPrunedDepthMapsCopyInput(Common::Mat<float> &tmpmat, size_t v) {
    const auto W = Common::downCast<int32_t>(tmpmat.width());
    const auto H = Common::downCast<int32_t>(tmpmat.height());

    for (int32_t h = 0; h < H; h++) {
      for (int32_t w = 0; w < W; w++) {
        tmpmat(h, w) = m_viewportDepth[v](h, w);
      }
    }
  }

  void filterReprojectedPrunedDepthMapsInplaceErode(Common::Mat<float> &tmpmat,
                                                    Common::Mat<float> &tmpmat2) const {
    static constexpr auto halfWindowSize = 1;
    const auto W = Common::downCast<int32_t>(tmpmat.width());
    const auto H = Common::downCast<int32_t>(tmpmat.height());

    for (int32_t i = 0; i < m_filterReprojectedPrunedDepthMaps->erodeCount; i++) {
      for (int32_t h = 0; h < H; h++) {
        for (int32_t w = 0; w < W; w++) {
          tmpmat2(h, w) = tmpmat(h, w);
        }
      }

      for (int32_t h = halfWindowSize; h < H - halfWindowSize; h++) {
        for (int32_t w = halfWindowSize; w < W - halfWindowSize; w++) {
          int32_t emptyNeighbors = 0;
          for (int32_t hh = h - halfWindowSize; hh <= h + halfWindowSize; hh++) {
            for (int32_t ww = w - halfWindowSize; ww <= w + halfWindowSize; ww++) {
              if (!(tmpmat2(hh, ww) > 0)) {
                emptyNeighbors++;
              }
            }
          }

          if (emptyNeighbors > 0) {
            tmpmat(h, w) = NAN;
          }
        }
      }
    }
  }

  void filterReprojectedPrunedDepthMapsInplaceDilate(Common::Mat<float> &tmpmat,
                                                     Common::Mat<float> &tmpmat2, size_t v) {
    static constexpr auto halfWindowSize = 1;
    const auto W = Common::downCast<int32_t>(tmpmat.width());
    const auto H = Common::downCast<int32_t>(tmpmat.height());

    for (int32_t i = 0; i < m_filterReprojectedPrunedDepthMaps->dilateCount; i++) {
      for (int32_t h = 0; h < H; h++) {
        for (int32_t w = 0; w < W; w++) {
          tmpmat2(h, w) = tmpmat(h, w);
        }
      }

      for (int32_t h = halfWindowSize; h < H - halfWindowSize; h++) {
        for (int32_t w = halfWindowSize; w < W - halfWindowSize; w++) {
          int32_t nonEmptyNeighbors = 0;
          for (int32_t hh = h - halfWindowSize; hh <= h + halfWindowSize; hh++) {
            for (int32_t ww = w - halfWindowSize; ww <= w + halfWindowSize; ww++) {
              if ((tmpmat2(hh, ww) > 0)) {
                nonEmptyNeighbors++;
              }
            }
          }

          if (nonEmptyNeighbors > 0) {
            tmpmat(h, w) = m_viewportDepth[v](h, w);
          }
        }
      }
    }
  }

  void filterReprojectedPrunedDepthMapsCopyOutput(Common::Mat<float> &tmpmat, size_t v) {
    const auto W = Common::downCast<int32_t>(tmpmat.width());
    const auto H = Common::downCast<int32_t>(tmpmat.height());

    for (int32_t h = 0; h < H; h++) {
      for (int32_t w = 0; w < W; w++) {
        m_viewportDepth[v](h, w) = tmpmat(h, w);
      }
    }
  }

  void computeCameraWeight(const ProjectionHelperList &sourceHelperList,
                           const ProjectionHelper &targetHelper) {
    const auto isTridimensional = [&]() -> bool {
      constexpr auto epsilon = 1e-2F;
      auto M = Common::Mat3x3f{};

      for (const auto &helper : sourceHelperList) {
        M += square(helper.getViewParams().pose.position);
      }

      return epsilon < det(M);
    };

    if (1 < sourceHelperList.size()) {
      // Distance to each source axis
      const auto is3D = isTridimensional();

      const auto &viewportPosition = targetHelper.getViewingPosition();
      auto cameraDistance = std::vector<float>{};

      for (size_t viewIdx = 0U; viewIdx != sourceHelperList.size(); ++viewIdx) {
        // inpainted view gets large distance to yield low weight
        if (isViewInpainted(viewIdx)) {
          cameraDistance.push_back(std::numeric_limits<float>::infinity());
          continue;
        }

        const auto &helper = sourceHelperList[viewIdx];
        const auto &cameraPosition = helper.getViewingPosition();
        const auto cameraDirection = helper.getViewingDirection();

        cameraDistance.push_back(
            is3D ? norm(cameraPosition - viewportPosition)
                 : norm(cross(cameraPosition - viewportPosition, cameraDirection)));
      }

      // Camera sorting
      auto closestCamera = std::vector<uint32_t>(cameraDistance.size());
      iota(closestCamera.begin(), closestCamera.end(), 0);
      std::sort(closestCamera.begin(), closestCamera.end(),
                [&](uint32_t i1, uint32_t i2) { return cameraDistance[i1] < cameraDistance[i2]; });

      // Reference distance
      auto refDistance = 0.F;

      for (size_t id = 1; refDistance <= std::numeric_limits<float>::epsilon(); id++) {
        refDistance = norm(sourceHelperList[closestCamera[0]].getViewingPosition() -
                           sourceHelperList[closestCamera[id]].getViewingPosition()) *
                      0.25F;
      }

      // Weighting
      m_cameraWeight.clear();

      for (float id : cameraDistance) {
        const auto w = 1.F / (1.F + Common::sqr(id / refDistance));
        m_cameraWeight.emplace_back(w);
      }
    } else {
      m_cameraWeight = {1.F};
    }
  }

  auto static findMaximumDepthRange(const ProjectionHelperList &sourceHelperList,
                                    const uint32_t depthBitDepth) -> Common::Vec2f {
    auto maxDepthRange = Common::Vec2f{INFINITY, 0.F};
    for (const auto &helper : sourceHelperList) {
      const auto depthTransform =
          MivBitstream::DepthTransform{helper.getViewParams().dq, depthBitDepth};

      const auto twoT =
          static_cast<float>(2 * helper.getViewParams().dq.dq_depth_occ_threshold_default());
      float A = (0.F - twoT) / (static_cast<float>(Common::maxLevel(depthBitDepth)) - twoT);
      float ORI_normDispLow = (helper.getViewParams().dq.dq_norm_disp_low() -
                               A * helper.getViewParams().dq.dq_norm_disp_high()) /
                              (1.F - A);
      maxDepthRange[0] =
          std::min(maxDepthRange[0], depthTransform.expandDepth(Common::maxLevel(depthBitDepth)));
      maxDepthRange[1] = std::max(maxDepthRange[1], 1.F / ORI_normDispLow);
    }
    return maxDepthRange;
  }

  void computeCameraVisibility(const ProjectionHelperList &sourceHelperList,
                               const ProjectionHelper &targetHelper, const uint32_t depthBitDepth) {
    const auto N = 4U;
    const auto depthRange = findMaximumDepthRange(sourceHelperList, depthBitDepth);

    auto pointCloud = std::vector<Common::Vec3f>{};
    auto x = 0.F;
    const auto step = 1.F / static_cast<float>(N - 1);

    for (auto i = 0U; i < N; i++) {
      auto y = 0.F;

      const auto px = x * targetHelper.getViewParams().ci.projectionPlaneSizeF().x();

      for (auto j = 0U; j < N; j++) {
        const auto py = y * targetHelper.getViewParams().ci.projectionPlaneSizeF().y();

        pointCloud.push_back(targetHelper.doUnprojection({px, py}, depthRange.x()));
        pointCloud.push_back(targetHelper.doUnprojection({px, py}, depthRange.y()));

        y += step;
      }

      x += step;
    }

    m_cameraVisibility.clear();

    for (size_t viewIdx = 0; viewIdx < sourceHelperList.size(); viewIdx++) {
      if (isViewInpainted(viewIdx)) {
        // The inpainted view should always be visible
        m_cameraVisibility.emplace_back(true);
        continue;
      }

      const auto &helper = sourceHelperList[viewIdx];
      auto K = 0U;

      for (const Common::Vec3f &P : pointCloud) {
        const auto p = helper.doProjection(P);

        if (isValidDepth(p.second) && helper.isInsideViewport(p.first)) {
          K++;
          break;
        }
      }

      m_cameraVisibility.emplace_back(0 < K);
    }
  }

  void computeAngularDistortionPerSource(const ProjectionHelperList &sourceHelperList) {
    m_cameraDistortion.resize(sourceHelperList.size(), 0.F);

    for (size_t viewIdx = 0; viewIdx < sourceHelperList.size(); viewIdx++) {
      if (m_cameraVisibility[viewIdx]) {
        m_cameraDistortion[viewIdx] =
            m_angularScaling *
            static_cast<float>(Common::deg2rad(
                2. / Common::pps2ppd(sourceHelperList[viewIdx].getAngularResolution())));
      }
    }
  }

  auto recoverPrunedSource(const MivBitstream::AccessUnit &frame,
                           const ProjectionHelperList &sourceHelperList) -> uint32_t {
    // Recover pruned views
    const auto prunedViews = recoverPrunedViews(frame);
    uint32_t geoBitDepth{};

    // Expand pruned views
    m_sourceColor.clear();
    m_sourceDepth.clear();

    for (size_t sourceIdx = 0; sourceIdx < prunedViews.size(); sourceIdx++) {
      const auto &viewParams = sourceHelperList[sourceIdx].getViewParams();

      VERIFY(!prunedViews[sourceIdx].texture.empty());
      m_sourceColor.emplace_back(expandTexture(prunedViews[sourceIdx].texture));

      geoBitDepth = prunedViews[sourceIdx].geometry.getBitDepth();

      m_sourceDepth.emplace_back(
          MivBitstream::DepthTransform{viewParams.dq, geoBitDepth}.expandDepth(
              prunedViews[sourceIdx].geometry));

      std::transform(
          prunedViews[sourceIdx].occupancy.getPlane(0).begin(),
          prunedViews[sourceIdx].occupancy.getPlane(0).end(), m_sourceDepth.back().begin(),
          m_sourceDepth.back().begin(),
          [&](auto maskValue, float depthValue) { return 0 < maskValue ? depthValue : NAN; });
    }
    return geoBitDepth;
  }

  void reprojectPrunedSource(const MivBitstream::AccessUnit &frame,
                             const ProjectionHelperList &sourceHelperList,
                             const ProjectionHelper &targetHelper) {
    m_sourceUnprojection.resize(m_sourceDepth.size());
    m_sourceReprojection.resize(m_sourceDepth.size());
    m_sourceRayDirection.resize(m_sourceDepth.size());

    for (size_t sourceIdx = 0; sourceIdx < m_sourceDepth.size(); sourceIdx++) {
      m_sourceUnprojection[sourceIdx].resize(m_sourceDepth[sourceIdx].height(),
                                             m_sourceDepth[sourceIdx].width());
      std::fill(m_sourceUnprojection[sourceIdx].begin(), m_sourceUnprojection[sourceIdx].end(),
                Common::Vec3f{NAN, NAN, NAN});

      m_sourceReprojection[sourceIdx].resize(m_sourceDepth[sourceIdx].height(),
                                             m_sourceDepth[sourceIdx].width());
      std::fill(m_sourceReprojection[sourceIdx].begin(), m_sourceReprojection[sourceIdx].end(),
                std::pair{Common::Vec2f{NAN, NAN}, NAN});

      m_sourceRayDirection[sourceIdx].resize(m_sourceDepth[sourceIdx].height(),
                                             m_sourceDepth[sourceIdx].width());
      std::fill(m_sourceRayDirection[sourceIdx].begin(), m_sourceRayDirection[sourceIdx].end(),
                Common::Vec3f{NAN, NAN, NAN});
    }

    for (const auto &atlas : frame.atlas) {
      if (atlas.asps.asps_miv_extension_present_flag() &&
          atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
        continue;
      }

      Common::parallel_for(
          atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height(), [&](size_t Y, size_t X) {
            const auto patchIdx =
                atlas.filteredPatchIdx(static_cast<int32_t>(Y), static_cast<int32_t>(X));
            if (patchIdx == Common::unusedPatchIdx) {
              return;
            }

            const auto &patchParams = atlas.patchParamsList[patchIdx];
            const auto viewIdx = frame.viewParamsList.indexOf(patchParams.atlasPatchProjectionId());

            if (!m_cameraVisibility[viewIdx]) {
              return;
            }

            const auto sourceViewPos =
                patchParams.atlasToView({static_cast<int32_t>(X), static_cast<int32_t>(Y)});
            const auto x = sourceViewPos.x();
            const auto y = sourceViewPos.y();

            if (y >= static_cast<int32_t>(m_sourceDepth[viewIdx].height()) ||
                x >= static_cast<int32_t>(m_sourceDepth[viewIdx].width())) {
              return;
            }

            const auto d = m_sourceDepth[viewIdx](y, x);

            if (!sourceHelperList[viewIdx].isValidDepth(d)) {
              return;
            }

            const auto P = sourceHelperList[viewIdx].doUnprojection(
                {static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F}, d);
            const auto p = targetHelper.doProjection(P);

            if (isValidDepth(p.second) && targetHelper.isInsideViewport(p.first)) {
              m_sourceUnprojection[viewIdx](y, x) = P;
              m_sourceReprojection[viewIdx](y, x) = p;
              m_sourceRayDirection[viewIdx](y, x) =
                  unit(P - targetHelper.getViewParams().pose.position);
            }
          });
    }
  }

  void warpPrunedSource(const MivBitstream::AccessUnit &frame,
                        const ProjectionHelper &targetHelper) {
    // 3.1) Prepare Viewports
    resizeAndResetViewports(targetHelper);

    // 3.2) Warping
    warpPrunedSourceOnResetViewports(frame);
  }

  void resizeAndResetViewports(const ProjectionHelper &targetHelper) {
    m_viewportUnprojection.resize(m_sourceDepth.size());
    m_viewportDepth.resize(m_sourceDepth.size());

    for (size_t viewIdx = 0; viewIdx < m_sourceDepth.size(); viewIdx++) {
      if (m_cameraVisibility[viewIdx]) {
        m_viewportUnprojection[viewIdx].resize(
            targetHelper.getViewParams().ci.projectionPlaneSize().y(),
            targetHelper.getViewParams().ci.projectionPlaneSize().x());
        std::fill(m_viewportUnprojection[viewIdx].begin(), m_viewportUnprojection[viewIdx].end(),
                  Common::Vec3f{NAN, NAN, NAN});

        m_viewportDepth[viewIdx].resize(targetHelper.getViewParams().ci.projectionPlaneSize().y(),
                                        targetHelper.getViewParams().ci.projectionPlaneSize().x());
        std::fill(m_viewportDepth[viewIdx].begin(), m_viewportDepth[viewIdx].end(), NAN);
      }
    }
  }

  void warpPrunedSourceOnResetViewports(const MivBitstream::AccessUnit &frame) {
    auto visibleSourceId = getEnabledIdList(m_cameraVisibility);

    Common::parallel_for(visibleSourceId.size(), [&](size_t id) {
      const auto viewIdx = Common::downCast<uint16_t>(visibleSourceId[id]);
      const auto viewId = frame.viewParamsList[viewIdx].viewId;

      for (const auto &atlas : frame.atlas) {
        if (atlas.asps.asps_miv_extension_present_flag() &&
            atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
          continue;
        }

        for (const auto &patchParams : atlas.patchParamsList) {
          if (patchParams.atlasPatchProjectionId() != viewId) {
            continue;
          }

          const auto x0 = patchParams.atlasPatch3dOffsetU();
          const auto x1 = x0 + patchParams.atlasPatch3dSizeU();

          const auto y0 = patchParams.atlasPatch3dOffsetV();
          const auto y1 = y0 + patchParams.atlasPatch3dSizeV();

          for (auto y = y0; y < y1; y++) {
            for (auto x = x0; x < x1; x++) {
              if (static_cast<uint32_t>(y) >= m_sourceReprojection[viewIdx].height() ||
                  static_cast<uint32_t>(x) >= m_sourceReprojection[viewIdx].width()) {
                continue;
              }

              const auto P = m_sourceReprojection[viewIdx](y, x);

              if (!isValidDepth(P.second)) {
                continue;
              }

              const auto splatParameters = getSplatParameters(viewIdx, x, y, P);

              if (0.F < splatParameters.pointSize) {
                rasterizePoint(viewIdx, getSplatParameters(viewIdx, x, y, P),
                               m_sourceUnprojection[viewIdx](y, x), P.second);
              }
            }
          }
        }
      }
    });
  }

  auto getSplatParameters(uint32_t viewIdx, int32_t x, int32_t y,
                          const std::pair<Common::Vec2f, float> &P) -> Splat {
    const auto [WT, M] = computeWeightAndMatrixM(viewIdx, x, y);

    if (WT < m_minimalWeight) {
      return {Common::Vec2f{0.F, 0.F}, Common::Vec2f{0.F, 0.F}, Common::Vec2f{0.F, 0.F}, 0.F};
    }

    // Axis (requires at least 5 good candidates)
    if (0.F < WT) {
      const auto b = M[0] + M[3];               // trace
      const auto c = M[0] * M[3] - M[1] * M[2]; // determinant
      const auto delta = (b * b - 4.F * c);

      if ((0.F < c) && (0. < delta)) {
        const auto sqrt_delta = std::sqrt(delta);
        const auto l1 = 0.5F * (b + sqrt_delta);
        const auto l2 = 0.5F * (b - sqrt_delta);

        if (0.F < l2) {
          auto e1 = Common::Vec2f{1.F, 0.F};
          auto e2 = Common::Vec2f{0.F, 1.F};

          if (l1 != l2) {
            const auto u1 = Common::Vec2f{M(0, 0) - l2, M(1, 0)};
            const auto u2 = Common::Vec2f{M(0, 1), M(1, 1) - l2};

            e1 = 0.F < dot(u1, u1) ? unit(u1) : unit(u2);
            e2 = Common::Vec2f{-e1.y(), e1.x()};
          }

          const auto r1 = std::sqrt(2.F * l1 / WT);
          const auto r2 = std::sqrt(2.F * l2 / WT);

          if (r1 < m_stretchFactor) {
            return {P.first, r1 * e1, r2 * e2, 2.F * r1};
          }
        }
      }
    }

    return {P.first, Common::Vec2f{0.F, 0.F}, Common::Vec2f{0.F, 0.F}, 1.F};
  }

  auto computeWeightAndMatrixM(uint32_t viewIdx, int32_t x, int32_t y)
      -> std::tuple<float, Common::Mat2x2f> {
    static constexpr auto offsetList =
        std::array{Common::Vec2i({1, 0}),  Common::Vec2i({1, 1}),  Common::Vec2i({0, 1}),
                   Common::Vec2i({-1, 1}), Common::Vec2i({-1, 0}), Common::Vec2i({-1, -1}),
                   Common::Vec2i({0, -1}), Common::Vec2i({1, -1})};
    auto Q = std::array<std::pair<Common::Vec2f, float>, 8>{};
    auto W = std::array<float, 8>{};
    auto WT = 0.F;

    // Center
    auto C = Common::Vec2f{0.F, 0.F};

    const auto OP = m_sourceRayDirection[viewIdx](y, x);
    const auto w_last = static_cast<int32_t>(m_sourceReprojection[viewIdx].width()) - 1;
    const auto h_last = static_cast<int32_t>(m_sourceReprojection[viewIdx].height()) - 1;

    for (size_t i = 0U; i < offsetList.size(); i++) {
      const auto xo = std::clamp(x + Common::at(offsetList, i).x(), 0, w_last);
      const auto yo = std::clamp(y + Common::at(offsetList, i).y(), 0, h_last);

      Common::at(Q, i) = m_sourceReprojection[viewIdx](yo, xo);

      if (isValidDepth(Common::at(Q, i).second)) {
        const auto OQ = m_sourceRayDirection[viewIdx](yo, xo);

        const float a = std::acos(dot(OP, OQ)) / m_cameraDistortion[viewIdx];
        const float wi = std::exp(-a * a);

        Common::at(W, i) = wi;
        C += wi * Common::at(Q, i).first;
        WT += wi;
      }
    }

    C /= WT;

    if (0.F < WT && m_minimalWeight <= WT) {
      return {WT, computeMatrixM<8>(offsetList, Q, W, C)};
    }
    return {WT, Common::Mat2x2f{0.F, 0.F, 0.F, 0.F}};
  }

  void rasterizePoint(uint32_t viewIdx, const Splat &splat, const Common::Vec3f &P,
                      float depthValue) {
    const auto R1 = dot(splat.firstAxis, splat.firstAxis);
    const auto R2 = dot(splat.secondAxis, splat.secondAxis);
    const auto boundingBox = computeBoundingBox(viewIdx, splat);

    // Looping on all pixels within the bounding box
    for (int32_t y = boundingBox.y0; y <= boundingBox.y1; y++) {
      const auto dy = (static_cast<float>(y) + 0.5F) - splat.center.y();

      for (int32_t x = boundingBox.x0; x <= boundingBox.x1; x++) {
        const auto dx = (static_cast<float>(x) + 0.5F) - splat.center.x();
        const auto depthRef = m_viewportDepth[viewIdx](y, x);

        if (!isValidDepth(depthRef) || (depthValue < depthRef)) {
          if (0.F < R1) {
            const auto dp = Common::Vec2f{dx, dy};

            const auto f1 = std::abs(dot(splat.firstAxis, dp));
            const auto f2 = std::abs(dot(splat.secondAxis, dp));

            if (f1 <= R1 && f2 <= R2) {
              m_viewportUnprojection[viewIdx](y, x) = P;
              m_viewportDepth[viewIdx](y, x) = depthValue;
            }
          } else {
            m_viewportUnprojection[viewIdx](y, x) = P;
            m_viewportDepth[viewIdx](y, x) = depthValue;
          }
        }
      }
    }
  }

  auto computeBoundingBox(uint32_t viewIdx, const Splat &splat) -> BoundingBox {
    // Initialization
    const auto w_last = static_cast<int32_t>(m_viewportDepth[viewIdx].width()) - 1;
    const auto h_last = static_cast<int32_t>(m_viewportDepth[viewIdx].height()) - 1;
    const auto radius = 0.5F * splat.pointSize;

    // Bounding box
    const auto xLow = std::max(0.F, splat.center.x() - radius);
    const auto xHigh = splat.center.x() + radius;
    const auto yLow = std::max(0.F, splat.center.y() - radius);
    const auto yHigh = splat.center.y() + radius;
    const auto x0 = std::max(0, static_cast<int32_t>(std::floor(xLow)));
    const auto x1 = std::min(w_last, static_cast<int32_t>(std::ceil(xHigh)));
    const auto y0 = std::max(0, static_cast<int32_t>(std::floor(yLow)));
    const auto y1 = std::min(h_last, static_cast<int32_t>(std::ceil(yHigh)));
    return {x0, x1, y0, y1};
  }

  void recoverPrunedWeight(const ProjectionHelperList &sourceHelperList,
                           const ProjectionHelper &targetHelper) {
    // Retrieve pruning information
    const auto hasPruningRelation =
        any_of(sourceHelperList.begin(), sourceHelperList.end(), [](const auto &helper) {
          const auto &viewParams = helper.getViewParams();
          return viewParams.pp && !viewParams.pp->pp_is_root_flag();
        });

    // Weight recovery
    m_viewportWeight.resize(sourceHelperList.size());

    for (size_t viewIdx = 0; viewIdx < m_viewportWeight.size(); viewIdx++) {
      m_viewportWeight[viewIdx].resize(m_viewportDepth[viewIdx].height(),
                                       m_viewportDepth[viewIdx].width());
      std::fill(m_viewportWeight[viewIdx].begin(), m_viewportWeight[viewIdx].end(),
                hasPruningRelation ? 0.F : m_cameraWeight[viewIdx]);
    }

    if (hasPruningRelation) {
      // Pruning graph (from children to parent)
      auto pruningGraph = Common::Graph::SparseDirectedAcyclicGraph<float>(sourceHelperList.size());

      for (size_t nodeId = 0; nodeId < sourceHelperList.size(); nodeId++) {
        const auto &viewParams = sourceHelperList[nodeId].getViewParams();

        if (viewParams.pp) {
          for (auto parentId : *viewParams.pp) {
            pruningGraph.connect(nodeId, static_cast<size_t>(parentId), 1.F);
          }
        }
      }

      const auto pruningOrderId = pruningGraph.getDescendingOrderId();

      // Recovery
      Common::parallel_for(
          targetHelper.getViewParams().ci.projectionPlaneSize().x(),
          targetHelper.getViewParams().ci.projectionPlaneSize().y(), [&](size_t y, size_t x) {
            for (auto prunedNodeId : pruningOrderId) {
              if (m_cameraVisibility[prunedNodeId]) {
                const auto zPruned = m_viewportDepth[prunedNodeId](y, x);

                const auto candidateList =
                    retrieveCandidateList(pruningGraph, y, x, prunedNodeId, zPruned);

                const auto representativeNodeId =
                    findBestRepresentativeNode(sourceHelperList, y, x, prunedNodeId, candidateList);

                m_viewportWeight[representativeNodeId](y, x) += m_cameraWeight[prunedNodeId];
              }
            }
          });
    }
  }

  auto retrieveCandidateList(const Common::Graph::SparseDirectedAcyclicGraph<float> &pruningGraph,
                             size_t y, size_t x, size_t prunedNodeId, float zPruned)
      -> std::vector<std::pair<Common::Graph::NodeId, float>> {
    auto nodeQueue = std::queue<Common::Graph::NodeId>{};
    auto candidateList = std::vector<std::pair<Common::Graph::NodeId, float>>{};

    for (const auto &linkToParent : pruningGraph.getNeighbourhood(prunedNodeId)) {
      nodeQueue.push(linkToParent.id());
    }
    while (!nodeQueue.empty()) {
      const auto unprunedNodeId = nodeQueue.front();

      if (m_cameraVisibility[unprunedNodeId]) {
        const auto zUnpruned = m_viewportDepth[unprunedNodeId](y, x);

        if (isValidDepth(zUnpruned) &&
            (!isValidDepth(zPruned) || ((m_blendingFactor * zUnpruned) < (zPruned - zUnpruned)))) {
          candidateList.emplace_back(unprunedNodeId, zUnpruned);
        }
      }

      for (const auto &linkToParent : pruningGraph.getNeighbourhood(unprunedNodeId)) {
        nodeQueue.push(linkToParent.id());
      }

      nodeQueue.pop();
    }

    std::sort(candidateList.begin(), candidateList.end(),
              [](const auto &p1, const auto &p2) { return (p1.second < p2.second); });

    return candidateList;
  }

  auto findBestRepresentativeNode(
      const ProjectionHelperList &sourceHelperList, const size_t y, const size_t x,
      size_t prunedNodeId,
      const std::vector<std::pair<Common::Graph::NodeId, float>> &candidateList) -> size_t {
    const auto &prunedHelper = sourceHelperList[prunedNodeId];
    const auto w_last = static_cast<int32_t>(m_sourceDepth[prunedNodeId].width()) - 1;
    const auto h_last = static_cast<int32_t>(m_sourceDepth[prunedNodeId].height()) - 1;
    auto representativeNodeId = prunedNodeId;

    for (const auto &candidate : candidateList) {
      const auto p = prunedHelper.doProjection(m_viewportUnprojection[candidate.first](y, x));

      if (isValidDepth(p.second) && prunedHelper.isInsideViewport(p.first)) {
        static constexpr auto offsetList =
            std::array{Common::Vec2i({-1, -1}), Common::Vec2i({0, -1}), Common::Vec2i({1, -1}),
                       Common::Vec2i({-1, 0}),  Common::Vec2i({0, 0}),  Common::Vec2i({1, 0}),
                       Common::Vec2i({-1, 1}),  Common::Vec2i({0, 1}),  Common::Vec2i({1, 1})};

        const auto X = static_cast<int32_t>(std::floor(p.first.x()));
        const auto Y = static_cast<int32_t>(std::floor(p.first.y()));

        for (const auto &offset : offsetList) {
          const auto xo = std::clamp(X + offset.x(), 0, w_last);
          const auto yo = std::clamp(Y + offset.y(), 0, h_last);

          const auto zOnPruned = m_sourceDepth[prunedNodeId](yo, xo);

          if (!prunedHelper.isValidDepth(zOnPruned)) {
            representativeNodeId = candidate.first;
            break;
          }
        }
      }

      if (representativeNodeId != prunedNodeId) {
        break;
      }
    }
    return representativeNodeId;
  }

  void selectViewportDepth(bool trustDepth, const ProjectionHelper &targetHelper) {
    m_viewportVisibility.resize(targetHelper.getViewParams().ci.projectionPlaneSize().y(),
                                targetHelper.getViewParams().ci.projectionPlaneSize().x());

    Common::parallel_for(
        m_viewportVisibility.width(), m_viewportVisibility.height(), [&](size_t y, size_t x) {
          auto stack = std::vector<Common::Vec2f>{};

          for (size_t viewIdx = 0; viewIdx < m_viewportDepth.size(); viewIdx++) {
            if (m_cameraVisibility[viewIdx] && !isViewInpainted(viewIdx)) {
              const auto z = m_viewportDepth[viewIdx](y, x);

              if (isValidDepth(z)) {
                insertWeightedDepthInStack(stack, m_viewportWeight[viewIdx](y, x), z,
                                           m_blendingFactor);
              }
            }
          }

          // Select best candidate
          auto bestCandidate = Common::Vec2f({0.F, 0.F});

          for (const auto &v : stack) {
            if ((bestCandidate.y() < v.y()) && ((bestCandidate.y() * m_overloadFactor) < v.y())) {
              bestCandidate = v;
            }

            if (trustDepth) {
              break;
            }
          }

          m_viewportVisibility(y, x) =
              (0.F < bestCandidate.y()) ? (bestCandidate.x() / bestCandidate.y()) : 0.F;
        });
  }

  void filterVisibilityMap() {
    static constexpr auto offsetList =
        std::array{Common::Vec2i({-1, -1}), Common::Vec2i({0, -1}), Common::Vec2i({1, -1}),
                   Common::Vec2i({-1, 0}),  Common::Vec2i({0, 0}),  Common::Vec2i({1, 0}),
                   Common::Vec2i({-1, 1}),  Common::Vec2i({0, 1}),  Common::Vec2i({1, 1})};

    Common::Mat<float> flipVisibility;

    auto firstWrapper = std::reference_wrapper<Common::Mat<float>>{
        (m_filteringPass % 2) != 0 ? flipVisibility : m_viewportVisibility};
    auto secondWrapper = std::reference_wrapper<Common::Mat<float>>{
        ((m_filteringPass % 2) != 0) ? m_viewportVisibility : flipVisibility};

    const auto w = m_viewportVisibility.width();
    const auto h = m_viewportVisibility.height();

    const auto w_last = static_cast<int32_t>(w) - 1;
    const auto h_last = static_cast<int32_t>(h) - 1;

    flipVisibility.resize(m_viewportVisibility.sizes());

    if (m_filteringPass % 2 != 0) {
      std::copy(m_viewportVisibility.begin(), m_viewportVisibility.end(), flipVisibility.begin());
    }

    for (auto iter = 0; iter < m_filteringPass; ++iter) {
      const auto &firstDepth = firstWrapper.get();
      auto &secondDepth = secondWrapper.get();

      Common::parallel_for(w, h, [&](size_t y, size_t x) {
        auto depthBuffer = std::array<float, 9>{};

        for (size_t i = 0; i < depthBuffer.size(); i++) {
          const auto xo = std::clamp(static_cast<int32_t>(x) + at(offsetList, i).x(), 0, w_last);
          const auto yo = std::clamp(static_cast<int32_t>(y) + at(offsetList, i).y(), 0, h_last);

          const auto z = firstDepth(yo, xo);

          Common::at(depthBuffer, i) = isValidDepth(z) ? z : 0.F;
        }

        std::sort(depthBuffer.begin(), depthBuffer.end());

        for (size_t i = 4; i < 6; i++) {
          if (0.F < Common::at(depthBuffer, i)) {
            secondDepth(y, x) = Common::at(depthBuffer, i);
            return;
          }
        }

        secondDepth(y, x) = 0.F;
      });

      swap(firstWrapper, secondWrapper);
    }
  }

  [[nodiscard]] auto isProneToGhosting(uint32_t sourceIdx, const std::pair<Common::Vec2f, float> &p,
                                       const Common::Vec3f &OP,
                                       const ProjectionHelperList &sourceHelperList) const -> bool {
    static constexpr auto offsetList = std::array{Common::Vec2i({1, 0}), Common::Vec2i({-1, 0}),
                                                  Common::Vec2i({0, 1}), Common::Vec2i({0, -1})};

    const auto w_last = static_cast<int32_t>(m_sourceDepth[sourceIdx].width()) - 1;
    const auto h_last = static_cast<int32_t>(m_sourceDepth[sourceIdx].height()) - 1;

    const auto x = static_cast<int32_t>(std::floor(p.first.x()));
    const auto y = static_cast<int32_t>(std::floor(p.first.y()));

    return std::any_of(offsetList.cbegin(), offsetList.cend(), [&](const auto &offset) {
      const auto xo = std::clamp(x + offset.x(), 0, w_last);
      const auto yo = std::clamp(y + offset.y(), 0, h_last);

      const auto z = m_sourceDepth[sourceIdx](yo, xo);

      if (!sourceHelperList[sourceIdx].isValidDepth(z)) {
        return true;
      }

      const auto OQ = m_sourceRayDirection[sourceIdx](yo, xo);
      return 2.F * m_cameraDistortion[sourceIdx] < std::abs(std::acos(dot(OP, OQ)));
    });
  }

  void computeShadingMapWithInpaintedPixels(size_t x, size_t y,
                                            const ProjectionHelperList &sourceHelperList,
                                            const ProjectionHelper &targetHelper) {
    const auto viewIdInpainted = *m_inpaintedViews.begin();
    const auto &backgroundDepth = m_viewportDepth[viewIdInpainted];
    ASSERT(!backgroundDepth.empty());
    const auto z = backgroundDepth(y, x);

    if (isValidDepth(z)) {
      const auto pn1 = Common::Vec2f{static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F};

      const auto P = targetHelper.doUnprojection(pn1, z);

      const auto [uvBg, zBg] = sourceHelperList[viewIdInpainted].doProjection(P);

      // nearest neighbour fetching of low-res inpainted image
      PRECONDITION(viewIdInpainted < m_sourceColor.size());
      auto &sourceColor = m_sourceColor[viewIdInpainted];
      const auto W = static_cast<int32_t>(sourceColor.width());
      const auto H = static_cast<int32_t>(sourceColor.height());
      const auto j = std::clamp(static_cast<int32_t>(std::round(uvBg.x())), 0, W - 1);
      const auto i = std::clamp(static_cast<int32_t>(std::round(uvBg.y())), 0, H - 1);
      m_viewportColor(y, x) = sourceColor(i, j);
    }
    m_viewportVisibility(y, x) = z;
  }

  void computeShadingMapWithRegularPixels(size_t x, size_t y,
                                          const ProjectionHelperList &sourceHelperList,
                                          const ProjectionHelper &targetHelper,
                                          const Common::Mat<float> &viewportVisibility) {
    static constexpr auto offsetList =
        std::array{Common::Vec2i({0, 0}),   Common::Vec2i({1, 0}),  Common::Vec2i({1, 1}),
                   Common::Vec2i({0, -1}),  Common::Vec2i({1, -1}), Common::Vec2i({0, 1}),
                   Common::Vec2i({-1, -1}), Common::Vec2i({-1, 0}), Common::Vec2i({-1, 1})};

    static const auto d2 = std::array{0.F, 1.F, 2.F, 1.F, 2.F, 1.F, 2.F, 1.F, 2.F};

    const auto w_last = static_cast<int32_t>(viewportVisibility.width()) - 1;
    const auto h_last = static_cast<int32_t>(viewportVisibility.height()) - 1;

    auto stack = std::vector<Common::Vec2f>{};

    for (size_t i = 0U; i < offsetList.size(); i++) {
      const auto xo =
          std::clamp(static_cast<int32_t>(x) + Common::at(offsetList, i).x(), 0, w_last);
      const auto yo =
          std::clamp(static_cast<int32_t>(y) + Common::at(offsetList, i).y(), 0, h_last);

      const auto z = viewportVisibility(yo, xo);

      if (isValidDepth(z)) {
        const auto ksi = 1.F / (1.F + Common::at(d2, i));
        insertWeightedDepthInStack(stack, ksi, z, m_blendingFactor);
      }
    }

    const auto [oColor, oWeight] =
        computeColorAndWeight(sourceHelperList, targetHelper, y, x, stack);

    static constexpr auto eps = 1e-3F;

    m_viewportColor(y, x) =
        eps < oWeight ? oColor / oWeight
                      : (isValidDepth(viewportVisibility(y, x)) ? Common::Vec3f{-1.F, -1.F, -1.F}
                                                                : Common::Vec3f{});
  }

  void computeShadingMap(const ProjectionHelperList &sourceHelperList,
                         const ProjectionHelper &targetHelper) {
    m_viewportColor.resize(targetHelper.getViewParams().ci.projectionPlaneSize().y(),
                           targetHelper.getViewParams().ci.projectionPlaneSize().x());

    const auto viewportVisibility = m_viewportVisibility;

    Common::parallel_for(
        m_viewportVisibility.width(), m_viewportVisibility.height(), [&](size_t y, size_t x) {
          if (!isValidDepth(m_viewportVisibility(y, x)) && hasInpaintedViews()) {
            computeShadingMapWithInpaintedPixels(x, y, sourceHelperList, targetHelper);
          } else {
            computeShadingMapWithRegularPixels(x, y, sourceHelperList, targetHelper,
                                               viewportVisibility);
          }
        });
  }

  [[nodiscard]] auto computeColorAndWeight(const ProjectionHelperList &sourceHelperList,
                                           const ProjectionHelper &targetHelper, size_t y, size_t x,
                                           const std::vector<Common::Vec2f> &stack) const
      -> std::tuple<Common::Vec3f, float> {
    auto oColor = Common::Vec3f{};
    auto oWeight = 0.F;
    const auto &O = targetHelper.getViewingPosition();
    const auto isOnViewportContour = (1U < stack.size());

    for (const auto &element : stack) {
      const auto z = element.x() / element.y();

      const auto pn1 = Common::Vec2f{static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F};

      const auto P = targetHelper.doUnprojection(pn1, z);
      const auto OP = unit(P - O);

      for (size_t sourceIdx = 0U; sourceIdx < sourceHelperList.size(); sourceIdx++) {
        if (m_cameraVisibility[sourceIdx] && !isViewInpainted(sourceIdx)) {
          const auto pn2 = sourceHelperList[sourceIdx].doProjection(P);

          if (isValidDepth(pn2.second) && sourceHelperList[sourceIdx].isInsideViewport(pn2.first)) {
            if (isOnViewportContour ||
                !isProneToGhosting(static_cast<uint32_t>(sourceIdx), pn2, OP, sourceHelperList)) {
              incrementColorAndWeight(element, sourceIdx, pn2, sourceHelperList, oColor, oWeight);
            }
          }
        }
      }
    }
    return {oColor, oWeight};
  }

  void incrementColorAndWeight(const Common::Vec2f &element, size_t sourceIdx,
                               const std::pair<Common::Vec2f, float> &pn2,
                               const ProjectionHelperList &sourceHelperList, Common::Vec3f &oColor,
                               float &oWeight) const {
    const auto zRef = textureGather(m_sourceDepth[sourceIdx], pn2.first);
    const auto cRef = textureGather(m_sourceColor[sourceIdx], pn2.first);

    const auto q = Common::Vec2f{pn2.first.x() - 0.5F, pn2.first.y() - 0.5F};
    const auto f = Common::Vec2f{q.x() - std::floor(q.x()), q.y() - std::floor(q.y())};
    const auto fb = Common::Vec2f{1.F - f.x(), 1.F - f.y()};

    const auto wColor =
        Common::Vec4f{fb.x() * f.y(), f.x() * f.y(), f.x() * fb.y(), fb.x() * fb.y()};

    for (auto j = 0U; j < 4U; j++) {
      if (sourceHelperList[sourceIdx].isValidDepth(zRef[j])) {
        const auto nu = (zRef[j] - pn2.second) / (pn2.second * m_blendingFactor);
        const auto wDepth = element.y() / (1.F + nu * nu);

        const auto w = (m_cameraWeight[sourceIdx] * wColor[j] * wDepth);

        oColor += w * cRef[j];
        oWeight += w;
      }
    }
  }
};

ViewWeightingSynthesizer::ViewWeightingSynthesizer(const Common::Json & /*rootNode*/,
                                                   const Common::Json &componentNode)
    : m_impl(new Impl{componentNode}) {}

ViewWeightingSynthesizer::~ViewWeightingSynthesizer() = default;

auto ViewWeightingSynthesizer::renderFrame(const MivBitstream::AccessUnit &frame,
                                           const MivBitstream::CameraConfig &cameraConfig) const
    -> Common::RendererFrame {
  return m_impl->renderFrame(frame, cameraConfig);
}
} // namespace TMIV::Renderer
