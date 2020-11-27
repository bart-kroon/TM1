/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Graph.h>
#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/Engine.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cmath>

namespace TMIV::Renderer {
namespace {
template <typename MAT>
auto textureGather(const MAT &m, const Common::Vec2f &p)
    -> Common::stack::Vec4<typename MAT::value_type> {
  Common::stack::Vec4<typename MAT::value_type> fetchedValues;

  int w_last = static_cast<int>(m.width()) - 1;
  int h_last = static_cast<int>(m.height()) - 1;

  int x0 = std::clamp(static_cast<int>(std::floor(p.x() - 0.5F)), 0, w_last);
  int y0 = std::clamp(static_cast<int>(std::floor(p.y() - 0.5F)), 0, h_last);

  int x1 = std::min(x0 + 1, w_last);
  int y1 = std::min(y0 + 1, h_last);

  fetchedValues[0] = m(y1, x0);
  fetchedValues[1] = m(y1, x1);
  fetchedValues[2] = m(y0, x1);
  fetchedValues[3] = m(y0, x0);

  return fetchedValues;
}

void insertWeightedDepthInStack(std::vector<Common::Vec2f> &stack, float weight, float z,
                                float blendingFactor) {
  if (0.F < weight) {
    for (size_t i = 0; i <= stack.size(); i++) {
      float zAfter =
          (i < stack.size()) ? (stack[i].x() / stack[i].y()) : std::numeric_limits<float>::max();

      if (z < zAfter) {
        float zBefore = (0 < i) ? (stack[i - 1].x() / stack[i - 1].y()) : -1.F;

        bool mergeBefore = (z - zBefore) < (zBefore * blendingFactor);
        bool mergeAfter = (zAfter - z) < (z * blendingFactor);

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
  std::vector<size_t> outputList;

  for (size_t id = 0; id < inputList.size(); id++) {
    if (inputList[id]) {
      outputList.emplace_back(id);
    }
  }

  return outputList;
}
} // namespace

class ViewWeightingSynthesizer::Impl {
private:
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

  float m_angularScaling = 1.5F;
  float m_minimalWeight = 2.5F;
  float m_stretchFactor = 100.F;
  float m_blendingFactor = 0.03F;
  float m_overloadFactor = 2.F;
  int m_filteringPass = 1;
  int viewIdInpainted = -1;

public:
  explicit Impl(const Common::Json &componentNode) {
    m_angularScaling = componentNode.require("angularScaling").as<float>();
    m_minimalWeight = componentNode.require("minimalWeight").as<float>();
    m_stretchFactor = componentNode.require("stretchFactor").as<float>();
    m_blendingFactor = componentNode.require("blendingFactor").as<float>();
    m_overloadFactor = componentNode.require("overloadFactor").as<float>();
    m_filteringPass = componentNode.require("filteringPass").as<int>();
  }

  Impl(float angularScaling, float minimalWeight, float stretchFactor, float blendingFactor,
       float overloadFactor, int filteringPass) {
    m_angularScaling = angularScaling;
    m_minimalWeight = minimalWeight;
    m_stretchFactor = stretchFactor;
    m_blendingFactor = blendingFactor;
    m_overloadFactor = overloadFactor;
    m_filteringPass = filteringPass;
  }

  auto renderFrame(const Decoder::AccessUnit &frame, const MivBitstream::ViewParams &viewportParams)
      -> Common::Texture444Depth16Frame {
    const auto &viewParamsList = frame.viewParamsList;
    const auto sourceHelperList = ProjectionHelperList{viewParamsList};
    const auto targetHelper = ProjectionHelper{viewportParams};

    // 0) Initialization
    findInpaintedView(frame);
    computeCameraWeight(sourceHelperList, targetHelper);
    computeCameraVisibility(sourceHelperList, targetHelper);
    computeAngularDistortionPerSource(sourceHelperList);

    // 1) Deconstruction
    recoverPrunedSource(frame, sourceHelperList);

    // 2) Reprojection
    reprojectPrunedSource(frame, sourceHelperList, targetHelper);

    // 3) Warping
    warpPrunedSource(frame, targetHelper);

    // 4) Weight recovery
    recoverPrunedWeight(sourceHelperList, targetHelper);

    // 5) Selection
    selectViewportDepth(!frame.vps.vps_miv_extension().vme_depth_low_quality_flag(), targetHelper);

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
          m_viewportVisibility[i] =
              std::clamp(1.F / m_viewportVisibility[i], viewportParams.dq.dq_norm_disp_low(),
                         viewportParams.dq.dq_norm_disp_high());
        }
      }
    }

    auto viewport = Common::Texture444Depth16Frame{
        quantizeTexture(m_viewportColor),
        MivBitstream::DepthTransform<16>{viewportParams.dq}.quantizeNormDisp(m_viewportVisibility,
                                                                             1)};
    viewport.first.filIInvalidWithNeutral(viewport.second);

    return viewport;
  }

private:
  void findInpaintedView(const Decoder::AccessUnit &frame) {
    viewIdInpainted = -1;
    for (const auto &atlas : frame.atlas) {
      for (const auto &patchParams : atlas.patchParamsList) {
        if (patchParams.atlasPatchInpaintFlag()) {
          int idx = int(patchParams.atlasPatchProjectionId());
          // support for single inpainted view
          assert(viewIdInpainted == -1 || viewIdInpainted == idx);
          viewIdInpainted = idx;
        }
      }
    }
  }

  void computeCameraWeight(const ProjectionHelperList &sourceHelperList,
                           const ProjectionHelper &targetHelper) {
    auto isTridimensional = [&]() -> bool {
      constexpr auto epsilon = 1e-2F;
      Common::Mat3x3f M{Common::Mat3x3f::zeros()};
      Common::Mat3x3f N;

      for (const auto &helper : sourceHelperList) {
        M += matprod(helper.getViewParams().ce.position(), 'N',
                     helper.getViewParams().ce.position(), 'T', N);
      }

      return (epsilon < det(M));
    };

    if (1 < sourceHelperList.size()) {
      // Distance to each source axis
      bool is3D = isTridimensional();

      const Common::Vec3f &viewportPosition = targetHelper.getViewingPosition();
      std::vector<float> cameraDistance;

      for (size_t viewId = 0U; viewId != sourceHelperList.size(); ++viewId) {

        // inpainted view gets large distance to yield low weight
        if (int(viewId) == viewIdInpainted) {
          const float largeDistance = 1e3;
          cameraDistance.push_back(largeDistance);
          continue;
        }

        const auto &helper = sourceHelperList[viewId];
        const Common::Vec3f &cameraPosition = helper.getViewingPosition();
        Common::Vec3f cameraDirection = helper.getViewingDirection();

        cameraDistance.push_back(
            is3D ? norm(cameraPosition - viewportPosition)
                 : norm(cross(cameraPosition - viewportPosition, cameraDirection)));
      }

      // Camera sorting
      std::vector<unsigned> closestCamera(cameraDistance.size());

      iota(closestCamera.begin(), closestCamera.end(), 0);
      std::sort(closestCamera.begin(), closestCamera.end(), [&](unsigned i1, unsigned i2) {
        return (cameraDistance[i1] < cameraDistance[i2]);
      });

      // Reference distance
      float refDistance = 0.F;

      for (size_t id = 1; refDistance <= std::numeric_limits<float>::epsilon(); id++) {
        refDistance = norm(sourceHelperList[closestCamera[0]].getViewingPosition() -
                           sourceHelperList[closestCamera[id]].getViewingPosition()) *
                      0.25F;
      }

      // Weighting
      m_cameraWeight.clear();

      for (float id : cameraDistance) {
        float w = 1.F / (1.F + Common::sqr(id / refDistance));
        m_cameraWeight.emplace_back(w);
      }
    } else {
      m_cameraWeight = {1.F};
    }
  }

  void computeCameraVisibility(const ProjectionHelperList &sourceHelperList,
                               const ProjectionHelper &targetHelper) {
    const unsigned N = 4;
    const Common::Vec2f depthRange = {0.5F, 10.F};

    std::vector<Common::Vec3f> pointCloud;
    float x = 0.F;
    float step = 1.F / static_cast<float>(N - 1);

    for (unsigned i = 0; i < N; i++) {
      float y = 0.F;

      float px = x * static_cast<float>(targetHelper.getViewParams().ci.projectionPlaneSize().x());

      for (unsigned j = 0; j < N; j++) {
        float py =
            y * static_cast<float>(targetHelper.getViewParams().ci.projectionPlaneSize().y());

        pointCloud.push_back(targetHelper.doUnprojection({px, py}, depthRange.x()));
        pointCloud.push_back(targetHelper.doUnprojection({px, py}, depthRange.y()));

        y += step;
      }

      x += step;
    }

    m_cameraVisibility.clear();

    for (size_t viewId = 0; viewId < sourceHelperList.size(); viewId++) {
      const auto &helper = sourceHelperList[viewId];
      unsigned K = 0;

      for (const Common::Vec3f &P : pointCloud) {
        auto p = helper.doProjection(P);

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

    for (size_t viewId = 0; viewId < sourceHelperList.size(); viewId++) {
      if (m_cameraVisibility[viewId]) {
        m_cameraDistortion[viewId] =
            m_angularScaling *
            static_cast<float>(Common::deg2rad(
                2. / Common::pps2ppd(sourceHelperList[viewId].getAngularResolution())));
      }
    }
  }

  void recoverPrunedSource(const Decoder::AccessUnit &frame,
                           const ProjectionHelperList &sourceHelperList) {
    // Recover pruned views
    const auto [prunedViews, prunedMasks] = recoverPrunedViewAndMask(frame);

    // Expand pruned views
    m_sourceColor.clear();
    m_sourceDepth.clear();

    for (size_t sourceId = 0; sourceId < prunedViews.size(); sourceId++) {
      const auto &viewParams = sourceHelperList[sourceId].getViewParams();

      m_sourceColor.emplace_back(expandTexture(prunedViews[sourceId].first));
      m_sourceDepth.emplace_back(MivBitstream::DepthTransform<10>{viewParams.dq}.expandDepth(
          prunedViews[sourceId].second));

      std::transform(
          prunedMasks[sourceId].getPlane(0).begin(), prunedMasks[sourceId].getPlane(0).end(),
          m_sourceDepth.back().begin(), m_sourceDepth.back().begin(),
          [&](auto maskValue, float depthValue) { return 0 < maskValue ? depthValue : NAN; });
    }
  }

  void reprojectPrunedSource(const Decoder::AccessUnit &frame,
                             const ProjectionHelperList &sourceHelperList,
                             const ProjectionHelper &targetHelper) {
    m_sourceUnprojection.resize(m_sourceDepth.size());
    m_sourceReprojection.resize(m_sourceDepth.size());
    m_sourceRayDirection.resize(m_sourceDepth.size());

    for (size_t sourceId = 0; sourceId < m_sourceDepth.size(); sourceId++) {
      m_sourceUnprojection[sourceId].resize(m_sourceDepth[sourceId].height(),
                                            m_sourceDepth[sourceId].width());
      std::fill(m_sourceUnprojection[sourceId].begin(), m_sourceUnprojection[sourceId].end(),
                Common::Vec3f{NAN, NAN, NAN});

      m_sourceReprojection[sourceId].resize(m_sourceDepth[sourceId].height(),
                                            m_sourceDepth[sourceId].width());
      std::fill(m_sourceReprojection[sourceId].begin(), m_sourceReprojection[sourceId].end(),
                std::pair{Common::Vec2f{NAN, NAN}, NAN});

      m_sourceRayDirection[sourceId].resize(m_sourceDepth[sourceId].height(),
                                            m_sourceDepth[sourceId].width());
      std::fill(m_sourceRayDirection[sourceId].begin(), m_sourceRayDirection[sourceId].end(),
                Common::Vec3f{NAN, NAN, NAN});
    }

    for (const auto &atlas : frame.atlas) {
      Common::parallel_for(
          atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height(), [&](size_t Y, size_t X) {
            const auto patchId = atlas.patchId(static_cast<int>(Y), static_cast<int>(X));
            if (patchId == Common::unusedPatchId) {
              return;
            }

            const auto &patchParams = atlas.patchParamsList[patchId];
            const auto viewId = patchParams.atlasPatchProjectionId();

            if (!m_cameraVisibility[viewId]) {
              return;
            }

            const auto sourceViewPos =
                patchParams.atlasToView({static_cast<int>(X), static_cast<int>(Y)});
            const auto x = sourceViewPos.x();
            const auto y = sourceViewPos.y();

            // temporary use only view dimensions
            if (y >= static_cast<int>(m_sourceDepth[viewId].height()) ||
                x >= static_cast<int>(m_sourceDepth[viewId].width())) {
              return;
            }

            const auto d = m_sourceDepth[viewId](y, x);

            if (!sourceHelperList[viewId].isValidDepth(d)) {
              return;
            }

            const auto P = sourceHelperList[viewId].doUnprojection(
                {static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F}, d);
            const auto p = targetHelper.doProjection(P);

            if (isValidDepth(p.second) && targetHelper.isInsideViewport(p.first)) {
              m_sourceUnprojection[viewId](y, x) = P;
              m_sourceReprojection[viewId](y, x) = p;
              m_sourceRayDirection[viewId](y, x) =
                  unit(P - targetHelper.getViewParams().ce.position());
            }
          });
    }
  }

  void warpPrunedSource(const Decoder::AccessUnit &frame, const ProjectionHelper &targetHelper) {
    struct Splat {
      Common::Vec2f center{};
      Common::Vec2f firstAxis{};
      Common::Vec2f secondAxis{};
      float pointSize{};
    };

    auto getSplatParameters = [&](unsigned viewId, int x, int y,
                                  const std::pair<Common::Vec2f, float> &P) -> Splat {
      static const std::array<Common::Vec2i, 8> offsetList = {
          Common::Vec2i({1, 0}),  Common::Vec2i({1, 1}),  Common::Vec2i({0, 1}),
          Common::Vec2i({-1, 1}), Common::Vec2i({-1, 0}), Common::Vec2i({-1, -1}),
          Common::Vec2i({0, -1}), Common::Vec2i({1, -1})};

      int w_last = static_cast<int>(m_sourceReprojection[viewId].width()) - 1;
      int h_last = static_cast<int>(m_sourceReprojection[viewId].height()) - 1;

      std::array<std::pair<Common::Vec2f, float>, 8> Q;
      std::array<float, 8> W{};
      float WT{0.F};

      // Center
      Common::Vec2f C{0.F, 0.F};

      auto OP = m_sourceRayDirection[viewId](y, x);

      for (size_t i = 0U; i < offsetList.size(); i++) {
        int xo = std::clamp(x + offsetList[i].x(), 0, w_last);
        int yo = std::clamp(y + offsetList[i].y(), 0, h_last);

        Q[i] = m_sourceReprojection[viewId](yo, xo);

        if (isValidDepth(Q[i].second)) {
          auto OQ = m_sourceRayDirection[viewId](yo, xo);

          float a = std::acos(dot(OP, OQ)) / m_cameraDistortion[viewId];
          float wi = std::exp(-a * a);

          W[i] = wi;
          C += wi * Q[i].first;
          WT += wi;
        }
      }

      if (WT < m_minimalWeight) {
        return {Common::Vec2f{0.F, 0.F}, Common::Vec2f{0.F, 0.F}, Common::Vec2f{0.F, 0.F}, 0.F};
      }

      // Axis (requires at least 5 good candidates)
      if (0.F < WT) {
        Common::Mat2x2f M{0.F, 0.F, 0.F, 0.F};

        C /= WT;

        for (size_t i = 0U; i < offsetList.size(); i++) {
          if (isValidDepth(Q[i].second)) {
            Common::Vec2f dp = (Q[i].first - C);
            M += W[i] * Common::stack::Mat2x2<float>{dp.x() * dp.x(), dp.x() * dp.y(),
                                                     dp.x() * dp.y(), dp.y() * dp.y()};
          }
        }

        float b = M[0] + M[3];               // trace
        float c = M[0] * M[3] - M[1] * M[2]; // determinant
        float delta = (b * b - 4.F * c);

        if ((0.F < c) && (0. < delta)) {
          float sqrt_delta = std::sqrt(delta);
          float l1 = 0.5F * (b + sqrt_delta);
          float l2 = 0.5F * (b - sqrt_delta);

          if (0.F < l2) {
            Common::Vec2f e1{1.F, 0.F};
            Common::Vec2f e2{0.F, 1.F};

            if (l1 != l2) {
              Common::Vec2f u1{M(0, 0) - l2, M(1, 0)};
              Common::Vec2f u2{M(0, 1), M(1, 1) - l2};

              e1 = (0.F < dot(u1, u1)) ? unit(u1) : unit(u2);
              e2 = Common::Vec2f{-e1.y(), e1.x()};
            }

            float r1 = std::sqrt(2.F * l1 / WT);
            float r2 = std::sqrt(2.F * l2 / WT);

            if (r1 < m_stretchFactor) {
              return {P.first, r1 * e1, r2 * e2, 2.F * r1};
            }
          }
        }
      }

      return {P.first, Common::Vec2f{0.F, 0.F}, Common::Vec2f{0.F, 0.F}, 1.F};
    };

    auto rasterizePoint = [&](unsigned viewId, const Splat &splat, const Common::Vec3f &P,
                              float depthValue) {
      // Initialization
      int w_last = static_cast<int>(m_viewportDepth[viewId].width()) - 1;
      int h_last = static_cast<int>(m_viewportDepth[viewId].height()) - 1;

      float R1 = dot(splat.firstAxis, splat.firstAxis);
      float R2 = dot(splat.secondAxis, splat.secondAxis);
      float radius = 0.5F * splat.pointSize;

      // Bounding box
      float xLow = std::max(0.F, splat.center.x() - radius);
      float xHigh = splat.center.x() + radius;
      float yLow = std::max(0.F, splat.center.y() - radius);
      float yHigh = splat.center.y() + radius;
      int x0 = std::max(0, static_cast<int>(std::floor(xLow)));
      int x1 = std::min(w_last, static_cast<int>(std::ceil(xHigh)));
      int y0 = std::max(0, static_cast<int>(std::floor(yLow)));
      int y1 = std::min(h_last, static_cast<int>(std::ceil(yHigh)));

      // Looping on all pixels within the bounding box
      for (int y = y0; y <= y1; y++) {
        float dy = ((static_cast<float>(y) + 0.5F) - splat.center.y());

        for (int x = x0; x <= x1; x++) {
          float dx = ((static_cast<float>(x) + 0.5F) - splat.center.x());
          float depthRef = m_viewportDepth[viewId](y, x);

          if (!isValidDepth(depthRef) || (depthValue < depthRef)) {
            if (0.F < R1) {
              Common::Vec2f dp{dx, dy};

              float f1 = std::abs(dot(splat.firstAxis, dp));
              float f2 = std::abs(dot(splat.secondAxis, dp));

              if ((f1 <= R1) && (f2 <= R2)) {
                m_viewportUnprojection[viewId](y, x) = P;
                m_viewportDepth[viewId](y, x) = depthValue;
              }
            } else {
              m_viewportUnprojection[viewId](y, x) = P;
              m_viewportDepth[viewId](y, x) = depthValue;
            }
          }
        }
      }
    };

    m_viewportUnprojection.resize(m_sourceDepth.size());
    m_viewportDepth.resize(m_sourceDepth.size());

    for (size_t viewId = 0; viewId < m_sourceDepth.size(); viewId++) {
      if (m_cameraVisibility[viewId]) {
        m_viewportUnprojection[viewId].resize(
            targetHelper.getViewParams().ci.projectionPlaneSize().y(),
            targetHelper.getViewParams().ci.projectionPlaneSize().x());
        std::fill(m_viewportUnprojection[viewId].begin(), m_viewportUnprojection[viewId].end(),
                  Common::Vec3f{NAN, NAN, NAN});

        m_viewportDepth[viewId].resize(targetHelper.getViewParams().ci.projectionPlaneSize().y(),
                                       targetHelper.getViewParams().ci.projectionPlaneSize().x());
        std::fill(m_viewportDepth[viewId].begin(), m_viewportDepth[viewId].end(), NAN);
      }
    }

    auto visibleSourceId = getEnabledIdList(m_cameraVisibility);

    Common::parallel_for(visibleSourceId.size(), [&](size_t id) {
      auto viewId = static_cast<unsigned>(visibleSourceId[id]);

      for (const auto &atlas : frame.atlas) {
        for (const auto &patchParams : atlas.patchParamsList) {
          if (patchParams.atlasPatchProjectionId() != visibleSourceId[id]) {
            continue;
          }

          const auto x0 = patchParams.atlasPatch3dOffsetU();
          const auto x1 = x0 + patchParams.atlasPatch3dSizeU();

          const auto y0 = patchParams.atlasPatch3dOffsetV();
          const auto y1 = y0 + patchParams.atlasPatch3dSizeV();

          for (auto y = y0; y < y1; y++) {
            for (auto x = x0; x < x1; x++) {
              // temporary use only view dimensions
              if (y >= m_sourceReprojection[viewId].height() ||
                  x >= m_sourceReprojection[viewId].width()) {
                continue;
              }

              auto P = m_sourceReprojection[viewId](y, x);

              if (!isValidDepth(P.second)) {
                continue;
              }

              auto splatParameters = getSplatParameters(viewId, x, y, P);

              if (0.F < splatParameters.pointSize) {
                rasterizePoint(viewId, getSplatParameters(viewId, x, y, P),
                               m_sourceUnprojection[viewId](y, x), P.second);
              }
            }
          }
        }
      }
    });
  }

  void recoverPrunedWeight(const ProjectionHelperList &sourceHelperList,
                           const ProjectionHelper &targetHelper) {
    // Retrieve pruning information
    auto hasPruningRelation =
        any_of(sourceHelperList.begin(), sourceHelperList.end(), [](const auto &helper) {
          const auto &viewParams = helper.getViewParams();
          return viewParams.pp && !viewParams.pp->pp_is_root_flag();
        });

    // Weight recovery
    m_viewportWeight.resize(sourceHelperList.size());

    for (size_t viewId = 0; viewId < m_viewportWeight.size(); viewId++) {
      m_viewportWeight[viewId].resize(m_viewportDepth[viewId].height(),
                                      m_viewportDepth[viewId].width());
      std::fill(m_viewportWeight[viewId].begin(), m_viewportWeight[viewId].end(),
                hasPruningRelation ? 0.F : m_cameraWeight[viewId]);
    }

    if (hasPruningRelation) {
      // Pruning graph (from children to parent)
      Common::Graph::BuiltIn::Sparse<float> pruningGraph(sourceHelperList.size());

      for (size_t nodeId = 0; nodeId < sourceHelperList.size(); nodeId++) {
        const auto &viewParams = sourceHelperList[nodeId].getViewParams();

        if (viewParams.pp) {
          for (auto parentId : *viewParams.pp) {
            pruningGraph.connect(nodeId, static_cast<size_t>(parentId), 1.F,
                                 Common::Graph::LinkType::Directed);
          }
        }
      }

      // Pruning order
      auto pruningOrderId = getDescendingOrderId(pruningGraph);

      // Recovery
      Common::parallel_for(
          targetHelper.getViewParams().ci.projectionPlaneSize().x(),
          targetHelper.getViewParams().ci.projectionPlaneSize().y(), [&](size_t y, size_t x) {
            for (auto prunedNodeId : pruningOrderId) {
              if (m_cameraVisibility[prunedNodeId]) {
                auto zPruned = m_viewportDepth[prunedNodeId](y, x);

                // Retrieve candidate
                std::queue<Common::Graph::NodeId> nodeQueue;

                for (const auto &linkToParent : pruningGraph.getNeighbourhood(prunedNodeId)) {
                  nodeQueue.push(linkToParent.node());
                }

                int w_last = static_cast<int>(m_sourceDepth[prunedNodeId].width()) - 1;
                int h_last = static_cast<int>(m_sourceDepth[prunedNodeId].height()) - 1;

                std::vector<std::pair<Common::Graph::NodeId, float>> candidateList;

                while (!nodeQueue.empty()) {
                  auto unprunedNodeId = nodeQueue.front();

                  if (m_cameraVisibility[unprunedNodeId]) {
                    auto zUnpruned = m_viewportDepth[unprunedNodeId](y, x);

                    if (isValidDepth(zUnpruned) &&
                        (!isValidDepth(zPruned) ||
                         ((m_blendingFactor * zUnpruned) < (zPruned - zUnpruned)))) {
                      candidateList.emplace_back(unprunedNodeId, zUnpruned);
                    }
                  }

                  for (const auto &linkToParent : pruningGraph.getNeighbourhood(unprunedNodeId)) {
                    nodeQueue.push(linkToParent.node());
                  }

                  nodeQueue.pop();
                }

                std::sort(candidateList.begin(), candidateList.end(),
                          [](const auto &p1, const auto &p2) { return (p1.second < p2.second); });

                // Find best
                const auto &prunedHelper = sourceHelperList[prunedNodeId];
                Common::Graph::NodeId representativeNodeId = prunedNodeId;

                for (const auto &candidate : candidateList) {
                  auto p = prunedHelper.doProjection(m_viewportUnprojection[candidate.first](y, x));

                  if (isValidDepth(p.second) && prunedHelper.isInsideViewport(p.first)) {
                    static const std::array<Common::Vec2i, 9> offsetList = {
                        Common::Vec2i({-1, -1}), Common::Vec2i({0, -1}), Common::Vec2i({1, -1}),
                        Common::Vec2i({-1, 0}),  Common::Vec2i({0, 0}),  Common::Vec2i({1, 0}),
                        Common::Vec2i({-1, 1}),  Common::Vec2i({0, 1}),  Common::Vec2i({1, 1})};

                    auto X = static_cast<int>(std::floor(p.first.x()));
                    auto Y = static_cast<int>(std::floor(p.first.y()));

                    for (const auto &offset : offsetList) {
                      int xo = std::clamp(X + offset.x(), 0, w_last);
                      int yo = std::clamp(Y + offset.y(), 0, h_last);

                      float zOnPruned = m_sourceDepth[prunedNodeId](yo, xo);

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

                m_viewportWeight[representativeNodeId](y, x) += m_cameraWeight[prunedNodeId];
              }
            }
          });
    }
  }

  void selectViewportDepth(bool trustDepth, const ProjectionHelper &targetHelper) {
    m_viewportVisibility.resize(targetHelper.getViewParams().ci.projectionPlaneSize().y(),
                                targetHelper.getViewParams().ci.projectionPlaneSize().x());

    Common::parallel_for(
        m_viewportVisibility.width(), m_viewportVisibility.height(), [&](size_t y, size_t x) {
          std::vector<Common::Vec2f> stack;

          for (size_t viewId = 0; viewId < m_viewportDepth.size(); viewId++) {
            if (m_cameraVisibility[viewId] && int(viewId) != viewIdInpainted) {
              float z = m_viewportDepth[viewId](y, x);

              if (isValidDepth(z)) {
                insertWeightedDepthInStack(stack, m_viewportWeight[viewId](y, x), z,
                                           m_blendingFactor);
              }
            }
          }

          // Select best candidate
          Common::Vec2f bestCandidate = Common::Vec2f({0.F, 0.F});

          for (const auto &v : stack) {
            if ((bestCandidate.y() < v.y()) && ((bestCandidate.y() * m_overloadFactor) < v.y())) {
              bestCandidate = v;
            }

            if (trustDepth) {
              break;
            }
          }

          m_viewportVisibility(y, x) =
              (0.f < bestCandidate.y()) ? (bestCandidate.x() / bestCandidate.y()) : 0.F;
        });
  }

  void filterVisibilityMap() {
    static const std::array<Common::Vec2i, 9> offsetList = {
        Common::Vec2i({-1, -1}), Common::Vec2i({0, -1}), Common::Vec2i({1, -1}),
        Common::Vec2i({-1, 0}),  Common::Vec2i({0, 0}),  Common::Vec2i({1, 0}),
        Common::Vec2i({-1, 1}),  Common::Vec2i({0, 1}),  Common::Vec2i({1, 1})};

    static Common::Mat<float> flipVisibility;

    std::reference_wrapper<Common::Mat<float>> firstWrapper =
        ((m_filteringPass % 2) != 0) ? flipVisibility : m_viewportVisibility;
    std::reference_wrapper<Common::Mat<float>> secondWrapper =
        ((m_filteringPass % 2) != 0) ? m_viewportVisibility : flipVisibility;

    size_t w = m_viewportVisibility.width();
    size_t h = m_viewportVisibility.height();

    int w_last = static_cast<int>(w) - 1;
    int h_last = static_cast<int>(h) - 1;

    flipVisibility.resize(m_viewportVisibility.sizes());

    if ((m_filteringPass % 2) != 0) {
      std::copy(m_viewportVisibility.begin(), m_viewportVisibility.end(), flipVisibility.begin());
    }

    for (int iter = 0U; iter < m_filteringPass; iter++) {
      const Common::Mat<float> &firstDepth = firstWrapper.get();
      Common::Mat<float> &secondDepth = secondWrapper.get();

      Common::parallel_for(w, h, [&](size_t y, size_t x) {
        std::array<float, 9> depthBuffer{};

        for (size_t i = 0; i < depthBuffer.size(); i++) {
          int xo = std::clamp(static_cast<int>(x) + offsetList[i].x(), 0, w_last);
          int yo = std::clamp(static_cast<int>(y) + offsetList[i].y(), 0, h_last);

          float z = firstDepth(yo, xo);

          depthBuffer[i] = isValidDepth(z) ? z : 0.F;
        }

        std::sort(depthBuffer.begin(), depthBuffer.end());

        for (size_t i = 4; i < 6; i++) {
          if (0.F < depthBuffer[i]) {
            secondDepth(y, x) = depthBuffer[i];
            return;
          }
        }

        secondDepth(y, x) = 0.F;
      });

      swap(firstWrapper, secondWrapper);
    }
  }

  void computeShadingMap(const ProjectionHelperList &sourceHelperList,
                         const ProjectionHelper &targetHelper) {
    auto isProneToGhosting = [&](unsigned sourceId, const std::pair<Common::Vec2f, float> &p,
                                 const Common::Vec3f &OP) -> bool {
      static const std::array<Common::Vec2i, 4> offsetList = {
          Common::Vec2i({1, 0}), Common::Vec2i({-1, 0}), Common::Vec2i({0, 1}),
          Common::Vec2i({0, -1})};

      int w_last = static_cast<int>(m_sourceDepth[sourceId].width()) - 1;
      int h_last = static_cast<int>(m_sourceDepth[sourceId].height()) - 1;

      int x = static_cast<int>(std::floor(p.first.x()));
      int y = static_cast<int>(std::floor(p.first.y()));

      return std::any_of(offsetList.cbegin(), offsetList.cend(), [&](const auto &offset) {
        int xo = std::clamp(x + offset.x(), 0, w_last);
        int yo = std::clamp(y + offset.y(), 0, h_last);

        float z = m_sourceDepth[sourceId](yo, xo);

        if (!sourceHelperList[sourceId].isValidDepth(z)) {
          return true;
        }

        auto OQ = m_sourceRayDirection[sourceId](yo, xo);
        return 2.F * m_cameraDistortion[sourceId] < std::abs(std::acos(dot(OP, OQ)));
      });
    };

    static const std::array<Common::Vec2i, 9> offsetList = {
        Common::Vec2i({0, 0}),   Common::Vec2i({1, 0}),  Common::Vec2i({1, 1}),
        Common::Vec2i({0, -1}),  Common::Vec2i({1, -1}), Common::Vec2i({0, 1}),
        Common::Vec2i({-1, -1}), Common::Vec2i({-1, 0}), Common::Vec2i({-1, 1})};

    static const std::array<float, 9> d2 = {0.F, 1.F, 2.F, 1.F, 2.F, 1.F, 2.F, 1.F, 2.F};

    int w_last = static_cast<int>(m_viewportVisibility.width()) - 1;
    int h_last = static_cast<int>(m_viewportVisibility.height()) - 1;

    m_viewportColor.resize(targetHelper.getViewParams().ci.projectionPlaneSize().y(),
                           targetHelper.getViewParams().ci.projectionPlaneSize().x());

    Common::parallel_for(
        m_viewportVisibility.width(), m_viewportVisibility.height(), [&](size_t y, size_t x) {
          Common::Vec2f pn1{static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F};
          bool missingData = !isValidDepth(m_viewportVisibility(y, x));

          if (missingData && viewIdInpainted >= 0) {

            auto &backgroundDepth = m_viewportDepth[viewIdInpainted];
            int W = static_cast<int>(backgroundDepth.width());
            int H = static_cast<int>(backgroundDepth.height());

            auto z = backgroundDepth(y, x);

            if (isValidDepth(z)) {
              auto P = targetHelper.doUnprojection(pn1, z);

              auto [uvBg, zBg] = sourceHelperList[viewIdInpainted].doProjection(P);

              // nearest neighbour fetching of low-res inpainted image
              int j = std::clamp(int(round(uvBg.x())), 0, W - 1);
              int i = std::clamp(int(round(uvBg.y())), 0, H - 1);
              auto inpaintedBackgroundColor = m_sourceColor[viewIdInpainted](i, j);
              m_viewportColor(y, x) = inpaintedBackgroundColor;
            }
            m_viewportVisibility(y, x) = z;

          } else {

            Common::Vec3f oColor{};
            float oWeight = 0.F;

            std::vector<Common::Vec2f> stack;

            for (size_t i = 0U; i < offsetList.size(); i++) {
              int xo = std::clamp(static_cast<int>(x) + offsetList[i].x(), 0, w_last);
              int yo = std::clamp(static_cast<int>(y) + offsetList[i].y(), 0, h_last);

              float z = m_viewportVisibility(yo, xo);

              if (isValidDepth(z)) {
                float ksi = 1.F / (1.F + d2[i]);
                insertWeightedDepthInStack(stack, ksi, z, m_blendingFactor);
              }
            }

            const auto &O = targetHelper.getViewingPosition();
            bool isOnViewportContour = (1U < stack.size());

            for (const auto &element : stack) {
              float z = element.x() / element.y();

              auto P = targetHelper.doUnprojection(pn1, z);
              auto OP = unit(P - O);

              for (size_t sourceId = 0U; sourceId < sourceHelperList.size(); sourceId++) {
                if (m_cameraVisibility[sourceId] && int(sourceId) != viewIdInpainted) {
                  auto pn2 = sourceHelperList[sourceId].doProjection(P);

                  if (isValidDepth(pn2.second) &&
                      sourceHelperList[sourceId].isInsideViewport(pn2.first)) {
                    if (isOnViewportContour ||
                        !isProneToGhosting(static_cast<unsigned>(sourceId), pn2, OP)) {
                      auto zRef = textureGather(m_sourceDepth[sourceId], pn2.first);
                      auto cRef = textureGather(m_sourceColor[sourceId], pn2.first);

                      Common::Vec2f q = {pn2.first.x() - 0.5F, pn2.first.y() - 0.5F};
                      Common::Vec2f f = {q.x() - std::floor(q.x()), q.y() - std::floor(q.y())};
                      Common::Vec2f fb = {1.F - f.x(), 1.F - f.y()};

                      Common::Vec4f wColor{fb.x() * f.y(), f.x() * f.y(), f.x() * fb.y(),
                                           fb.x() * fb.y()};

                      for (unsigned j = 0; j < 4U; j++) {
                        if (sourceHelperList[sourceId].isValidDepth(zRef[j])) {
                          float nu = (zRef[j] - pn2.second) / (pn2.second * m_blendingFactor);
                          float wDepth = element.y() / (1.F + nu * nu);

                          float w = (m_cameraWeight[sourceId] * wColor[j] * wDepth);

                          oColor += w * cRef[j];
                          oWeight += w;
                        }
                      }
                    }
                  }
                }
              }
            }

            static const float eps = 1e-3F;

            m_viewportColor(y, x) = (eps < oWeight) ? (oColor / oWeight)
                                                    : (isValidDepth(m_viewportVisibility(y, x))
                                                           ? Common::Vec3f{-1.F, -1.F, -1.F}
                                                           : Common::Vec3f{});
          }
        });
  }
}; // namespace TMIV::Renderer

ViewWeightingSynthesizer::ViewWeightingSynthesizer(const Common::Json & /*rootNode*/,
                                                   const Common::Json &componentNode)
    : m_impl(new Impl{componentNode}) {}

ViewWeightingSynthesizer::ViewWeightingSynthesizer(float angularScaling, float minimalWeight,
                                                   float stretchFactor, float blendingFactor,
                                                   float overloadFactor, int filteringPass)
    : m_impl(new Impl(angularScaling, minimalWeight, stretchFactor, blendingFactor, overloadFactor,
                      filteringPass)) {}

ViewWeightingSynthesizer::~ViewWeightingSynthesizer() = default;

auto ViewWeightingSynthesizer::renderFrame(const Decoder::AccessUnit &frame,
                                           const MivBitstream::ViewParams &viewportParams) const
    -> Common::Texture444Depth16Frame {
  return m_impl->renderFrame(frame, viewportParams);
}
} // namespace TMIV::Renderer
