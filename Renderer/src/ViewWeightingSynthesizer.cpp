/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Graph.h>
#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/Image/Image.h>
#include <TMIV/Metadata/DepthOccupancyTransform.h>
#include <TMIV/Renderer/Engine.h>
#include <TMIV/Renderer/ViewWeightingSynthesizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace TMIV::Common;
using namespace TMIV::Common::Graph;
using namespace TMIV::Metadata;
using namespace TMIV::Image;

namespace TMIV::Renderer {
////////////////////////////////////////////////////////////////////////////////
namespace {

template <typename MAT>
auto textureGather(const MAT &m, const Vec2f &p) -> stack::Vec4<typename MAT::value_type> {
  stack::Vec4<typename MAT::value_type> fetchedValues;

  int w_last = static_cast<int>(m.width()) - 1;
  int h_last = static_cast<int>(m.height()) - 1;

  int x0 = clamp(ifloor(p.x() - 0.5F), 0, w_last);
  int y0 = clamp(ifloor(p.y() - 0.5F), 0, h_last);

  int x1 = std::min(x0 + 1, w_last);
  int y1 = std::min(y0 + 1, h_last);

  fetchedValues[0] = m(y1, x0);
  fetchedValues[1] = m(y1, x1);
  fetchedValues[2] = m(y0, x1);
  fetchedValues[3] = m(y0, x0);

  return fetchedValues;
}

void insertWeightedDepthInStack(std::vector<Vec2f> &stack, float weight, float z,
                                float blendingFactor) {
  if (0.F < weight) {

    for (std::size_t i = 0; i <= stack.size(); i++) {
      float zAfter =
          (i < stack.size()) ? (stack[i].x() / stack[i].y()) : std::numeric_limits<float>::max();

      if (z < zAfter) {
        float zBefore = (0 < i) ? (stack[i - 1].x() / stack[i - 1].y()) : -1.F;

        bool mergeBefore = (z - zBefore) < (zBefore * blendingFactor);
        bool mergeAfter = (zAfter - z) < (z * blendingFactor);

        if (mergeBefore && mergeAfter) {
          stack[i - 1] += (weight * Vec2f({z, 1.F}) + stack[i]);

          for (std::size_t j = i; j < stack.size() - 1; j++) {
            stack[j] = stack[j + 1];
          }

          stack.pop_back();
        } else if (mergeBefore) {
          stack[i - 1] += weight * Vec2f({z, 1.F});
        } else if (mergeAfter) {
          stack[i] += weight * Vec2f({z, 1.F});
        } else {
          stack.push_back(Vec2f({0.F, 0.F}));

          for (std::size_t j = (stack.size() - 1); i < j; j--) {
            stack[j] = stack[j - 1];
          }

          stack[i] = weight * Vec2f({z, 1.F});
        }

        break;
      }
    }
  }
}

auto getEnabledIdList(const std::vector<bool> &inputList) -> std::vector<std::size_t> {
  std::vector<std::size_t> outputList;

  for (std::size_t id = 0; id < inputList.size(); id++) {
    if (inputList[id]) {
      outputList.emplace_back(id);
    }
  }

  return outputList;
}
} // namespace

////////////////////////////////////////////////////////////////////////////////
class ViewWeightingSynthesizer::Impl {
private:
  std::vector<float> m_cameraWeight;
  std::vector<bool> m_cameraVisibility;
  std::vector<float> m_cameraDistortion;
  std::vector<Mat<Vec3f>> m_sourceColor;
  std::vector<Mat<float>> m_sourceDepth;
  std::vector<Mat<Vec3f>> m_sourceUnprojection;
  std::vector<Mat<std::pair<Vec2f, float>>> m_sourceReprojection;
  std::vector<Mat<Vec3f>> m_sourceRayDirection;
  std::vector<Mat<Vec3f>> m_viewportUnprojection;
  std::vector<Mat<float>> m_viewportDepth;
  std::vector<Mat<float>> m_viewportWeight;
  Mat<float> m_viewportVisibility;
  Mat<Vec3f> m_viewportColor;

  float m_angularScaling = 1.5F;
  float m_minimalWeight = 2.5F;
  float m_stretchFactor = 100.F;
  float m_blendingFactor = 0.03F;
  float m_overloadFactor = 2.F;
  int m_filteringPass = 1;

public:
  explicit Impl(const TMIV::Common::Json &componentNode) {
    m_angularScaling = componentNode.require("angularScaling").asFloat();
    m_minimalWeight = componentNode.require("minimalWeight").asFloat();
    m_stretchFactor = componentNode.require("stretchFactor").asFloat();
    m_blendingFactor = componentNode.require("blendingFactor").asFloat();
    m_overloadFactor = componentNode.require("overloadFactor").asFloat();
    m_filteringPass = componentNode.require("filteringPass").asInt();
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
  template <typename SourceProjectionType, typename TargetProjectionType, typename MVD>
  auto renderFrame(const MVD &atlasList, const PatchIdMapList &maps,
                   const Metadata::IvSequenceParams &ivSequenceParams,
                   const Metadata::IvAccessUnitParams &ivAccessUnitParams,
                   const Metadata::ViewParams &targetCamera) -> Common::Texture444Depth16Frame {

    typename ProjectionHelper<SourceProjectionType>::List sourceHelperList{
        ivSequenceParams.viewParamsList};
    ProjectionHelper<TargetProjectionType> targetHelper{targetCamera};

    //######################################################################################
    // 0) Initialization
    computeCameraWeight<SourceProjectionType, TargetProjectionType>(sourceHelperList, targetHelper);
    computeCameraVisibility<SourceProjectionType, TargetProjectionType>(sourceHelperList,
                                                                        targetHelper);
    computeAngularDistortionPerSource<SourceProjectionType>(sourceHelperList);

    //######################################################################################
    // 1) Deconstruction
    recoverPrunedSource<MVD, SourceProjectionType>(
        atlasList, ivSequenceParams, *ivAccessUnitParams.atlasParamsList, sourceHelperList);

    //######################################################################################
    // 2) Reprojection
    reprojectPrunedSource<SourceProjectionType, TargetProjectionType>(
        maps, *ivAccessUnitParams.atlasParamsList, sourceHelperList, targetHelper);

    //######################################################################################
    // 3) Warping
    warpPrunedSource<TargetProjectionType>(*ivAccessUnitParams.atlasParamsList, targetHelper);

    //######################################################################################
    // 4) Weight recovery
    recoverPrunedWeight<SourceProjectionType, TargetProjectionType>(sourceHelperList, targetHelper);

    //######################################################################################
    // 5) Selection
    selectViewportDepth<TargetProjectionType>(!ivSequenceParams.depthLowQualityFlag, targetHelper);

    //######################################################################################
    // 6) Filtering
    filterVisibilityMap();

    //######################################################################################
    // 7) Shading
    computeShadingMap<SourceProjectionType, TargetProjectionType>(sourceHelperList, targetHelper);

    //######################################################################################
    // 8) Output
    for (std::size_t i = 0U; i < m_viewportColor.size(); i++) {
      if (isValidDepth(m_viewportVisibility[i])) {
        if (m_viewportColor[i].x() < 0.F) {
          m_viewportVisibility[i] = Common::NaN;
          m_viewportColor[i] = Vec3f{};
        } else {
          m_viewportVisibility[i] =
              std::clamp(1.F / m_viewportVisibility[i], targetCamera.normDispRange.x(),
                         targetCamera.normDispRange.y());
        }
      }
    }

    const auto depthTransform = DepthTransform<16>{targetCamera};
    auto frame = Texture444Depth16Frame{quantizeTexture(m_viewportColor),
                                        depthTransform.quantizeNormDisp(m_viewportVisibility, 1)};
    frame.first.filIInvalidWithNeutral(frame.second);
    return frame;

  }

private:
  template <typename SourceProjectionType, typename TargetProjectionType>
  void
  computeCameraWeight(const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList,
                      const ProjectionHelper<TargetProjectionType> &targetHelper) {

    auto isTridimensional = [&]() -> bool {
      constexpr auto epsilon = 1e-2F;
      Mat3x3f M{Mat3x3f::zeros()};
      Mat3x3f N;

      for (const auto &helper : sourceHelperList) {
        M += matprod(helper.getViewParams().position, 'N', helper.getViewParams().position, 'T', N);
      }

      return (epsilon < det(M));
    };

    if (1 < sourceHelperList.size()) {
      // Distance to each source axis
      bool is3D = isTridimensional();

      const Vec3f &viewportPosition = targetHelper.getViewingPosition();
      std::vector<float> cameraDistance;

      for (const auto &helper : sourceHelperList) {
        const Vec3f &cameraPosition = helper.getViewingPosition();
        Vec3f cameraDirection = helper.getViewingDirection();

        cameraDistance.push_back(
            is3D ? norm(cameraPosition - viewportPosition)
                 : norm(cross(cameraPosition - viewportPosition, cameraDirection)));
      }

      // Camera sorting
      std::vector<unsigned> closestCamera(cameraDistance.size());

      std::iota(closestCamera.begin(), closestCamera.end(), 0);
      std::sort(closestCamera.begin(), closestCamera.end(), [&](unsigned i1, unsigned i2) {
        return (cameraDistance[i1] < cameraDistance[i2]);
      });

      // Reference distance
      float refDistance = 0.F;

      for (std::size_t id = 1; refDistance <= std::numeric_limits<float>::epsilon(); id++) {
        refDistance = norm(sourceHelperList[closestCamera[0]].getViewingPosition() -
                           sourceHelperList[closestCamera[id]].getViewingPosition()) *
                      0.25F;
      }

      // Weighting
      m_cameraWeight.clear();

      for (float id : cameraDistance) {
        float w = 1.F / (1.F + sqr(id / refDistance));
        m_cameraWeight.emplace_back(w);
      }
    } else {
      m_cameraWeight = {1.F};
    }
  }
  template <typename SourceProjectionType, typename TargetProjectionType>
  void computeCameraVisibility(
      const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList,
      const ProjectionHelper<TargetProjectionType> &targetHelper) {
    const unsigned N = 4;
    const Vec2f depthRange = {0.5F, 10.F};

    std::vector<Vec3f> pointCloud;
    float x = 0.F;
    float step = 1.F / static_cast<float>(N - 1);

    for (unsigned i = 0; i < N; i++) {
      float y = 0.F;

      float px = x * static_cast<float>(targetHelper.getViewParams().size.x());

      for (unsigned j = 0; j < N; j++) {
        float py = y * static_cast<float>(targetHelper.getViewParams().size.y());

        pointCloud.push_back(targetHelper.doUnprojection({px, py}, depthRange.x()));
        pointCloud.push_back(targetHelper.doUnprojection({px, py}, depthRange.y()));

        y += step;
      }

      x += step;
    }

    m_cameraVisibility.clear();

    for (std::size_t viewId = 0; viewId < sourceHelperList.size(); viewId++) {

      const auto &helper = sourceHelperList[viewId];
      unsigned K = 0;

      for (const Vec3f &P : pointCloud) {
        auto p = helper.doProjection(P);

        if (isValidDepth(p.second) && helper.isInsideViewport(p.first)) {
          K++;
          break;
        }
      }

      m_cameraVisibility.emplace_back(0 < K);
    }
  }
  template <typename SourceProjectionType>
  void computeAngularDistortionPerSource(
      const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList) {

    m_cameraDistortion.resize(sourceHelperList.size(), 0.F);

    for (std::size_t viewId = 0; viewId < sourceHelperList.size(); viewId++) {
      if (m_cameraVisibility[viewId]) {
        m_cameraDistortion[viewId] =
            m_angularScaling * static_cast<float>(deg2rad(
                                   2. / pps2ppd(sourceHelperList[viewId].getAngularResolution())));
      }
    }
  }
  template <typename MVD>
  auto recoverPrunedViewAndMask(const MVD &atlas, const ViewParamsVector &viewParamsVector,
                                const AtlasParamsVector &atlasParamsVector)
      -> std::pair<MVD, MaskList> {

    using TextureDepthFrame = typename MVD::value_type;
    using DepthFrame = typename TextureDepthFrame::second_type;

    // Initialization
    MVD frame;
    MaskList maskList;

    for (const auto &cam : viewParamsVector) {
      TextureFrame tex(cam.size.x(), cam.size.y());
      DepthFrame depth(cam.size.x(), cam.size.y());
      tex.fillNeutral();
      frame.push_back(TextureDepthFrame{std::move(tex), std::move(depth)});

      Mask mask(cam.size.x(), cam.size.y());
      std::fill(mask.getPlane(0).begin(), mask.getPlane(0).end(), 0);
      maskList.push_back(std::move(mask));
    }

    // Process patches
    auto atlas_pruned = atlas;

    for (auto iter = atlasParamsVector.rbegin(); iter != atlasParamsVector.rend(); ++iter) {
      const auto &patch = *iter;
      const auto occupancyTransform = OccupancyTransform{viewParamsVector[patch.viewId], patch};

      auto &currentAtlas = atlas_pruned[patch.atlasId];
      auto &currentView = frame[patch.viewId];

      auto &textureAtlasMap = currentAtlas.first;
      auto &depthAtlasMap = currentAtlas.second;

      auto &textureViewMap = currentView.first;
      auto &depthViewMap = currentView.second;

      auto &mask = maskList[patch.viewId];

      const auto sizeInAtlas = patch.patchSizeInAtlas();
      int wP = sizeInAtlas.x();
      int hP = sizeInAtlas.y();
      int xP = patch.posInAtlas.x();
      int yP = patch.posInAtlas.y();

      for (int dy = 0; dy < hP; dy++) {
        for (int dx = 0; dx < wP; dx++) {
          // get position
          Vec2i pAtlas = {xP + dx, yP + dy};
          Vec2i pView = atlasToView(pAtlas, patch);
          // Y
          if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
            textureViewMap.getPlane(0)(pView.y(), pView.x()) =
                textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
            textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
          }
          // UV
          if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
            for (int p = 1; p < 3; p++) {
              if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
                textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2) =
                    textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
                textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) = 0x200;
              }
            }
          }
          // Depth
          if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
            depthViewMap.getPlane(0)(pView.y(), pView.x()) =
                depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
            mask.getPlane(0)(pView.y(), pView.x()) = 255U;
          }
        }
      }
    }

    return {frame, maskList};
  }

  template <typename MVD, typename SourceProjectionType>
  void recoverPrunedSource(
      const MVD &atlasList, const IvSequenceParams &ivSequenceParams,
      const AtlasParamsList &atlasParamsList,
      const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList) {

    using TextureDepthFrame = typename MVD::value_type;
    using DepthFrame = typename TextureDepthFrame::second_type;

    // Recover pruned views
    auto prunedViewsAndMask =
        recoverPrunedViewAndMask(atlasList, ivSequenceParams.viewParamsList, atlasParamsList);

    const auto &prunedViews = prunedViewsAndMask.first;
    const auto &prunedMasks = prunedViewsAndMask.second;

    // Expand pruned views
    m_sourceColor.clear();
    m_sourceDepth.clear();

    for (std::size_t sourceId = 0; sourceId < prunedViews.size(); sourceId++) {

      const auto &viewParams = sourceHelperList[sourceId].getViewParams();

      m_sourceColor.emplace_back(expandTexture(prunedViews[sourceId].first));

      m_sourceDepth.emplace_back(DepthTransform<DepthFrame::getBitDepth()>{viewParams}.expandDepth(
          prunedViews[sourceId].second));

      std::transform(prunedMasks[sourceId].getPlane(0).begin(),
                     prunedMasks[sourceId].getPlane(0).end(), m_sourceDepth.back().begin(),
                     m_sourceDepth.back().begin(), [&](auto maskValue, float depthValue) {
                       return (0 < maskValue) ? depthValue : Common::NaN;
                     });
    }
  }
  template <typename SourceProjectionType, typename TargetProjectionType>
  void reprojectPrunedSource(
      const PatchIdMapList &patchIdMapList, const AtlasParamsList &atlasParamsList,
      const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList,
      const ProjectionHelper<TargetProjectionType> &targetHelper) {

    m_sourceUnprojection.resize(m_sourceDepth.size());
    m_sourceReprojection.resize(m_sourceDepth.size());
    m_sourceRayDirection.resize(m_sourceDepth.size());

    for (std::size_t sourceId = 0; sourceId < m_sourceDepth.size(); sourceId++) {

      m_sourceUnprojection[sourceId].resize(m_sourceDepth[sourceId].height(),
                                            m_sourceDepth[sourceId].width());
      std::fill(m_sourceUnprojection[sourceId].begin(), m_sourceUnprojection[sourceId].end(),
                Vec3f{Common::NaN, Common::NaN, Common::NaN});

      m_sourceReprojection[sourceId].resize(m_sourceDepth[sourceId].height(),
                                            m_sourceDepth[sourceId].width());
      std::fill(m_sourceReprojection[sourceId].begin(), m_sourceReprojection[sourceId].end(),
                std::make_pair(Vec2f{Common::NaN, Common::NaN}, Common::NaN));

      m_sourceRayDirection[sourceId].resize(m_sourceDepth[sourceId].height(),
                                            m_sourceDepth[sourceId].width());
      std::fill(m_sourceRayDirection[sourceId].begin(), m_sourceRayDirection[sourceId].end(),
                Vec3f{Common::NaN, Common::NaN, Common::NaN});
    }

    for (const auto &patchIdMap : patchIdMapList) {

      if (0 < patchIdMap.getPlane(0).width() && 0 < patchIdMap.getPlane(0).height()) {

        parallel_for(
            patchIdMap.getWidth(), patchIdMap.getHeight(), [&](std::size_t Y, std::size_t X) {
              auto patchId = patchIdMap.getPlane(0)(Y, X);

              if (patchId != unusedPatchId) {
                const auto &patch = atlasParamsList[patchId];
                auto viewId = patch.viewId;

                if (m_cameraVisibility[viewId]) {
                  auto posInView = atlasToView({static_cast<int>(X), static_cast<int>(Y)}, patch);

                  int x = posInView.x();
                  int y = posInView.y();
                  float z = m_sourceDepth[viewId](y, x);

                  if (sourceHelperList[viewId].isValidDepth(z)) {
                    auto P = sourceHelperList[viewId].doUnprojection(
                        Vec2f({static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F}), z);
                    auto p = targetHelper.doProjection(P);

                    if (isValidDepth(p.second) && targetHelper.isInsideViewport(p.first)) {
                      m_sourceUnprojection[viewId](y, x) = P;
                      m_sourceReprojection[viewId](y, x) = p;
                      m_sourceRayDirection[viewId](y, x) =
                          unit(P - targetHelper.getViewParams().position);
                    }
                  }
                }
              }
            });
      }
    }
  }
  template <typename TargetProjectionType>
  void warpPrunedSource(const AtlasParamsList &atlasParamsList,
                        const ProjectionHelper<TargetProjectionType> &targetHelper) {

    struct Splat {
      Vec2f center{};
      Vec2f firstAxis{};
      Vec2f secondAxis{};
      float pointSize{};
    };

    auto getSplatParameters = [&](unsigned viewId, int x, int y,
                                  const std::pair<Vec2f, float> &P) -> Splat {
      static const std::array<Vec2i, 8> offsetList = {
          Vec2i({1, 0}),  Vec2i({1, 1}),   Vec2i({0, 1}),  Vec2i({-1, 1}),
          Vec2i({-1, 0}), Vec2i({-1, -1}), Vec2i({0, -1}), Vec2i({1, -1})};

      int w_last = static_cast<int>(m_sourceReprojection[viewId].width()) - 1;
      int h_last = static_cast<int>(m_sourceReprojection[viewId].height()) - 1;

      std::array<std::pair<Vec2f, float>, 8> Q;
      std::array<float, 8> W{};
      float WT{0.F};

      // Center
      Vec2f C{0.F, 0.F};

      auto OP = m_sourceRayDirection[viewId](y, x);

      for (std::size_t i = 0U; i < offsetList.size(); i++) {

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
        return {Vec2f{0.F, 0.F}, Vec2f{0.F, 0.F}, Vec2f{0.F, 0.F}, 0.F};
      }

      // Axis (requires at least 5 good candidates)
      if (0.F < WT) {

        Mat2x2f M{0.F, 0.F, 0.F, 0.F};

        C /= WT;

        for (std::size_t i = 0U; i < offsetList.size(); i++) {

          if (isValidDepth(Q[i].second)) {
            Vec2f dp = (Q[i].first - C);
            M += W[i] * stack::Mat2x2<float>{dp.x() * dp.x(), dp.x() * dp.y(), dp.x() * dp.y(),
                                             dp.y() * dp.y()};
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
            Vec2f e1{1.F, 0.F};
            Vec2f e2{0.F, 1.F};

            if (l1 != l2) {

              Vec2f u1{M(0, 0) - l2, M(1, 0)};
              Vec2f u2{M(0, 1), M(1, 1) - l2};

              e1 = (0.F < dot(u1, u1)) ? unit(u1) : unit(u2);
              e2 = Vec2f{-e1.y(), e1.x()};
            }

            float r1 = std::sqrt(2.F * l1 / WT);
            float r2 = std::sqrt(2.F * l2 / WT);

            if (r1 < m_stretchFactor) {
              return {P.first, r1 * e1, r2 * e2, 2.F * r1};
            }
          }
        }
      }

      return {P.first, Vec2f{0.F, 0.F}, Vec2f{0.F, 0.F}, 1.F};
    };

    auto rasterizePoint = [&](unsigned viewId, const Splat &splat, const Vec3f &P,
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
      int x0 = std::max(0, ifloor(xLow));
      int x1 = std::min(w_last, iceil(xHigh));
      int y0 = std::max(0, ifloor(yLow));
      int y1 = std::min(h_last, iceil(yHigh));

      // Looping on all pixels within the bounding box
      for (int y = y0; y <= y1; y++) {
        float dy = ((static_cast<float>(y) + 0.5F) - splat.center.y());

        for (int x = x0; x <= x1; x++) {
          float dx = ((static_cast<float>(x) + 0.5F) - splat.center.x());
          float depthRef = m_viewportDepth[viewId](y, x);

          if (!isValidDepth(depthRef) || (depthValue < depthRef)) {
            if (0.F < R1) {
              Vec2f dp{dx, dy};

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

    for (std::size_t viewId = 0; viewId < m_sourceDepth.size(); viewId++) {
      if (m_cameraVisibility[viewId]) {

        m_viewportUnprojection[viewId].resize(targetHelper.getViewParams().size.y(),
                                              targetHelper.getViewParams().size.x());
        std::fill(m_viewportUnprojection[viewId].begin(), m_viewportUnprojection[viewId].end(),
                  Vec3f{Common::NaN, Common::NaN, Common::NaN});

        m_viewportDepth[viewId].resize(targetHelper.getViewParams().size.y(),
                                       targetHelper.getViewParams().size.x());
        std::fill(m_viewportDepth[viewId].begin(), m_viewportDepth[viewId].end(), Common::NaN);
      }
    }

    auto visibleSourceId = getEnabledIdList(m_cameraVisibility);

    parallel_for(visibleSourceId.size(), [&](std::size_t id) {
      auto viewId = static_cast<unsigned>(visibleSourceId[id]);

      for (const auto &patch : atlasParamsList) {
        if (patch.viewId == visibleSourceId[id]) {
          int x0 = patch.posInView.x();
          int x1 = x0 + patch.patchSizeInView.x();

          int y0 = patch.posInView.y();
          int y1 = y0 + patch.patchSizeInView.y();

          for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
              auto P = m_sourceReprojection[viewId](y, x);

              if (isValidDepth(P.second)) {

                auto splatParameters = getSplatParameters(viewId, x, y, P);

                if (0.F < splatParameters.pointSize) {
                  rasterizePoint(viewId, getSplatParameters(viewId, x, y, P),
                                 m_sourceUnprojection[viewId](y, x), P.second);
                }
              }
            }
          }
        }
      }
    });
  }
  template <typename SourceProjectionType, typename TargetProjectionType>
  void
  recoverPrunedWeight(const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList,
                      const ProjectionHelper<TargetProjectionType> &targetHelper) {

    // Retrieve pruning information
    auto hasPruningRelation =
        std::any_of(sourceHelperList.begin(), sourceHelperList.end(), [](const auto &helper) {
          const auto &viewParams = helper.getViewParams();
          return viewParams.pruningChildren && !viewParams.pruningChildren->empty();
        });

    // Weight recovery
    m_viewportWeight.resize(sourceHelperList.size());

    for (std::size_t viewId = 0; viewId < m_viewportWeight.size(); viewId++) {

      m_viewportWeight[viewId].resize(m_viewportDepth[viewId].height(),
                                      m_viewportDepth[viewId].width());
      std::fill(m_viewportWeight[viewId].begin(), m_viewportWeight[viewId].end(),
                hasPruningRelation ? 0.F : m_cameraWeight[viewId]);
    }

    if (hasPruningRelation) {
      // Pruning graph (from children to parent)
      Graph::BuiltIn::Sparse<float> pruningGraph(sourceHelperList.size());

      for (std::size_t nodeId = 0; nodeId < sourceHelperList.size(); nodeId++) {
        const auto &viewParams = sourceHelperList[nodeId].getViewParams();

        if (viewParams.pruningChildren) {
          for (auto childId : *viewParams.pruningChildren) {
            pruningGraph.connect(nodeId, static_cast<std::size_t>(childId), 1.F,
                                 LinkType::Directed);
          }
        }
      }

      pruningGraph = getReversedGraph(pruningGraph);

      // Pruning order
      auto pruningOrderId = getDescendingOrderId(pruningGraph);

      // Recovery
      parallel_for(
          targetHelper.getViewParams().size.x(), targetHelper.getViewParams().size.y(),
          [&](std::size_t y, std::size_t x) {
            for (auto prunedNodeId : pruningOrderId) {

              if (m_cameraVisibility[prunedNodeId]) {

                auto zPruned = m_viewportDepth[prunedNodeId](y, x);

                // Retrieve candidate
                std::queue<NodeId> nodeQueue;

                for (const auto &linkToParent : pruningGraph.getNeighbourhood(prunedNodeId)) {
                  nodeQueue.push(linkToParent.node());
                }

                int w_last = static_cast<int>(m_sourceDepth[prunedNodeId].width()) - 1;
                int h_last = static_cast<int>(m_sourceDepth[prunedNodeId].height()) - 1;

                std::vector<std::pair<NodeId, float>> candidateList;

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
                NodeId representativeNodeId = prunedNodeId;

                for (const auto &candidate : candidateList) {

                  auto p = prunedHelper.doProjection(m_viewportUnprojection[candidate.first](y, x));

                  if (isValidDepth(p.second) && prunedHelper.isInsideViewport(p.first)) {

                    static const std::array<Vec2i, 9> offsetList = {
                        Vec2i({-1, -1}), Vec2i({0, -1}), Vec2i({1, -1}),
                        Vec2i({-1, 0}),  Vec2i({0, 0}),  Vec2i({1, 0}),
                        Vec2i({-1, 1}),  Vec2i({0, 1}),  Vec2i({1, 1})};

                    auto X = ifloor(p.first.x());
                    auto Y = ifloor(p.first.y());

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
  template <typename SourceProjectionType, typename TargetProjectionType>
  void selectViewportDepth(bool trustDepth,
                           const ProjectionHelper<TargetProjectionType> &targetHelper) {

    m_viewportVisibility.resize(targetHelper.getViewParams().size.y(),
                                targetHelper.getViewParams().size.x());

    parallel_for(m_viewportVisibility.width(), m_viewportVisibility.height(),
                 [&](std::size_t y, std::size_t x) {
                   std::vector<Vec2f> stack;

                   for (std::size_t viewId = 0; viewId < m_viewportDepth.size(); viewId++) {

                     if (m_cameraVisibility[viewId]) {

                       float z = m_viewportDepth[viewId](y, x);

                       if (isValidDepth(z)) {
                         insertWeightedDepthInStack(stack, m_viewportWeight[viewId](y, x), z,
                                                    m_blendingFactor);
                       }
                     }
                   }

                   // Select best candidate
                   Vec2f bestCandidate = Vec2f({0.F, 0.F});

                   for (const auto &v : stack) {
                     if ((bestCandidate.y() < v.y()) &&
                         ((bestCandidate.y() * m_overloadFactor) < v.y())) {
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
    static const std::array<Vec2i, 9> offsetList = {Vec2i({-1, -1}), Vec2i({0, -1}), Vec2i({1, -1}),
                                                    Vec2i({-1, 0}),  Vec2i({0, 0}),  Vec2i({1, 0}),
                                                    Vec2i({-1, 1}),  Vec2i({0, 1}),  Vec2i({1, 1})};

    static Mat<float> flipVisibility;

    std::reference_wrapper<Mat<float>> firstWrapper =
        ((m_filteringPass % 2) != 0) ? flipVisibility : m_viewportVisibility;
    std::reference_wrapper<Mat<float>> secondWrapper =
        ((m_filteringPass % 2) != 0) ? m_viewportVisibility : flipVisibility;

    std::size_t w = m_viewportVisibility.width();
    std::size_t h = m_viewportVisibility.height();

    int w_last = static_cast<int>(w) - 1;
    int h_last = static_cast<int>(h) - 1;

    flipVisibility.resize(m_viewportVisibility.sizes());

    if ((m_filteringPass % 2) != 0) {
      std::copy(m_viewportVisibility.begin(), m_viewportVisibility.end(), flipVisibility.begin());
    }

    for (int iter = 0U; iter < m_filteringPass; iter++) {
      const Mat<float> &firstDepth = firstWrapper.get();
      Mat<float> &secondDepth = secondWrapper.get();

      parallel_for(w, h, [&](std::size_t y, std::size_t x) {
        std::array<float, 9> depthBuffer{};

        for (std::size_t i = 0; i < depthBuffer.size(); i++) {
          int xo = std::clamp(static_cast<int>(x) + offsetList[i].x(), 0, w_last);
          int yo = std::clamp(static_cast<int>(y) + offsetList[i].y(), 0, h_last);

          float z = firstDepth(yo, xo);

          depthBuffer[i] = isValidDepth(z) ? z : 0.F;
        }

        std::sort(depthBuffer.begin(), depthBuffer.end());

        for (std::size_t i = 4; i < 6; i++) {
          if (0.F < depthBuffer[i]) {
            secondDepth(y, x) = depthBuffer[i];
            return;
          }
        }

        secondDepth(y, x) = 0.F;
      });

      std::swap(firstWrapper, secondWrapper);
    }
  }
  template <typename SourceProjectionType, typename TargetProjectionType>
  void
  computeShadingMap(const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList,
                    const ProjectionHelper<TargetProjectionType> &targetHelper) {

    auto isProneToGhosting = [&](unsigned sourceId, const std::pair<Vec2f, float> &p,
                                 const Vec3f &OP) -> bool {
      static const std::array<Vec2i, 4> offsetList = {Vec2i({1, 0}), Vec2i({-1, 0}), Vec2i({0, 1}),
                                                      Vec2i({0, -1})};

      int w_last = static_cast<int>(m_sourceDepth[sourceId].width()) - 1;
      int h_last = static_cast<int>(m_sourceDepth[sourceId].height()) - 1;

      int x = ifloor(p.first.x());
      int y = ifloor(p.first.y());

      for (const auto &offset : offsetList) {
        int xo = std::clamp(x + offset.x(), 0, w_last);
        int yo = std::clamp(y + offset.y(), 0, h_last);

        float z = m_sourceDepth[sourceId](yo, xo);

        if (sourceHelperList[sourceId].isValidDepth(z)) {
          auto OQ = m_sourceRayDirection[sourceId](yo, xo);

          if (2.F * m_cameraDistortion[sourceId] < std::abs(std::acos(dot(OP, OQ)))) {
            return true;
          }
        } else {
          return true;
        }
      }

      return false;
    };

    static const std::array<Vec2i, 9> offsetList = {
        Vec2i({0, 0}), Vec2i({1, 0}),   Vec2i({1, 1}),  Vec2i({0, -1}), Vec2i({1, -1}),
        Vec2i({0, 1}), Vec2i({-1, -1}), Vec2i({-1, 0}), Vec2i({-1, 1})};

    static const std::array<float, 9> d2 = {0.F, 1.F, 2.F, 1.F, 2.F, 1.F, 2.F, 1.F, 2.F};

    int w_last = static_cast<int>(m_viewportVisibility.width()) - 1;
    int h_last = static_cast<int>(m_viewportVisibility.height()) - 1;

    m_viewportColor.resize(targetHelper.getViewParams().size.y(),
                           targetHelper.getViewParams().size.x());

    parallel_for(
        m_viewportVisibility.width(), m_viewportVisibility.height(),
        [&](std::size_t y, std::size_t x) {
          Vec3f oColor{};
          float oWeight = 0.F;

          std::vector<Vec2f> stack;

          for (std::size_t i = 0U; i < offsetList.size(); i++) {
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

            Vec2f pn1{static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F};

            auto P = targetHelper.doUnprojection(pn1, z);
            auto OP = unit(P - O);

            for (std::size_t sourceId = 0U; sourceId < sourceHelperList.size(); sourceId++) {

              if (m_cameraVisibility[sourceId]) {
                auto pn2 = sourceHelperList[sourceId].doProjection(P);

                if (isValidDepth(pn2.second) &&
                    sourceHelperList[sourceId].isInsideViewport(pn2.first)) {

                  if (isOnViewportContour ||
                      !isProneToGhosting(static_cast<unsigned>(sourceId), pn2, OP)) {

                    auto zRef = textureGather(m_sourceDepth[sourceId], pn2.first);
                    auto cRef = textureGather(m_sourceColor[sourceId], pn2.first);

                    Vec2f q = {pn2.first.x() - 0.5F, pn2.first.y() - 0.5F};
                    Vec2f f = {q.x() - std::floor(q.x()), q.y() - std::floor(q.y())};
                    Vec2f fb = {1.F - f.x(), 1.F - f.y()};

                    Vec4f wColor{fb.x() * f.y(), f.x() * f.y(), f.x() * fb.y(), fb.x() * fb.y()};

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

          m_viewportColor(y, x) =
              (eps < oWeight)
                  ? (oColor / oWeight)
                  : (isValidDepth(m_viewportVisibility(y, x)) ? Vec3f{-1.F, -1.F, -1.F} : Vec3f{});
        });
  }
};

////////////////////////////////////////////////////////////////////////////////
ViewWeightingSynthesizer::ViewWeightingSynthesizer(const TMIV::Common::Json & /*rootNode*/,
                                                   const TMIV::Common::Json &componentNode)
    : m_impl(new Impl(componentNode)) {}

ViewWeightingSynthesizer::ViewWeightingSynthesizer(float angularScaling, float minimalWeight,
                                                   float stretchFactor, float blendingFactor,
                                                   float overloadFactor, int filteringPass)
    : m_impl(new Impl(angularScaling, minimalWeight, stretchFactor, blendingFactor, overloadFactor,
                      filteringPass)) {}

ViewWeightingSynthesizer::~ViewWeightingSynthesizer() = default;

auto ViewWeightingSynthesizer::renderFrame(const Common::MVD10Frame &atlas,
                                           const Common::PatchIdMapList &maps,
                                           const Metadata::IvSequenceParams &ivSequenceParams,
                                           const Metadata::IvAccessUnitParams &ivAccessUnitParams,
                                           const Metadata::ViewParams &target) const
    -> Common::Texture444Depth16Frame {

  return std::visit(
      [&](const auto &sourceProjection, const auto &targetProjection) {
        using SourceProjectionType = std::decay_t<decltype(sourceProjection)>;
        using TargetProjectionType = std::decay_t<decltype(targetProjection)>;

        return m_impl->renderFrame<SourceProjectionType, TargetProjectionType>(
            atlas, maps, ivSequenceParams, ivAccessUnitParams, target);
      },
      ivSequenceParams.viewParamsList[0].projection, target.projection);
}
} // namespace TMIV::Renderer
