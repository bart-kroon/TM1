/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/ViewOptimizer/ServerSideInpainter.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/Filter.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/IInpainter.h>
#include <TMIV/Renderer/ISynthesizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <cstring>
#include <numeric>

using namespace std::string_literals;

namespace TMIV::ViewOptimizer {
using Common::DeepFrame;
using Common::DeepFrameList;
using Common::Json;
using Common::QuatD;
using Common::Vec2f;
using Common::Vec2i;
using Common::Vec3d;
using Common::Vec3f;
using MivBitstream::AccessUnit;
using MivBitstream::AtlasAccessUnit;
using MivBitstream::CiCamType;
using MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using MivBitstream::ViewId;
using MivBitstream::ViewParams;
using MivBitstream::ViewParamsList;
using Renderer::IInpainter;
using Renderer::ISynthesizer;

template <typename V> struct CameraFrustum {
  std::vector<V> planeNear;
  std::vector<V> planeFar;
};

template <size_t cn>
auto meanOfVectors(const std::vector<Common::stack::Vector<float, cn>> &values) {
  PRECONDITION(!values.empty());

  Common::stack::Vector<float, cn> result{};
  for (auto v : values) {
    result += v;
  }
  return result / static_cast<float>(values.size());
}

template <typename ReturnType, typename RangeType, typename Func>
auto sampleRange(const RangeType &range, Func get) {
  std::vector<ReturnType> result;
  result.reserve(range.size());
  for (const auto &v : range) {
    result.push_back(get(v));
  }
  return result;
}

auto cartesianToPolarDegrees(const Vec3f &position) -> Vec2f {
  const auto phi = Common::rad2deg<float>(std::atan2(position.y(), position.x()));
  const auto theta =
      Common::rad2deg<float>(std::atan2(position.z(), std::hypot(position.x(), position.y())));
  return {phi, theta};
}

auto calcProjectionPlaneBoundary(const Vec2i &size, int32_t segmentsPerEdge) -> std::vector<Vec2f> {
  std::vector<Vec2f> boundary;

  const float dx = 1.F / static_cast<float>(segmentsPerEdge);
  const float dy = 1.F / static_cast<float>(segmentsPerEdge);

  // normalized boundary
  for (float x = 0.F; x < 1.F; x += dx) {
    boundary.push_back({x, 0.F});
  }
  for (float y = 0.F; y < 1.F; y += dy) {
    boundary.push_back({1.F, y});
  }
  for (float x = 1.F; x > 0.F; x -= dx) {
    boundary.push_back({x, 1.F});
  }
  for (float y = 1.F; y > 0.F; y -= dy) {
    boundary.push_back({0.F, y});
  }

  // un-normalize
  auto w = static_cast<float>(size[0]);
  auto h = static_cast<float>(size[1]);
  for (auto &xy : boundary) {
    xy = {xy[0] * (w - 1), xy[1] * (h - 1)};
  }

  return boundary;
}

auto rad2deg(const Vec3d &v) -> Vec3d {
  using TMIV::Common::rad2deg;
  return Vec3d{rad2deg(v[0]), rad2deg(v[1]), rad2deg(v[2])};
}

auto rad2deg(const Vec2f &v) -> Vec2f {
  using TMIV::Common::rad2deg;
  return Vec2f{rad2deg(v[0]), rad2deg(v[1])};
}

auto calcCameraFrustum(const MivBitstream::ViewParams &viewParams, int32_t segmentsPerEdge)
    -> CameraFrustum<Vec3f> {
  const auto ci = viewParams.ci;

  const Vec2i projectionPlaneSize = {ci.ci_projection_plane_width_minus1() + 1,
                                     ci.ci_projection_plane_height_minus1() + 1};

  const auto projectionPlaneBoundary =
      calcProjectionPlaneBoundary(projectionPlaneSize, segmentsPerEdge);

  // The choosen bit-depth is arbitrary. It is needed to find the maxium depth range
  const uint32_t depthBitDepth = 16U;
  const auto depthTransform = MivBitstream::DepthTransform{viewParams.dq, depthBitDepth};
  const auto depthNear = depthTransform.expandDepth(Common::maxLevel(depthBitDepth));
  const auto depthFar = 1.F / depthTransform.minNormDisp();

  CameraFrustum<Vec3f> frustumInCameraCoordinates;
  for (const auto &loc : projectionPlaneBoundary) {
    frustumInCameraCoordinates.planeNear.push_back(Renderer::unprojectVertex(loc, depthNear, ci));
    frustumInCameraCoordinates.planeFar.push_back(Renderer::unprojectVertex(loc, depthFar, ci));
  }

  return frustumInCameraCoordinates;
}

auto calcCameraFrustum(const MivBitstream::ViewParams &viewParams,
                       const MivBitstream::Pose &poseWorld, int32_t segmentsPerEdge)
    -> CameraFrustum<Vec3f> {
  auto frustumInCameraCoordinates = calcCameraFrustum(viewParams, segmentsPerEdge);

  // Transform coordinates to world
  CameraFrustum<Vec3f> frustumInWorldCoordinates;
  const auto Rt_transform = Renderer::AffineTransform(viewParams.pose, poseWorld);

  for (auto &xyz : frustumInCameraCoordinates.planeNear) {
    frustumInWorldCoordinates.planeNear.push_back(Rt_transform(xyz));
  }

  for (auto &xyz : frustumInCameraCoordinates.planeFar) {
    frustumInWorldCoordinates.planeFar.push_back(Rt_transform(xyz));
  }

  return frustumInWorldCoordinates;
}

class ServerSideInpainter::Impl {
private:
  const int32_t m_projectionPlaneWidth;
  const int32_t m_projectionPlaneHeight;
  std::unique_ptr<IViewOptimizer> m_optimizer;
  std::unique_ptr<ISynthesizer> m_synthesizer;
  std::unique_ptr<IInpainter> m_inpainter;
  int32_t m_blurKernel;
  int32_t m_inpaintThreshold;
  float m_fieldOfViewMargin;
  SourceParams m_sourceParams;
  ViewOptimizerParams m_transportParams;
  int32_t m_segmentsPerEdge;

public:
  Impl(const Json &rootNode, const Json &componentNode)
      : m_projectionPlaneWidth{componentNode.require("resolution"s)
                                   .as<Json::Array>()
                                   .at(0)
                                   .as<int32_t>()}
      , m_projectionPlaneHeight{componentNode.require("resolution"s)
                                    .as<Json::Array>()
                                    .at(1)
                                    .as<int32_t>()}
      , m_optimizer{Common::create<IViewOptimizer>("Sub"s, rootNode, componentNode)}
      , m_synthesizer{Common::create<ISynthesizer>("Synthesizer"s, rootNode, componentNode)}
      , m_inpainter{Common::create<IInpainter>("Inpainter"s, rootNode, componentNode)}
      , m_blurKernel{componentNode.require("blurRadius").as<int32_t>()}
      , m_inpaintThreshold{componentNode.require("inpaintThreshold").as<int32_t>()}
      , m_fieldOfViewMargin{componentNode.require("fieldOfViewMargin").as<float>()}
      , m_segmentsPerEdge{componentNode.require("segmentsPerEdge").as<int32_t>()} {}

  auto optimizeParams(const SourceParams &params) -> ViewOptimizerParams {
    m_sourceParams = params;
    m_transportParams = m_optimizer->optimizeParams(m_sourceParams);
    m_transportParams.viewParamsList.push_back(syntheticViewParams());
    m_transportParams.viewParamsList.constructViewIdIndex();
    return m_transportParams;
  }

  [[nodiscard]] auto optimizeFrame(DeepFrameList frame) const -> DeepFrameList {
    PRECONDITION(!m_transportParams.viewParamsList.empty());

    auto viewportParams = MivBitstream::CameraConfig{};
    viewportParams.viewParams = m_transportParams.viewParamsList.back();

    for (const auto &view : frame) {
      viewportParams.bitDepthTexture =
          std::max(viewportParams.bitDepthTexture, view.texture.getBitDepth());
      viewportParams.bitDepthGeometry =
          std::max(viewportParams.bitDepthGeometry, view.geometry.getBitDepth());
    }

    auto synthFrame = m_synthesizer->renderFrame(synthesizerInputFrame(frame), viewportParams);
    filterDepthFrame(synthFrame.geometry);
    m_inpainter->inplaceInpaint(synthFrame, viewportParams.viewParams);
    frame = m_optimizer->optimizeFrame(std::move(frame));
    frame.emplace_back().texture = Common::yuv420(synthFrame.texture);
    frame.back().geometry = synthFrame.geometry;
    return frame;
  }

private:
  [[nodiscard]] auto syntheticViewParams() -> ViewParams {
    auto vp = ViewParams{};

    vp.pose = computeCameraRigPose();

    auto [minPhi, minTheta, maxPhi, maxTheta] = computeCameraRigFieldOfView(vp.pose);

    vp.ci.ci_projection_plane_width_minus1(m_projectionPlaneWidth - 1)
        .ci_projection_plane_height_minus1(m_projectionPlaneHeight - 1)
        .ci_cam_type(MivBitstream::CiCamType::equirectangular)
        .ci_erp_phi_min(minPhi)
        .ci_erp_phi_max(maxPhi)
        .ci_erp_theta_min(minTheta)
        .ci_erp_theta_max(maxTheta);

    vp.dq.dq_norm_disp_low(INFINITY);
    vp.dq.dq_norm_disp_high(-INFINITY);

    for (const auto &vpIn : m_sourceParams.viewParamsList) {
      vp.dq.dq_norm_disp_low(std::min(vp.dq.dq_norm_disp_low(), vpIn.dq.dq_norm_disp_low()));
      vp.dq.dq_norm_disp_high(std::max(vp.dq.dq_norm_disp_high(), vpIn.dq.dq_norm_disp_high()));
    }

    vp.name = "s0"s;
    vp.viewInpaintFlag = true;
    vp.isBasicView = false;

    // Assign the first available view ID
    for (uint16_t v = 0;; ++v) {
      if (std::none_of(m_transportParams.viewParamsList.cbegin(),
                       m_transportParams.viewParamsList.cend(),
                       [v](const auto &viewParams) { return viewParams.viewId == ViewId{v}; })) {
        vp.viewId = ViewId{v};
        break;
      }
    }

    return vp;
  }

  [[nodiscard]] auto computeCameraRigPose() const -> MivBitstream::Pose {
    auto cameraPositions =
        sampleRange<Vec3f>(m_sourceParams.viewParamsList, [](auto v) { return v.pose.position; });
    auto cameraOrientations = sampleRange<QuatD>(m_sourceParams.viewParamsList,
                                                 [](auto v) { return v.pose.orientation; });

    auto meanPosition = meanOfVectors(cameraPositions);
    auto meanOrientation = directAveragingOfOrientations(cameraOrientations);

    return MivBitstream::Pose{meanPosition, meanOrientation};
  }

  [[nodiscard]] auto computeCameraRigFieldOfView(const MivBitstream::Pose &poseRig)
      -> std::array<float, 4> {
    float minPhi = -180.F;
    float maxPhi = 180.F;
    float minTheta = -90.F;
    float maxTheta = 90.F;

    bool allCamerasArePerspective = true;
    for (const auto &vp : m_sourceParams.viewParamsList) {
      if (vp.ci.ci_cam_type() != CiCamType::perspective) {
        allCamerasArePerspective = false;
        break;
      }
    }

    if (allCamerasArePerspective) {
      std::vector<CameraFrustum<Vec3f>> frustums;
      for (auto &viewParams : m_sourceParams.viewParamsList) {
        frustums.push_back(calcCameraFrustum(viewParams, poseRig, m_segmentsPerEdge));
      }

      std::vector<Vec3f> vecXyz;
      for (const auto &f : frustums) {
        vecXyz.insert(vecXyz.end(), f.planeNear.begin(), f.planeNear.end());
        vecXyz.insert(vecXyz.end(), f.planeFar.begin(), f.planeFar.end());
      }

      std::vector<Vec2f> vecPolarDegrees;
      vecPolarDegrees.reserve(vecXyz.size());
      for (const auto &xyz : vecXyz) {
        vecPolarDegrees.push_back(cartesianToPolarDegrees(xyz));
      }

      // compute bounding rect
      minPhi = vecPolarDegrees[0][0];
      maxPhi = vecPolarDegrees[0][0];
      minTheta = vecPolarDegrees[0][1];
      maxTheta = vecPolarDegrees[0][1];
      for (const auto &pt : vecPolarDegrees) {
        minPhi = std::min(minPhi, pt[0]);
        maxPhi = std::max(maxPhi, pt[0]);
        minTheta = std::min(minTheta, pt[1]);
        maxTheta = std::max(maxTheta, pt[1]);
      }

      // apply margin
      minPhi = std::max(minPhi - m_fieldOfViewMargin * std::abs(minPhi), -180.F);
      maxPhi = std::min(maxPhi + m_fieldOfViewMargin * std::abs(maxPhi), 180.F);
      minTheta = std::max(minTheta - m_fieldOfViewMargin * std::abs(minTheta), -90.F);
      maxTheta = std::min(maxTheta + m_fieldOfViewMargin * std::abs(maxTheta), 90.F);
    }

    return {minPhi, minTheta, maxPhi, maxTheta};
  }

  [[nodiscard]] auto synthesizerInputFrame(const DeepFrameList &frame) const -> AccessUnit {
    auto inFrame = AccessUnit{};
    inFrame.viewParamsList = m_sourceParams.viewParamsList;

    std::transform(
        frame.cbegin(), frame.cend(), std::back_inserter(inFrame.atlas),
        [&vpl = inFrame.viewParamsList, viewIdx = uint16_t{}](const DeepFrame &frame) mutable {
          return synthesizerInputAtlasAccessUnit(frame, vpl[viewIdx++].viewId);
        });

    // Transfer depth low quality flag
    inFrame.casps = CommonAtlasSequenceParameterSetRBSP{};
    inFrame.casps->casps_miv_extension().casme_depth_low_quality_flag(
        m_sourceParams.depthLowQualityFlag);

    return inFrame;
  }

  static auto synthesizerInputAtlasAccessUnit(const DeepFrame &frame, ViewId viewId)
      -> AtlasAccessUnit {
    auto aau = AtlasAccessUnit();

    const auto w = frame.texture.getWidth();
    const auto h = frame.texture.getHeight();

    aau.asps.asps_frame_width(static_cast<uint16_t>(w));
    aau.asps.asps_frame_height(static_cast<uint16_t>(h));

    aau.texFrame = Common::yuv444(frame.texture);

    aau.geoFrame.createY({w, h}, 10);
    const auto maxInValue = Common::maxLevel(frame.geometry.getBitDepth());
    const auto maxOutValue = Common::maxLevel(aau.geoFrame.getBitDepth());
    std::transform(frame.geometry.getPlane(0).cbegin(), frame.geometry.getPlane(0).cend(),
                   aau.geoFrame.getPlane(0).begin(), [=](uint16_t value) {
                     return static_cast<uint16_t>((value * maxOutValue + maxInValue / 2) /
                                                  maxInValue);
                   });

    aau.occFrame.createY({w, h});
    aau.occFrame.fillOne();

    auto &pp = aau.patchParamsList.emplace_back();
    pp.atlasPatchProjectionId(viewId);
    pp.atlasPatchOrientationIndex(MivBitstream::FlexiblePatchOrientation::FPO_NULL);
    pp.atlasPatch2dSizeX(frame.texture.getWidth());
    pp.atlasPatch2dSizeY(frame.texture.getHeight());
    pp.atlasPatch3dRangeD(Common::maxLevel(aau.geoFrame.getBitDepth()));

    const auto ppbs = std::gcd(128, std::gcd(w, h));
    aau.asps.asps_log2_patch_packing_block_size(Common::ceilLog2(ppbs));
    aau.blockToPatchMap.createY({w / ppbs, h / ppbs});
    std::fill(aau.blockToPatchMap.getPlane(0).begin(), aau.blockToPatchMap.getPlane(0).end(),
              uint16_t{});

    return aau;
  }

  void filterDepthFrame(Common::Frame<> &frame) const noexcept {
    const auto blurred = Common::boxBlur<uint32_t>(frame.getPlane(0), m_blurKernel);

    std::transform(frame.getPlane(0).cbegin(), frame.getPlane(0).cend(), blurred.cbegin(),
                   frame.getPlane(0).begin(), [this](auto orig, auto blurred) {
                     if ((orig - blurred) > m_inpaintThreshold) {
                       return uint16_t{};
                     }
                     return orig;
                   });
  }
};

ServerSideInpainter::ServerSideInpainter(const Json &rootNode, const Json &componentNode)
    : m_impl{new Impl{rootNode, componentNode}} {}

ServerSideInpainter::~ServerSideInpainter() = default;

auto ServerSideInpainter::optimizeParams(const SourceParams &params) -> ViewOptimizerParams {
  return m_impl->optimizeParams(params);
}

auto ServerSideInpainter::optimizeFrame(DeepFrameList frame) const -> DeepFrameList {
  return m_impl->optimizeFrame(std::move(frame));
}

} // namespace TMIV::ViewOptimizer
