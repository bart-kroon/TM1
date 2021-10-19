/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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
#include <TMIV/Renderer/IInpainter.h>
#include <TMIV/Renderer/ISynthesizer.h>

#include <cstring>
#include <numeric>

using namespace std::string_literals;

namespace TMIV::ViewOptimizer {
using Common::Json;
using Common::MVD16Frame;
using Common::TextureDepth16Frame;
using MivBitstream::AccessUnit;
using MivBitstream::AtlasAccessUnit;
using MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using MivBitstream::ViewId;
using MivBitstream::ViewParams;
using MivBitstream::ViewParamsList;
using Renderer::IInpainter;
using Renderer::ISynthesizer;
using TMIV::Common::Vec2f;
using TMIV::MivBitstream::CiCamType;

class FieldOfView {
private:
  float m_minPhi;
  float m_maxPhi;
  float m_minTheta;
  float m_maxTheta;

public:
  [[nodiscard]] constexpr auto minPhi() const noexcept { return m_minPhi; }
  [[nodiscard]] constexpr auto maxPhi() const noexcept { return m_maxPhi; }
  [[nodiscard]] constexpr auto minTheta() const noexcept { return m_minTheta; }
  [[nodiscard]] constexpr auto maxTheta() const noexcept { return m_maxTheta; }

  FieldOfView(float minPhi_, float maxPhi_, float minTheta_, float maxTheta_)
      : m_minPhi{std::max(-180.F, minPhi_)}
      , m_maxPhi{std::min(180.F, maxPhi_)}
      , m_minTheta{std::max(-90.F, minTheta_)}
      , m_maxTheta{std::min(90.F, maxTheta_)} {}

  // Null element of the combine operation: combine(zero, something) == something
  [[nodiscard]] static auto zero() noexcept -> FieldOfView { return {180.F, -180.F, 90.F, -90.F}; }

  [[nodiscard]] static auto full() noexcept -> FieldOfView { return {-180.F, 180.F, -90.F, 90.F}; }

  [[nodiscard]] static auto computeFrom(const ViewParams &vp) noexcept -> FieldOfView {
    const auto focal = Vec2f{vp.ci.ci_perspective_focal_hor(), vp.ci.ci_perspective_focal_ver()};

    // half angle of the field of view of the perspective projection
    const float halfFovX =
        Common::rad2deg(std::atan(0.5F * vp.ci.projectionPlaneSizeF().x() / std::abs(focal.x())));
    const float halfFovY =
        Common::rad2deg(std::atan(0.5F * vp.ci.projectionPlaneSizeF().y() / std::abs(focal.y())));

    const auto euler = Common::Vec3f{Common::floatCast, quat2eulerDeg(vp.pose.orientation)};
    const auto yaw = euler[0];
    const auto pitch = euler[1];
    const auto phi = yaw;
    const auto theta = -pitch;

    return {phi - halfFovX, phi + halfFovX, theta - halfFovY, theta + halfFovY};
  }

  [[nodiscard]] auto applyMargin(float margin) const noexcept -> FieldOfView {
    return {m_minPhi - margin * std::abs(m_minPhi), m_maxPhi + margin * std::abs(m_maxPhi),
            m_minTheta - margin * std::abs(m_minTheta), m_maxTheta + margin * std::abs(m_maxTheta)};
  }

  [[nodiscard]] auto combine(const FieldOfView &other) const noexcept -> FieldOfView {
    return {std::min(m_minPhi, other.m_minPhi), std::max(m_maxPhi, other.m_maxPhi),
            std::min(m_minTheta, other.m_minTheta), std::max(m_maxTheta, other.m_maxTheta)};
  }
};

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
      , m_fieldOfViewMargin{componentNode.require("fieldOfViewMargin").as<float>()} {}

  auto optimizeParams(const SourceParams &params) -> ViewOptimizerParams {
    m_sourceParams = params;
    m_transportParams = m_optimizer->optimizeParams(m_sourceParams);
    m_transportParams.viewParamsList.push_back(syntheticViewParams());
    m_transportParams.viewParamsList.constructViewIdIndex();
    return m_transportParams;
  }

  [[nodiscard]] auto optimizeFrame(MVD16Frame frame) const -> MVD16Frame {
    PRECONDITION(!m_transportParams.viewParamsList.empty());

    auto viewportParams = MivBitstream::CameraConfig{};
    viewportParams.viewParams = m_transportParams.viewParamsList.back();
    viewportParams.bitDepthColor = 10; // TODO(#397): Magic bit depth
    viewportParams.bitDepthDepth = 16; // TODO(#397): Magic bit depth

    auto synthFrame = m_synthesizer->renderFrame(synthesizerInputFrame(frame), viewportParams);
    filterDepthFrame(synthFrame.second);
    m_inpainter->inplaceInpaint(synthFrame, viewportParams.viewParams);
    frame = m_optimizer->optimizeFrame(std::move(frame));
    frame.emplace_back().texture = Common::yuv420(synthFrame.first);
    frame.back().depth = synthFrame.second;
    return frame;
  }

private:
  [[nodiscard]] auto syntheticViewParams() noexcept -> ViewParams {
    auto vp = ViewParams{};

    const auto fov = fieldOfViewRig(m_sourceParams.viewParamsList, m_fieldOfViewMargin);

    vp.ci.ci_projection_plane_width_minus1(m_projectionPlaneWidth - 1)
        .ci_projection_plane_height_minus1(m_projectionPlaneHeight - 1)
        .ci_cam_type(MivBitstream::CiCamType::equirectangular)
        .ci_erp_phi_min(fov.minPhi())
        .ci_erp_phi_max(fov.maxPhi())
        .ci_erp_theta_min(fov.minTheta())
        .ci_erp_theta_max(fov.maxTheta());

    const auto center = centerOfGravity(m_sourceParams.viewParamsList);
    std::copy(center.cbegin(), center.cend(), vp.pose.position.begin());

    vp.dq.dq_norm_disp_low(INFINITY);
    vp.dq.dq_norm_disp_high(-INFINITY);

    for (const auto &vpIn : m_sourceParams.viewParamsList) {
      vp.dq.dq_norm_disp_low(std::min(vp.dq.dq_norm_disp_low(), vpIn.dq.dq_norm_disp_low()));
      vp.dq.dq_norm_disp_high(std::max(vp.dq.dq_norm_disp_high(), vpIn.dq.dq_norm_disp_high()));
    }

    vp.name = "s0"s;
    vp.isInpainted = true;
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

  static auto centerOfGravity(const ViewParamsList &vpl) noexcept -> std::array<float, 3> {
    PRECONDITION(!vpl.empty());

    auto sumX = 0.;
    auto sumY = 0.;
    auto sumZ = 0.;
    auto count = 0.;

    for (const auto &vp : vpl) {
      sumX += vp.pose.position.x();
      sumY += vp.pose.position.y();
      sumZ += vp.pose.position.z();
      ++count;
    }

    return {static_cast<float>(sumX / count), static_cast<float>(sumY / count),
            static_cast<float>(sumZ / count)};
  }

  static auto fieldOfViewRig(const ViewParamsList &vpl, float margin) noexcept -> FieldOfView {
    PRECONDITION(!vpl.empty());

    auto erpFov = FieldOfView::zero();

    for (const auto &vp : vpl) {
      if (vp.ci.ci_cam_type() != CiCamType::perspective) {
        return FieldOfView::full();
      }

      const auto fov = FieldOfView::computeFrom(vp);
      erpFov = fov.applyMargin(margin).combine(erpFov);
    }

    return erpFov;
  }

  [[nodiscard]] auto synthesizerInputFrame(const MVD16Frame &frame) const -> AccessUnit {
    auto inFrame = AccessUnit{};
    inFrame.viewParamsList = m_sourceParams.viewParamsList;

    std::transform(frame.cbegin(), frame.cend(), std::back_inserter(inFrame.atlas),
                   [&vpl = inFrame.viewParamsList,
                    viewIdx = uint16_t{}](const TextureDepth16Frame &frame) mutable {
                     return synthesizerInputAtlasAccessUnit(frame, vpl[viewIdx++].viewId);
                   });

    // Transfer depth low quality flag
    inFrame.casps = CommonAtlasSequenceParameterSetRBSP{};
    inFrame.casps->casps_miv_extension().casme_depth_low_quality_flag(
        m_sourceParams.depthLowQualityFlag);

    return inFrame;
  }

  static auto synthesizerInputAtlasAccessUnit(const TextureDepth16Frame &frame, ViewId viewId)
      -> AtlasAccessUnit {
    auto aau = AtlasAccessUnit();

    const auto w = frame.texture.getWidth();
    const auto h = frame.texture.getHeight();

    aau.asps.asps_frame_width(static_cast<uint16_t>(w));
    aau.asps.asps_frame_height(static_cast<uint16_t>(h));

    aau.attrFrame = Common::yuv444(frame.texture);

    // TODO(#397): Improve performance by increasing bit depth to Common::sampleBitDepth
    aau.geoFrame.createY({w, h}, 10);
    const auto maxInValue = Common::maxLevel(frame.depth.getBitDepth());
    const auto maxOutValue = Common::maxLevel(aau.geoFrame.getBitDepth());
    std::transform(frame.depth.getPlane(0).cbegin(), frame.depth.getPlane(0).cend(),
                   aau.geoFrame.getPlane(0).begin(), [=](uint16_t value) {
                     return static_cast<uint16_t>((value * maxOutValue + maxInValue / 2) /
                                                  maxInValue);
                   });

    // TODO(#397): The bit depth can be set to 1
    aau.occFrame.createY({w, h}, 10);
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

  void filterDepthFrame(Common::Depth16Frame &frame) const noexcept {
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

auto ServerSideInpainter::optimizeFrame(MVD16Frame frame) const -> MVD16Frame {
  return m_impl->optimizeFrame(std::move(frame));
}

} // namespace TMIV::ViewOptimizer
