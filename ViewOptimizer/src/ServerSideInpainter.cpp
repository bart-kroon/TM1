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

#include <TMIV/ViewOptimizer/ServerSideInpainter.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/Filter.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Decoder/AccessUnit.h>
#include <TMIV/Renderer/IInpainter.h>
#include <TMIV/Renderer/ISynthesizer.h>

#include <cstring>
#include <numeric>

using namespace std::string_literals;

namespace TMIV::ViewOptimizer {
using Common::Json;
using Common::MVD16Frame;
using Common::TextureDepth16Frame;
using Decoder::AccessUnit;
using Decoder::AtlasAccessUnit;
using MivBitstream::EncoderParams;
using MivBitstream::ViewParams;
using MivBitstream::ViewParamsList;
using Renderer::IInpainter;
using Renderer::ISynthesizer;

class ServerSideInpainter::Impl {
private:
  const int m_projectionPlaneWidth;
  const int m_projectionPlaneHeight;
  std::unique_ptr<IViewOptimizer> m_optimizer;
  std::unique_ptr<ISynthesizer> m_synthesizer;
  std::unique_ptr<IInpainter> m_inpainter;
  int m_blurKernel;
  int m_inpaintThreshold;
  EncoderParams m_sourceParams;
  EncoderParams m_transportParams;

public:
  Impl(const Json &rootNode, const Json &componentNode)
      : m_projectionPlaneWidth{componentNode.require("resolution"s)
                                   .as<Json::Array>()
                                   .at(0)
                                   .as<int>()}
      , m_projectionPlaneHeight{componentNode.require("resolution"s)
                                    .as<Json::Array>()
                                    .at(1)
                                    .as<int>()}
      , m_optimizer{Common::create<IViewOptimizer>("Sub"s, rootNode, componentNode)}
      , m_synthesizer{Common::create<ISynthesizer>("Synthesizer"s, rootNode, componentNode)}
      , m_inpainter{Common::create<IInpainter>("Inpainter"s, rootNode, componentNode)}
      , m_blurKernel{componentNode.require("blurKernel").as<int>()}
      , m_inpaintThreshold{componentNode.require("inpaintThreshold").as<int>()} {}

  auto optimizeParams(EncoderParams params) -> const EncoderParams & {
    m_sourceParams = std::move(params);
    m_transportParams = m_optimizer->optimizeParams(m_sourceParams);
    m_transportParams.viewParamsList.push_back(syntheticViewParams());
    return m_transportParams;
  }

  [[nodiscard]] auto optimizeFrame(MVD16Frame frame) const -> MVD16Frame {
    assert(!m_transportParams.viewParamsList.empty());
    const auto &viewportParams = m_transportParams.viewParamsList.back();
    auto synthFrame = m_synthesizer->renderFrame(synthesizerInputFrame(frame), viewportParams);
    filterDepthFrame(synthFrame.second);
    m_inpainter->inplaceInpaint(synthFrame, viewportParams);
    frame = m_optimizer->optimizeFrame(std::move(frame));
    frame.emplace_back(Common::yuv420p(synthFrame.first), synthFrame.second);
    return frame;
  }

private:
  [[nodiscard]] auto syntheticViewParams() noexcept -> ViewParams {
    auto vp = ViewParams{};

    vp.ci.ci_projection_plane_width_minus1(m_projectionPlaneWidth - 1)
        .ci_projection_plane_height_minus1(m_projectionPlaneHeight - 1)
        .ci_cam_type(MivBitstream::CiCamType::equirectangular)
        .ci_erp_phi_min(-Common::halfCycle)
        .ci_erp_phi_max(Common::halfCycle)
        .ci_erp_theta_min(-Common::quarterCycle)
        .ci_erp_theta_max(Common::quarterCycle);

    auto xyz = centerOfGravity(m_sourceParams.viewParamsList);
    vp.ce.ce_view_pos_x(xyz[0]).ce_view_pos_y(xyz[1]).ce_view_pos_z(xyz[2]);

    vp.dq.dq_norm_disp_low(INFINITY);
    vp.dq.dq_norm_disp_high(-INFINITY);

    for (const auto &vpIn : m_sourceParams.viewParamsList) {
      vp.dq.dq_norm_disp_low(std::min(vp.dq.dq_norm_disp_low(), vpIn.dq.dq_norm_disp_low()));
      vp.dq.dq_norm_disp_high(std::max(vp.dq.dq_norm_disp_high(), vpIn.dq.dq_norm_disp_high()));
    }

    vp.name = "s0"s;
    vp.isInpainted = true;

    return vp;
  }

  static auto centerOfGravity(const ViewParamsList &vpl) noexcept -> std::array<float, 3> {
    assert(!vpl.empty());

    auto sumX = 0.;
    auto sumY = 0.;
    auto sumZ = 0.;
    auto count = 0.;

    for (const auto &vp : vpl) {
      sumX += vp.ce.ce_view_pos_x();
      sumY += vp.ce.ce_view_pos_y();
      sumZ += vp.ce.ce_view_pos_z();
      ++count;
    }

    return {static_cast<float>(sumX / count), static_cast<float>(sumY / count),
            static_cast<float>(sumZ / count)};
  }

  [[nodiscard]] auto synthesizerInputFrame(const MVD16Frame &frame) const -> AccessUnit {
    auto inFrame = AccessUnit{};
    inFrame.viewParamsList = m_sourceParams.viewParamsList;

    std::transform(frame.cbegin(), frame.cend(), std::back_inserter(inFrame.atlas),
                   [viewIndex = uint16_t{}](const TextureDepth16Frame &frame) mutable {
                     return synthesizerInputAtlasAccessUnit(frame, viewIndex++);
                   });

    const auto dlqf = m_sourceParams.vps.vps_miv_extension().vme_depth_low_quality_flag();
    inFrame.vps.vps_extension_present_flag(true)
        .vps_miv_extension_present_flag(true)
        .vps_miv_extension()
        .vme_depth_low_quality_flag(dlqf);

    return inFrame;
  }

  static auto synthesizerInputAtlasAccessUnit(const TextureDepth16Frame &frame, uint16_t viewIndex)
      -> AtlasAccessUnit {
    auto aau = AtlasAccessUnit();

    const auto w = frame.texture.getWidth();
    const auto h = frame.texture.getHeight();

    aau.asps.asps_frame_width(static_cast<uint16_t>(w));
    aau.asps.asps_frame_height(static_cast<uint16_t>(h));

    aau.attrFrame = Common::yuv444p(frame.texture);

    aau.geoFrame.resize(w, h);
    std::transform(frame.depth.getPlane(0).cbegin(), frame.depth.getPlane(0).cend(),
                   aau.geoFrame.getPlane(0).begin(), [](uint16_t value) {
                     return static_cast<uint16_t>((value * 0x3FF + 0x7FFF) / 0xFFFF);
                   });

    aau.occFrame.resize(w, h);
    aau.occFrame.fillOne();

    auto &pp = aau.patchParamsList.emplace_back();
    pp.atlasPatchProjectionId(viewIndex);
    pp.atlasPatchOrientationIndex(MivBitstream::FlexiblePatchOrientation::FPO_NULL);
    pp.atlasPatch2dSizeX(frame.texture.getWidth());
    pp.atlasPatch2dSizeY(frame.texture.getHeight());

    const auto ppbs = std::gcd(128, std::gcd(w, h));
    aau.asps.asps_log2_patch_packing_block_size(Common::ceilLog2(ppbs));
    aau.blockToPatchMap.resize(w / ppbs, h / ppbs);
    std::fill(aau.blockToPatchMap.getPlane(0).begin(), aau.blockToPatchMap.getPlane(0).end(),
              uint16_t{});

    return aau;
  }

  void filterDepthFrame(Common::Depth16Frame &frame) const noexcept {
    const auto blurred = Common::boxBlur<int32_t>(frame.getPlane(0), m_blurKernel);

    std::transform(frame.getPlane(0).cbegin(), frame.getPlane(0).cend(), blurred.cbegin(),
                   frame.getPlane(0).begin(), [this](int orig, int blurred) {
                     if ((orig - blurred) > m_inpaintThreshold) {
                       return 0;
                     }
                     return orig;
                   });
  }
};

ServerSideInpainter::ServerSideInpainter(const Json &rootNode, const Json &componentNode)
    : m_impl{new Impl{rootNode, componentNode}} {}

ServerSideInpainter::~ServerSideInpainter() = default;

auto ServerSideInpainter::optimizeParams(EncoderParams params) -> const EncoderParams & {
  return m_impl->optimizeParams(std::move(params));
}

auto ServerSideInpainter::optimizeFrame(MVD16Frame frame) const -> MVD16Frame {
  return m_impl->optimizeFrame(std::move(frame));
}

} // namespace TMIV::ViewOptimizer
