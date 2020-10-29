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

#include <TMIV/Decoder/Decoder.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Decoder/GeometryScaler.h>
#include <TMIV/Decoder/OccupancyReconstructor.h>

namespace TMIV::Decoder {
Decoder::Decoder(const Common::Json &rootNode, const Common::Json &componentNode)
    : m_geometryScaler{rootNode, componentNode}
    , m_entityBasedPatchMapFilter{rootNode, componentNode} {
  m_culler =
      Common::Factory<Renderer::ICuller>::getInstance().create("Culler", rootNode, componentNode);
  m_renderer = Common::Factory<Renderer::IRenderer>::getInstance().create("Renderer", rootNode,
                                                                          componentNode);
}

namespace {
void checkRestrictions(const AccessUnit &frame) {
  if (frame.vui) {
    if (frame.vui->vui_coordinate_system_parameters_present_flag()) {
      const auto &csp = frame.vui->coordinate_system_parameters();
      if (!csp.isOmafCas()) {
        throw std::runtime_error(
            "The VUI indicates that a coordinate axis system other than that of OMAF is used. "
            "The TMIV decoder/renderer is not yet able to convert between coordinate axis "
            "systems.");
      }
    }
  }
}

void addAttributeOffset(AccessUnit &frame) {
  for (std::uint8_t k = 0; k <= frame.vps.vps_atlas_count_minus1(); ++k) {
    const auto atlasId = frame.vps.vps_atlas_id(k);
    auto &atlas = frame.atlas[k];

    if (!atlas.asps.asps_miv_extension().asme_patch_attribute_offset_flag()) {
      return;
    }

    const auto &btpm = atlas.blockToPatchMap.getPlane(0);
    auto &YUV = atlas.attrFrame.getPlanes();

    const auto H = atlas.attrFrame.getHeight();
    const auto W = atlas.attrFrame.getWidth();

    const auto &ai = frame.vps.attribute_information(atlasId);
    const auto inputBitCount = ai.ai_attribute_2d_bit_depth_minus1(0) + 1;
    const auto inputMaxVal = (1 << inputBitCount) - 1;
    const auto inputMedVal = 1 << (inputBitCount - 1);

    const int scaledBitCount =
        atlas.asps.asps_miv_extension().asme_patch_attribute_offset_bit_count_minus1() + 1;
    const int bitShift = inputBitCount - scaledBitCount;

    const int btpmScale = int(YUV[0].width() / btpm.width());

    for (int h = 0; h < H; h++) {
      const int hs = h / btpmScale;
      for (int w = 0; w < W; w++) {
        const int ws = w / btpmScale;
        if (btpm(hs, ws) != 65535) {

          for (int c = 0; c < 3; c++) {
            const auto pduAttributeOffset = static_cast<int16_t>(
                (atlas.patchParamsList[btpm(hs, ws)].atlasPatchAttributeOffset()->operator[](c)
                 << bitShift) -
                inputMedVal);

            if (YUV[c](h, w) + pduAttributeOffset > inputMaxVal) {
              YUV[c](h, w) = inputMaxVal;
            } else if (YUV[c](h, w) + pduAttributeOffset < 0) {
              YUV[c](h, w) = 0;
            } else {
              YUV[c](h, w) += (pduAttributeOffset);
            }
          }
        }
      }
    }
  }
}

} // namespace

void Decoder::recoverFrame(AccessUnit &frame) {
  checkRestrictions(frame);
  addAttributeOffset(frame);
  m_geometryScaler.inplaceScale(frame);
  OccupancyReconstructor::reconstruct(frame);
  m_entityBasedPatchMapFilter.inplaceFilterBlockToPatchMaps(frame);
}

auto Decoder::renderFrame(AccessUnit &frame, const MivBitstream::ViewParams &viewportParams) const
    -> Common::Texture444Depth16Frame {
  m_culler->inplaceFilterBlockToPatchMaps(frame, viewportParams);
  return m_renderer->renderFrame(frame, viewportParams);
}
} // namespace TMIV::Decoder
