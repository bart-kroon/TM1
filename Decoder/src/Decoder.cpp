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

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;
using namespace TMIV::Renderer;

namespace TMIV::Decoder {
Decoder::Decoder(const Json &rootNode, const Json &componentNode)
    : m_geometryScaler{rootNode, componentNode}
    , m_entityBasedPatchMapFilter{rootNode, componentNode} {
  m_culler = Factory<ICuller>::getInstance().create("Culler", rootNode, componentNode);
  m_renderer = Factory<IRenderer>::getInstance().create("Renderer", rootNode, componentNode);
}

namespace {
void checkRestrictions(const AccessUnit &frame) {
  if (frame.vps.vps_miv_extension_flag()) {
    const auto &vme = frame.vps.vps_miv_extension();
    if (vme.vme_vui_params_present_flag()) {
      const auto &mvp = vme.miv_vui_parameters();
      if (!mvp.coordinate_axis_system_params().isOmafCas()) {
        throw runtime_error(
            "The VUI indicates that a coordinate axis system other than that of OMAF is used. The "
            "TMIV decoder/renderer is not yet able to convert between coordinate axis systems.");
      }
    }
  }
}
} // namespace

auto Decoder::decodeFrame(AccessUnit &frame, const ViewParams &viewportParams) const
    -> Texture444Depth16Frame {
  checkRestrictions(frame);
  m_geometryScaler.inplaceScale(frame);
  m_entityBasedPatchMapFilter.inplaceFilterBlockToPatchMaps(frame);
  m_culler->inplaceFilterBlockToPatchMaps(frame, viewportParams);
  return m_renderer->renderFrame(frame, viewportParams);
}
} // namespace TMIV::Decoder
