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

#include <TMIV/Downscaler/DownscalerStage.h>

#include <TMIV/Common/LoggingStrategy.h>
#include <TMIV/Downscaler/GeometryDownscaler.h>

namespace TMIV::Downscaler {
DownscalerStage::DownscalerStage(const Common::Json &componentNode)
    : m_geometryScaleEnabledFlag{componentNode.require("haveGeometryVideo").as<bool>() &&
                                 componentNode.require("haveTextureVideo").as<bool>() &&
                                 componentNode.require("geometryScaleEnabledFlag").as<bool>()} {}

void DownscalerStage::encode(CodableUnit unit) {
  Common::logDebug("Downscaler stage");

  if (m_geometryScaleEnabledFlag) {
    unit.encoderParams.vps.vps_miv_extension().vme_geometry_scale_enabled_flag(true);

    for (auto &atlas : unit.encoderParams.atlas) {
      atlas.asps.asps_miv_extension()
          .asme_geometry_scale_factor_x_minus1(1)
          .asme_geometry_scale_factor_y_minus1(1);
    }

    unit.deepFrameList = downscaleGeometry(unit.encoderParams.atlas, std::move(unit.deepFrameList));
  }

  source.encode(unit);
}
} // namespace TMIV::Downscaler
