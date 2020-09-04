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

#include <TMIV/Decoder/AccessUnit.h>

using namespace TMIV::Common;

namespace TMIV::Decoder {
auto AtlasAccessUnit::frameSize() const noexcept -> Common::Vec2i {
  return Common::Vec2i{asps.asps_frame_width(), asps.asps_frame_height()};
}

auto AtlasAccessUnit::decGeoFrameSize(const MivBitstream::V3cParameterSet &vps) const noexcept
    -> Common::Vec2i {
  if (vps.vps_miv_extension_flag()) {
    const auto &vme = vps.vps_miv_extension();
    if (vme.vme_geometry_scale_enabled_flag()) {
      const auto &asme = asps.asps_miv_extension();
      return Common::Vec2i{
          asps.asps_frame_width() / (asme.asme_geometry_scale_factor_x_minus1() + 1),
          asps.asps_frame_height() / (asme.asme_geometry_scale_factor_y_minus1() + 1)};
    }
  }
  return frameSize();
}

auto AtlasAccessUnit::decOccFrameSize(const MivBitstream::V3cParameterSet &vps) const noexcept
    -> Vec2i {
  if (vps.vps_miv_extension_flag()) {
    const auto &vme = vps.vps_miv_extension();
    if (!vme.vme_embedded_occupancy_flag() && vme.vme_occupancy_scale_enabled_flag()) {
      const auto &asme = asps.asps_miv_extension();
      const int codedUnpaddedOccupancyWidth =
          asps.asps_frame_width() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
      const int codedUnpadedOccupancyHeight =
          asps.asps_frame_height() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
      const int codedOccupancyWidth = codedUnpaddedOccupancyWidth + codedUnpaddedOccupancyWidth % 2;
      const int codedOccupancyHeight =
          codedUnpadedOccupancyHeight + codedUnpadedOccupancyHeight % 2;
      return Vec2i{codedOccupancyWidth, codedOccupancyHeight};
    }
  }
  return frameSize();
}

auto AtlasAccessUnit::patchId(unsigned row, unsigned column) const -> std::uint16_t {
  const auto k = asps.asps_log2_patch_packing_block_size();
  return blockToPatchMap.getPlane(0)(row >> k, column >> k);
}
} // namespace TMIV::Decoder
