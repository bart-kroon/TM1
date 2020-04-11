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

#ifndef _TMIV_MIVBITSTREAM_IVACCESSUNITPARAMS_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
inline AtlasAccessUnitParams::AtlasAccessUnitParams() {
  asps.asps_extension_present_flag(true)
      .asps_miv_extension_present_flag(true)
      .asps_num_ref_atlas_frame_lists_in_asps(1);
  afps.afps_fixed_camera_model_flag(false);
  atgh.atgh_type(AtghType::I_TILE_GRP);
}

inline auto operator<<(std::ostream &stream, const AtlasAccessUnitParams &x) -> std::ostream & {
  stream << x.asps << x.afps << x.atgh;
  return stream;
}

inline auto AtlasAccessUnitParams::operator==(const AtlasAccessUnitParams &other) const -> bool {
  return asps == other.asps && afps == other.afps && atgh == other.atgh;
}

inline auto operator<<(std::ostream &stream, const IvAccessUnitParams &x) -> std::ostream & {
  for (const auto &atlas : x.atlas) {
    stream << atlas.asps << atlas.afps << atlas.atgh;
  }
  stream << "Total number of patches: " << x.patchParamsList.size() << '\n';
  return stream;
}

inline auto IvAccessUnitParams::operator==(const IvAccessUnitParams &other) const -> bool {
  return atlas == other.atlas && patchParamsList == other.patchParamsList;
}
} // namespace TMIV::MivBitstream
