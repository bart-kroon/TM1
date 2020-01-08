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

#include <TMIV/VpccBitstream/AtlasFrameParameterSetRBSP.h>

#include "verify.h"
#include <TMIV/Common/Bitstream.h>

#include <ostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::VpccBitstream {
auto operator<<(ostream &stream, const AtlasFrameTileInformation & /* unused */) -> ostream & {
  return stream << "afti_single_tile_in_atlas_frame_flag=true";
}

auto AtlasFrameTileInformation::decodeFrom(InputBitstream &bitstream) -> AtlasFrameTileInformation {
  const auto afti_single_tile_in_atlas_frame_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(afti_single_tile_in_atlas_frame_flag);
  return {};
}

void AtlasFrameTileInformation::encodeTo(OutputBitstream &bitstream) {
  constexpr auto afti_single_tile_in_atlas_frame_flag = true;
  bitstream.putFlag(afti_single_tile_in_atlas_frame_flag);
}

auto operator<<(ostream &stream, const AtlasFrameParameterSetRBSP &x) -> ostream & {
  return stream << "afps_atlas_frame_parameter_set_id="
                << int(x.afps_atlas_frame_parameter_set_id())
                << "\nafps_atlas_sequence_parameter_set_id="
                << int(x.afps_atlas_sequence_parameter_set_id()) << "\n"
                << x.atlas_frame_tile_information()
                << "\nafps_num_ref_idx_default_active=" << int(x.afps_num_ref_idx_default_active())
                << "\nafps_additional_lt_afoc_lsb_len=" << int(x.afps_additional_lt_afoc_lsb_len())
                << "\nafps_2d_pos_x_bit_count=" << int(x.afps_2d_pos_x_bit_count())
                << "\nafps_2d_pos_y_bit_count=" << int(x.afps_2d_pos_y_bit_count())
                << "\nafps_3d_pos_x_bit_count=" << int(x.afps_3d_pos_x_bit_count())
                << "\nafps_3d_pos_y_bit_count=" << int(x.afps_3d_pos_y_bit_count())
                << "\nafps_lod_bit_count=" << int(x.afps_lod_bit_count())
                << "\nafps_override_eom_for_depth_flag=" << boolalpha
                << x.afps_override_eom_for_depth_flag()
                << "\nafps_raw_3d_pos_bit_count_explicit_mode_flag=" << boolalpha
                << x.afps_raw_3d_pos_bit_count_explicit_mode_flag()
                << "\nafps_extension_present_flag=" << boolalpha << x.afps_extension_present_flag()
                << '\n';
}

auto AtlasFrameParameterSetRBSP::operator==(const AtlasFrameParameterSetRBSP &other) const noexcept
    -> bool {
  return afps_atlas_frame_parameter_set_id() == other.afps_atlas_frame_parameter_set_id() &&
         afps_atlas_sequence_parameter_set_id() == other.afps_atlas_sequence_parameter_set_id() &&
         atlas_frame_tile_information() == other.atlas_frame_tile_information() &&
         afps_num_ref_idx_default_active() == other.afps_num_ref_idx_default_active() &&
         afps_additional_lt_afoc_lsb_len() == other.afps_additional_lt_afoc_lsb_len() &&
         afps_2d_pos_x_bit_count() == other.afps_2d_pos_x_bit_count() &&
         afps_2d_pos_y_bit_count() == other.afps_2d_pos_y_bit_count() &&
         afps_3d_pos_x_bit_count() == other.afps_3d_pos_x_bit_count() &&
         afps_3d_pos_y_bit_count() == other.afps_3d_pos_y_bit_count() &&
         afps_lod_bit_count() == other.afps_lod_bit_count() &&
         afps_override_eom_for_depth_flag() == other.afps_override_eom_for_depth_flag() &&
         afps_override_eom_for_depth_flag() == other.afps_override_eom_for_depth_flag() &&
         afps_raw_3d_pos_bit_count_explicit_mode_flag() ==
             other.afps_raw_3d_pos_bit_count_explicit_mode_flag() &&
         afps_extension_present_flag() == other.afps_extension_present_flag();
}

auto AtlasFrameParameterSetRBSP::operator!=(const AtlasFrameParameterSetRBSP &other) const noexcept
    -> bool {
  return !operator==(other);
}

auto AtlasFrameParameterSetRBSP::decodeFrom(
    istream &stream, const std::vector<AtlasSequenceParameterSetRBSP> &aspses)
    -> AtlasFrameParameterSetRBSP {
  auto x = AtlasFrameParameterSetRBSP{};
  InputBitstream bitstream{stream};

  x.afps_atlas_frame_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));
  VERIFY_VPCCBITSTREAM(x.afps_atlas_frame_parameter_set_id() <= 63);

  x.afps_atlas_sequence_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));
  VERIFY_VPCCBITSTREAM(x.afps_atlas_sequence_parameter_set_id() <= 15);
  VERIFY_VPCCBITSTREAM(x.afps_atlas_sequence_parameter_set_id() < aspses.size());
  const auto &asps = aspses[x.afps_atlas_sequence_parameter_set_id()];

  x.atlas_frame_tile_information(AtlasFrameTileInformation::decodeFrom(bitstream));

  x.afps_num_ref_idx_default_active(uint8_t(bitstream.getUExpGolomb() + 1));
  VERIFY_VPCCBITSTREAM(x.afps_num_ref_idx_default_active() <= 15);

  x.afps_additional_lt_afoc_lsb_len(uint8_t(bitstream.getUExpGolomb()));
  VERIFY_VPCCBITSTREAM(x.afps_additional_lt_afoc_lsb_len() <=
                       32 - asps.asps_log2_max_atlas_frame_order_cnt_lsb());
  VERIFY_VPCCBITSTREAM(asps.asps_long_term_ref_atlas_frames_flag() ||
                       x.afps_additional_lt_afoc_lsb_len() == 0);

  x.afps_2d_pos_x_bit_count(uint8_t(bitstream.readBits(4) + 1));
  x.afps_2d_pos_y_bit_count(uint8_t(bitstream.readBits(4) + 1));
  x.afps_3d_pos_x_bit_count(uint8_t(bitstream.readBits(5) + 1));
  x.afps_3d_pos_y_bit_count(uint8_t(bitstream.readBits(5) + 1));
  x.afps_lod_bit_count(uint8_t(bitstream.readBits(5)));

  x.afps_override_eom_for_depth_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.afps_override_eom_for_depth_flag());

  x.afps_raw_3d_pos_bit_count_explicit_mode_flag(bitstream.getFlag());

  x.afps_extension_present_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.afps_extension_present_flag());

  bitstream.rbspTrailingBits();

  return x;
}

void AtlasFrameParameterSetRBSP::encodeTo(
    ostream &stream, const std::vector<AtlasSequenceParameterSetRBSP> &aspses) const {
  OutputBitstream bitstream{stream};

  VERIFY_VPCCBITSTREAM(afps_atlas_frame_parameter_set_id() <= 63);
  bitstream.putUExpGolomb(afps_atlas_frame_parameter_set_id());

  VERIFY_VPCCBITSTREAM(afps_atlas_sequence_parameter_set_id() <= 15);
  VERIFY_VPCCBITSTREAM(afps_atlas_sequence_parameter_set_id() < aspses.size());
  bitstream.putUExpGolomb(afps_atlas_sequence_parameter_set_id());
  const auto &asps = aspses[afps_atlas_sequence_parameter_set_id()];

  atlas_frame_tile_information().encodeTo(bitstream);

  VERIFY_VPCCBITSTREAM(1 <= afps_num_ref_idx_default_active() &&
                       afps_num_ref_idx_default_active() <= 15);
  bitstream.putUExpGolomb(afps_num_ref_idx_default_active() - 1);

  VERIFY_VPCCBITSTREAM(afps_additional_lt_afoc_lsb_len() <=
                       32 - asps.asps_log2_max_atlas_frame_order_cnt_lsb());
  VERIFY_VPCCBITSTREAM(asps.asps_long_term_ref_atlas_frames_flag() ||
                       afps_additional_lt_afoc_lsb_len() == 0);
  bitstream.putUExpGolomb(afps_additional_lt_afoc_lsb_len());

  VERIFY_VPCCBITSTREAM(1 <= afps_2d_pos_x_bit_count() && afps_2d_pos_x_bit_count() <= 16);
  bitstream.writeBits(afps_2d_pos_x_bit_count() - 1, 4);

  VERIFY_VPCCBITSTREAM(1 <= afps_2d_pos_y_bit_count() && afps_2d_pos_y_bit_count() <= 16);
  bitstream.writeBits(afps_2d_pos_y_bit_count() - 1, 4);

  VERIFY_VPCCBITSTREAM(1 <= afps_3d_pos_x_bit_count() && afps_3d_pos_x_bit_count() <= 32);
  bitstream.writeBits(afps_3d_pos_x_bit_count() - 1, 5);

  VERIFY_VPCCBITSTREAM(1 <= afps_3d_pos_y_bit_count() && afps_3d_pos_y_bit_count() <= 32);
  bitstream.writeBits(afps_3d_pos_y_bit_count() - 1, 5);

  VERIFY_VPCCBITSTREAM(afps_lod_bit_count() <= 31);
  bitstream.writeBits(afps_lod_bit_count(), 5);

  VERIFY_MIVBITSTREAM(!afps_override_eom_for_depth_flag());
  bitstream.putFlag(afps_override_eom_for_depth_flag());

  bitstream.putFlag(afps_raw_3d_pos_bit_count_explicit_mode_flag());

  VERIFY_MIVBITSTREAM(!afps_extension_present_flag());
  bitstream.putFlag(afps_extension_present_flag());

  bitstream.rbspTrailingBits();
}
} // namespace TMIV::VpccBitstream
