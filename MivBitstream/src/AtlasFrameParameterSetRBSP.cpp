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

#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/MivDecoderMode.h>

#include "verify.h"

#include <ostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const AtlasFrameTileInformation & /* unused */) -> ostream & {
  return stream << "afti_single_tile_in_atlas_frame_flag=true\n";
}

auto AtlasFrameTileInformation::decodeFrom(InputBitstream &bitstream) -> AtlasFrameTileInformation {
  const auto afti_single_tile_in_atlas_frame_flag = bitstream.getFlag();
  // NOTE(BK): The proposal is to restrict to afti_single_tile_in_atlas_frame_flag == 1, but for
  // sake of being able to parse the provided V-PCC bitstream, this implementation accepts more
  // as long as there is only a single tile and tile group.

  if (afti_single_tile_in_atlas_frame_flag) {
    return {};
  }

  const auto afti_uniform_tile_spacing_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!afti_uniform_tile_spacing_flag);

  const auto afti_num_tile_columns_minus1 = bitstream.getUExpGolomb<size_t>();
  VERIFY_MIVBITSTREAM(afti_num_tile_columns_minus1 == 0);

  const auto afti_num_tile_rows_minus1 = bitstream.getUExpGolomb<size_t>();
  VERIFY_MIVBITSTREAM(afti_num_tile_rows_minus1 == 0);

  const auto afti_single_tile_per_tile_group_flag = bitstream.getFlag();

  if (!afti_single_tile_per_tile_group_flag) {
    const auto afti_num_tile_groups_in_atlas_frame_minus1 = bitstream.getUExpGolomb<size_t>();
    VERIFY_MIVBITSTREAM(afti_num_tile_groups_in_atlas_frame_minus1 == 0);
  }

  const auto afti_signalled_tile_group_id_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!afti_signalled_tile_group_id_flag);

  return {};
}

void AtlasFrameTileInformation::encodeTo(OutputBitstream &bitstream) {
  constexpr auto afti_single_tile_in_atlas_frame_flag = true;
  bitstream.putFlag(afti_single_tile_in_atlas_frame_flag);
}

auto operator<<(ostream &stream, const AtlasFrameParameterSetRBSP &x) -> ostream & {
  stream << "afps_atlas_frame_parameter_set_id=" << int(x.afps_atlas_frame_parameter_set_id())
         << '\n';
  stream << "afps_atlas_sequence_parameter_set_id=" << int(x.afps_atlas_sequence_parameter_set_id())
         << '\n';
  stream << x.atlas_frame_tile_information();
  stream << "afps_output_flag_present_flag=" << boolalpha << x.afps_output_flag_present_flag()
         << '\n';
  stream << "afps_num_ref_idx_default_active_minus1="
         << int(x.afps_num_ref_idx_default_active_minus1()) << '\n';
  stream << "afps_additional_lt_afoc_lsb_len=" << int(x.afps_additional_lt_afoc_lsb_len()) << '\n';
  stream << "afps_3d_pos_x_bit_count_minus1=" << int(x.afps_3d_pos_x_bit_count_minus1()) << '\n';
  stream << "afps_3d_pos_y_bit_count_minus1=" << int(x.afps_3d_pos_y_bit_count_minus1()) << '\n';
  stream << "afps_lod_mode_enabled_flag=" << boolalpha << x.afps_lod_mode_enabled_flag() << '\n';
  stream << "afps_override_eom_for_depth_flag=" << boolalpha << x.afps_override_eom_for_depth_flag()
         << '\n';
  stream << "afps_raw_3d_pos_bit_count_explicit_mode_flag=" << boolalpha
         << x.afps_raw_3d_pos_bit_count_explicit_mode_flag() << '\n';
  stream << "afps_extension_present_flag=" << boolalpha << x.afps_extension_present_flag() << '\n';
  return stream;
}

auto AtlasFrameParameterSetRBSP::operator==(const AtlasFrameParameterSetRBSP &other) const noexcept
    -> bool {
  return afps_atlas_frame_parameter_set_id() == other.afps_atlas_frame_parameter_set_id() &&
         afps_atlas_sequence_parameter_set_id() == other.afps_atlas_sequence_parameter_set_id() &&
         atlas_frame_tile_information() == other.atlas_frame_tile_information() &&
         afps_output_flag_present_flag() == other.afps_output_flag_present_flag() &&
         afps_num_ref_idx_default_active_minus1() ==
             other.afps_num_ref_idx_default_active_minus1() &&
         afps_additional_lt_afoc_lsb_len() == other.afps_additional_lt_afoc_lsb_len() &&
         afps_3d_pos_x_bit_count_minus1() == other.afps_3d_pos_x_bit_count_minus1() &&
         afps_3d_pos_y_bit_count_minus1() == other.afps_3d_pos_y_bit_count_minus1() &&
         afps_lod_mode_enabled_flag() == other.afps_lod_mode_enabled_flag() &&
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

auto AtlasFrameParameterSetRBSP::decodeFrom(istream &stream,
                                            const std::vector<AtlasSequenceParameterSetRBSP> &aspsV)
    -> AtlasFrameParameterSetRBSP {
  auto x = AtlasFrameParameterSetRBSP{};
  InputBitstream bitstream{stream};

  x.afps_atlas_frame_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_VPCCBITSTREAM(x.afps_atlas_frame_parameter_set_id() <= 63);

  x.afps_atlas_sequence_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_VPCCBITSTREAM(x.afps_atlas_sequence_parameter_set_id() <= 15);
  VERIFY_VPCCBITSTREAM(x.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[x.afps_atlas_sequence_parameter_set_id()];

  x.atlas_frame_tile_information(AtlasFrameTileInformation::decodeFrom(bitstream));

  x.afps_output_flag_present_flag(bitstream.getFlag());

  x.afps_num_ref_idx_default_active_minus1(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_VPCCBITSTREAM(x.afps_num_ref_idx_default_active_minus1() <= 14);

  x.afps_additional_lt_afoc_lsb_len(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_VPCCBITSTREAM(x.afps_additional_lt_afoc_lsb_len() <=
                       32 - (asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));
  VERIFY_VPCCBITSTREAM(mode == MivDecoderMode::TMC2 ||
                       asps.asps_long_term_ref_atlas_frames_flag() ||
                       x.afps_additional_lt_afoc_lsb_len() == 0);

  x.afps_3d_pos_x_bit_count_minus1(bitstream.readBits<uint8_t>((5)));
  x.afps_3d_pos_y_bit_count_minus1(bitstream.readBits<uint8_t>((5)));
  x.afps_lod_mode_enabled_flag(bitstream.getFlag());

  x.afps_override_eom_for_depth_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.afps_override_eom_for_depth_flag());

  x.afps_raw_3d_pos_bit_count_explicit_mode_flag(bitstream.getFlag());

  x.afps_extension_present_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.afps_extension_present_flag());

  bitstream.rbspTrailingBits();

  return x;
}

void AtlasFrameParameterSetRBSP::encodeTo(
    ostream &stream, const std::vector<AtlasSequenceParameterSetRBSP> &aspsV) const {
  OutputBitstream bitstream{stream};

  VERIFY_VPCCBITSTREAM(afps_atlas_frame_parameter_set_id() <= 63);
  bitstream.putUExpGolomb(afps_atlas_frame_parameter_set_id());

  VERIFY_VPCCBITSTREAM(afps_atlas_sequence_parameter_set_id() <= 15);
  VERIFY_VPCCBITSTREAM(afps_atlas_sequence_parameter_set_id() < aspsV.size());
  bitstream.putUExpGolomb(afps_atlas_sequence_parameter_set_id());
  const auto &asps = aspsV[afps_atlas_sequence_parameter_set_id()];

  atlas_frame_tile_information().encodeTo(bitstream);

  bitstream.putFlag(afps_output_flag_present_flag());

  VERIFY_VPCCBITSTREAM(afps_num_ref_idx_default_active_minus1() <= 14);
  bitstream.putUExpGolomb(afps_num_ref_idx_default_active_minus1());

  VERIFY_VPCCBITSTREAM(afps_additional_lt_afoc_lsb_len() <=
                       32 - (asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));
  VERIFY_VPCCBITSTREAM(asps.asps_long_term_ref_atlas_frames_flag() ||
                       afps_additional_lt_afoc_lsb_len() == 0);
  bitstream.putUExpGolomb(afps_additional_lt_afoc_lsb_len());

  VERIFY_VPCCBITSTREAM(afps_3d_pos_x_bit_count_minus1() < 32);
  bitstream.writeBits(afps_3d_pos_x_bit_count_minus1(), 5);

  VERIFY_VPCCBITSTREAM(afps_3d_pos_y_bit_count_minus1() < 32);
  bitstream.writeBits(afps_3d_pos_y_bit_count_minus1(), 5);

  bitstream.putFlag(afps_lod_mode_enabled_flag());

  VERIFY_MIVBITSTREAM(!afps_override_eom_for_depth_flag());
  bitstream.putFlag(afps_override_eom_for_depth_flag());

  bitstream.putFlag(afps_raw_3d_pos_bit_count_explicit_mode_flag());

  VERIFY_MIVBITSTREAM(!afps_extension_present_flag());
  bitstream.putFlag(afps_extension_present_flag());

  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
