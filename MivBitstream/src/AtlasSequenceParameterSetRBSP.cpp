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

#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

#include <TMIV/MivBitstream/MivDecoder.h>

#include "verify.h"
#include <TMIV/Common/Bitstream.h>

#include <ostream>
#include <utility>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
RefListStruct::RefListStruct(vector<int16_t> deltaAfocSt) : m_deltaAfocSt{move(deltaAfocSt)} {}

auto RefListStruct::num_ref_entries() const noexcept -> size_t { return m_deltaAfocSt.size(); }

auto RefListStruct::deltaAfocSt(size_t i) const noexcept -> int16_t {
  VERIFY_VPCCBITSTREAM(i < num_ref_entries());
  return m_deltaAfocSt[i];
}

auto RefListStruct::printTo(ostream &stream, uint8_t rlsIdx) const -> ostream & {
  stream << "num_ref_entries( " << int(rlsIdx) << " )=" << int(num_ref_entries()) << '\n';
  for (size_t i = 0; i < num_ref_entries(); ++i) {
    stream << "DeltaAfocSt( " << int(rlsIdx) << ", " << i << " )=" << deltaAfocSt(i) << '\n';
  }
  return stream;
}

auto RefListStruct::operator==(const RefListStruct &other) const noexcept -> bool {
  return m_deltaAfocSt == other.m_deltaAfocSt;
}

auto RefListStruct::operator!=(const RefListStruct &other) const noexcept -> bool {
  return !operator==(other);
}

auto RefListStruct::decodeFrom(InputBitstream &bitstream, const AtlasSequenceParameterSetRBSP &asps)
    -> RefListStruct {
  VERIFY_MIVBITSTREAM(!asps.asps_long_term_ref_atlas_frames_flag());
  auto deltaAfocSt = vector<int16_t>(bitstream.getUExpGolomb(), 0);

  for (auto &x : deltaAfocSt) {
    const auto abs_delta_afoc_st = int(bitstream.getUExpGolomb());
    VERIFY_VPCCBITSTREAM(0 <= abs_delta_afoc_st && abs_delta_afoc_st <= INT16_MAX);

    if (abs_delta_afoc_st != 0) {
      const auto strpf_entry_sign_flag = bitstream.getFlag();
      x = strpf_entry_sign_flag ? abs_delta_afoc_st : -abs_delta_afoc_st;
    }
  }

  return RefListStruct{deltaAfocSt};
}

void RefListStruct::encodeTo(OutputBitstream &bitstream,
                             const AtlasSequenceParameterSetRBSP &asps) const {
  VERIFY_MIVBITSTREAM(!asps.asps_long_term_ref_atlas_frames_flag());
  bitstream.putUExpGolomb(num_ref_entries());

  for (auto x : m_deltaAfocSt) {
    bitstream.putUExpGolomb(abs(x));
    if (x != 0) {
      bitstream.putFlag(x > 0);
    }
  }
}

auto MivAtlasSequenceParams::masp_omaf_v1_compatible_flag() const noexcept {
  VERIFY_MIVBITSTREAM(m_masp_omaf_v1_compatible_flag.has_value());
  return *m_masp_omaf_v1_compatible_flag;
}

auto MivAtlasSequenceParams::masp_geometry_frame_width_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(m_masp_geometry_frame_width_minus1.has_value());
  return *m_masp_geometry_frame_width_minus1;
}

auto MivAtlasSequenceParams::masp_geometry_frame_height_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(m_masp_geometry_frame_height_minus1.has_value());
  return *m_masp_geometry_frame_height_minus1;
}

auto operator<<(ostream &stream, const MivAtlasSequenceParams &x) -> ostream & {
  if (x.m_masp_omaf_v1_compatible_flag) {
    stream << "masp_omaf_v1_compatible_flag=" << x.masp_omaf_v1_compatible_flag() << '\n';
  }
  stream << "masp_group_id=" << x.masp_group_id() << "\nmasp_auxiliary_atlas_flag=" << boolalpha
         << x.masp_auxiliary_atlas_flag() << "\nmasp_depth_occ_map_threshold_flag=" << boolalpha
         << x.masp_depth_occ_map_threshold_flag() << '\n';
  if (x.m_masp_geometry_frame_width_minus1 && x.m_masp_geometry_frame_height_minus1) {
    stream << "masp_geometry_frame_width_minus1=" << x.masp_geometry_frame_width_minus1()
           << "\nmasp_geometry_frame_height_minus1=" << x.masp_geometry_frame_height_minus1()
           << '\n';
  }
  return stream;
}

auto MivAtlasSequenceParams::decodeFrom(InputBitstream &bitstream, const VpccUnitHeader &vuh,
                                        const VpccParameterSet &vps) -> MivAtlasSequenceParams {
  auto x = MivAtlasSequenceParams{};
  if (vuh.vuh_atlas_id() == 0) {
    x.masp_omaf_v1_compatible_flag(bitstream.getFlag());
  }
  x.masp_group_id(
      unsigned(bitstream.getUVar(vps.miv_sequence_params().msp_num_groups_minus1() + 1)));
  x.masp_auxiliary_atlas_flag(bitstream.getFlag());
  x.masp_depth_occ_map_threshold_flag(bitstream.getFlag());
  if (vps.miv_sequence_params().msp_geometry_scale_enabled_flag()) {
    x.masp_geometry_frame_width_minus1(
        uint16_t(bitstream.getUVar(vps.vps_frame_width(vuh.vuh_atlas_id()))));
    x.masp_geometry_frame_height_minus1(
        uint16_t(bitstream.getUVar(vps.vps_frame_height(vuh.vuh_atlas_id()))));
  }
  return x;
}

void MivAtlasSequenceParams::encodeTo(OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                                      const VpccParameterSet &vps) const {
  if (vuh.vuh_atlas_id() == 0) {
    bitstream.putFlag(masp_omaf_v1_compatible_flag());
  }
  bitstream.putUVar(masp_group_id(), vps.miv_sequence_params().msp_num_groups_minus1() + 1);
  bitstream.putFlag(masp_auxiliary_atlas_flag());
  bitstream.putFlag(masp_depth_occ_map_threshold_flag());
  if (vps.miv_sequence_params().msp_geometry_scale_enabled_flag()) {
    bitstream.putUVar(masp_geometry_frame_width_minus1(), vps.vps_frame_width(vuh.vuh_atlas_id()));
    bitstream.putUVar(masp_geometry_frame_height_minus1(),
                      vps.vps_frame_height(vuh.vuh_atlas_id()));
  }
}

auto AtlasSequenceParameterSetRBSP::asps_num_ref_atlas_frame_lists_in_asps() const noexcept
    -> uint8_t {
  return uint8_t(m_ref_list_structs.size());
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx) const -> const RefListStruct & {
  VERIFY_VPCCBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  return m_ref_list_structs[rlsIdx];
}

auto AtlasSequenceParameterSetRBSP::miv_atlas_sequence_params() const noexcept
    -> const MivAtlasSequenceParams & {
  VERIFY_MIVBITSTREAM(m_miv_atlas_sequence_params.has_value());
  return *m_miv_atlas_sequence_params;
}

auto AtlasSequenceParameterSetRBSP::asps_num_ref_atlas_frame_lists_in_asps(const size_t value)
    -> AtlasSequenceParameterSetRBSP & {
  m_ref_list_structs.resize(value);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx, RefListStruct value)
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_VPCCBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  m_ref_list_structs[rlsIdx] = move(value);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx) -> RefListStruct & {
  VERIFY_VPCCBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  return m_ref_list_structs[rlsIdx];
}

auto AtlasSequenceParameterSetRBSP::asps_max_projections_minus1() const noexcept -> unsigned {
  VERIFY_VPCCBITSTREAM(asps_extended_projection_enabled_flag());
  VERIFY_VPCCBITSTREAM(m_asps_max_projections_minus1.has_value());
  return *m_asps_max_projections_minus1;
}

auto AtlasSequenceParameterSetRBSP::asps_max_projections_minus1(const unsigned value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_VPCCBITSTREAM(asps_extended_projection_enabled_flag());
  m_asps_max_projections_minus1 = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::miv_atlas_sequence_params() noexcept
    -> MivAtlasSequenceParams & {
  VERIFY_MIVBITSTREAM(asps_miv_extension_present_flag());
  if (!m_miv_atlas_sequence_params) {
    m_miv_atlas_sequence_params = MivAtlasSequenceParams{};
  }
  return *m_miv_atlas_sequence_params;
}

auto operator<<(ostream &stream, const AtlasSequenceParameterSetRBSP &x) -> ostream & {
  stream << "asps_atlas_sequence_parameter_set_id=" << int(x.asps_atlas_sequence_parameter_set_id())
         << "\nasps_frame_width=" << x.asps_frame_width()
         << "\nasps_frame_height=" << x.asps_frame_height()
         << "\nasps_log2_patch_packing_block_size=" << int(x.asps_log2_patch_packing_block_size())
         << "\nasps_log2_max_atlas_frame_order_cnt_lsb_minus4="
         << int(x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4())
         << "\nasps_max_dec_atlas_frame_buffering_minus1="
         << x.asps_max_dec_atlas_frame_buffering_minus1()
         << "\nasps_long_term_ref_atlas_frames_flag=" << boolalpha
         << x.asps_long_term_ref_atlas_frames_flag() << "\nasps_num_ref_atlas_frame_lists_in_asps="
         << int(x.asps_num_ref_atlas_frame_lists_in_asps()) << '\n';

  for (int rlsIdx = 0; rlsIdx < x.asps_num_ref_atlas_frame_lists_in_asps(); ++rlsIdx) {
    x.ref_list_struct(rlsIdx).printTo(stream, rlsIdx);
  }

  stream << "asps_use_eight_orientations_flag=" << boolalpha << x.asps_use_eight_orientations_flag()
         << "\nasps_extended_projection_enabled_flag=" << boolalpha
         << x.asps_extended_projection_enabled_flag() << '\n';

  if (x.asps_extended_projection_enabled_flag()) {
    stream << "asps_max_projections_minus1=" << x.asps_max_projections_minus1() << '\n';
  }

  stream << "asps_normal_axis_limits_quantization_enabled_flag=" << boolalpha
         << x.asps_normal_axis_limits_quantization_enabled_flag()
         << "\nasps_normal_axis_max_delta_value_enabled_flag=" << boolalpha
         << x.asps_normal_axis_max_delta_value_enabled_flag()
         << "\nasps_remove_duplicate_point_enabled_flag=" << boolalpha
         << x.asps_remove_duplicate_point_enabled_flag()
         << "\nasps_pixel_deinterleaving_flag=" << boolalpha << x.asps_pixel_deinterleaving_flag()
         << "\nasps_patch_precedence_order_flag=" << boolalpha
         << x.asps_patch_precedence_order_flag()
         << "\nasps_patch_size_quantizer_present_flag=" << boolalpha
         << x.asps_patch_size_quantizer_present_flag()
         << "\nasps_raw_patch_enabled_flag=" << boolalpha << x.asps_raw_patch_enabled_flag()
         << "\nasps_eom_patch_enabled_flag=" << boolalpha << x.asps_eom_patch_enabled_flag()
         << "\nasps_point_local_reconstruction_enabled_flag=" << boolalpha
         << x.asps_point_local_reconstruction_enabled_flag()
         << "\nasps_map_count_minus1=" << int(x.asps_map_count_minus1())
         << "\nasps_vui_parameters_present_flag=" << boolalpha
         << x.asps_vui_parameters_present_flag() << "\nasps_extension_present_flag=" << boolalpha
         << x.asps_extension_present_flag() << '\n';

  if (x.asps_extension_present_flag()) {
    stream << "asps_miv_extension_present_flag=" << boolalpha << x.asps_miv_extension_present_flag()
           << '\n';

    if (x.asps_miv_extension_present_flag()) {
      stream << x.miv_atlas_sequence_params();
    }

    stream << "asps_extension2_present_flag=" << boolalpha << x.asps_extension2_present_flag()
           << '\n';
  }
  return stream;
}

auto AtlasSequenceParameterSetRBSP::operator==(const AtlasSequenceParameterSetRBSP &other) const
    noexcept -> bool {
  return m_asps_atlas_sequence_parameter_set_id == other.m_asps_atlas_sequence_parameter_set_id &&
         m_asps_frame_width == other.m_asps_frame_width &&
         m_asps_frame_height == other.m_asps_frame_height &&
         m_asps_log2_patch_packing_block_size == other.m_asps_log2_patch_packing_block_size &&
         m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4 ==
             other.m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4 &&
         m_asps_max_dec_atlas_frame_buffering_minus1 ==
             other.m_asps_max_dec_atlas_frame_buffering_minus1 &&
         m_asps_long_term_ref_atlas_frames_flag == other.m_asps_long_term_ref_atlas_frames_flag &&
         asps_num_ref_atlas_frame_lists_in_asps() ==
             other.asps_num_ref_atlas_frame_lists_in_asps() &&
         m_asps_use_eight_orientations_flag == other.m_asps_use_eight_orientations_flag &&
         m_asps_extended_projection_enabled_flag == other.m_asps_extended_projection_enabled_flag &&
         m_asps_max_projections_minus1 == other.m_asps_max_projections_minus1 &&
         m_asps_normal_axis_limits_quantization_enabled_flag ==
             other.m_asps_normal_axis_limits_quantization_enabled_flag &&
         m_asps_normal_axis_max_delta_value_enabled_flag ==
             other.m_asps_normal_axis_max_delta_value_enabled_flag &&
         m_asps_remove_duplicate_point_enabled_flag ==
             other.m_asps_remove_duplicate_point_enabled_flag &&
         m_asps_pixel_deinterleaving_flag == other.m_asps_pixel_deinterleaving_flag &&
         m_asps_patch_precedence_order_flag == other.m_asps_patch_precedence_order_flag &&
         m_asps_patch_size_quantizer_present_flag ==
             other.m_asps_patch_size_quantizer_present_flag &&
         m_asps_raw_patch_enabled_flag == other.m_asps_raw_patch_enabled_flag &&
         m_asps_eom_patch_enabled_flag == other.m_asps_eom_patch_enabled_flag &&
         m_asps_point_local_reconstruction_enabled_flag ==
             other.m_asps_point_local_reconstruction_enabled_flag &&
         m_asps_map_count_minus1 == other.m_asps_map_count_minus1 &&
         m_asps_vui_parameters_present_flag == other.m_asps_vui_parameters_present_flag &&
         m_asps_extension_present_flag == other.m_asps_extension_present_flag &&
         m_asps_miv_extension_present_flag == other.m_asps_miv_extension_present_flag &&
         m_asps_extension2_present_flag == other.m_asps_extension2_present_flag;
}

auto AtlasSequenceParameterSetRBSP::operator!=(const AtlasSequenceParameterSetRBSP &other) const
    noexcept -> bool {
  return !operator==(other);
}

auto AtlasSequenceParameterSetRBSP::decodeFrom(istream &stream, const VpccUnitHeader &vuh,
                                               const VpccParameterSet &vps)
    -> AtlasSequenceParameterSetRBSP {
  auto x = AtlasSequenceParameterSetRBSP{};
  InputBitstream bitstream{stream};

  x.asps_atlas_sequence_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));
  VERIFY_VPCCBITSTREAM(x.asps_atlas_sequence_parameter_set_id() <= 15);

  x.asps_frame_width(bitstream.getUint16());
  VERIFY_VPCCBITSTREAM(vps.vps_frame_width(vuh.vuh_atlas_id()) == x.asps_frame_width());

  x.asps_frame_height(bitstream.getUint16());
  VERIFY_VPCCBITSTREAM(vps.vps_frame_height(vuh.vuh_atlas_id()) == x.asps_frame_height());

  x.asps_log2_patch_packing_block_size(uint8_t(bitstream.readBits(3)));
  VERIFY_VPCCBITSTREAM(x.asps_log2_patch_packing_block_size() <= 7);

  x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4(uint8_t(bitstream.getUExpGolomb()));
  VERIFY_VPCCBITSTREAM(x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() <= 12);

  x.asps_max_dec_atlas_frame_buffering_minus1(uint8_t(bitstream.getUExpGolomb()));

  x.asps_long_term_ref_atlas_frames_flag(bitstream.getFlag());

  x.asps_num_ref_atlas_frame_lists_in_asps(bitstream.getUExpGolomb());
  VERIFY_VPCCBITSTREAM(x.asps_num_ref_atlas_frame_lists_in_asps() <= 64);

  for (int rlsIdx = 0; rlsIdx < x.asps_num_ref_atlas_frame_lists_in_asps(); ++rlsIdx) {
    x.ref_list_struct(rlsIdx, RefListStruct::decodeFrom(bitstream, x));
  }

  x.asps_use_eight_orientations_flag(bitstream.getFlag());

  x.asps_extended_projection_enabled_flag(bitstream.getFlag());
  if (x.asps_extended_projection_enabled_flag()) {
    x.asps_max_projections_minus1(unsigned(bitstream.getUExpGolomb()));
  }

  x.asps_normal_axis_limits_quantization_enabled_flag(bitstream.getFlag());
  x.asps_normal_axis_max_delta_value_enabled_flag(bitstream.getFlag());
  x.asps_remove_duplicate_point_enabled_flag(bitstream.getFlag());

  x.asps_pixel_deinterleaving_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_pixel_deinterleaving_flag());

  x.asps_patch_precedence_order_flag(bitstream.getFlag());
  x.asps_patch_size_quantizer_present_flag(bitstream.getFlag());

  x.asps_raw_patch_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_raw_patch_enabled_flag());

  x.asps_eom_patch_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_eom_patch_enabled_flag());

  x.asps_point_local_reconstruction_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_point_local_reconstruction_enabled_flag());

  x.asps_map_count_minus1(uint8_t(bitstream.readBits(4)));
  VERIFY_VPCCBITSTREAM(x.asps_map_count_minus1() == vps.vps_map_count_minus1(vuh.vuh_atlas_id()));

  x.asps_vui_parameters_present_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_vui_parameters_present_flag());

  x.asps_extension_present_flag(bitstream.getFlag());

  if (MivDecoder::mode == MivDecoder::Mode::MIV) {
    const auto asps_extension_bit_equal_to_one = x.asps_extension_present_flag();
    VERIFY_MIVBITSTREAM(asps_extension_bit_equal_to_one);

    x.asps_miv_extension_present_flag(bitstream.getFlag());

    if (x.asps_miv_extension_present_flag()) {
      x.miv_atlas_sequence_params() = MivAtlasSequenceParams::decodeFrom(bitstream, vuh, vps);
    }

    x.asps_extension2_present_flag(bitstream.getFlag());
    VERIFY_MIVBITSTREAM(!x.asps_extension2_present_flag());
  } else {
    VERIFY_VPCCBITSTREAM(!x.asps_extension_present_flag());
  }

  bitstream.rbspTrailingBits();

  return x;
}

void AtlasSequenceParameterSetRBSP::encodeTo(ostream &stream, const VpccUnitHeader &vuh,
                                             const VpccParameterSet &vps) const {
  OutputBitstream bitstream{stream};

  VERIFY_VPCCBITSTREAM(asps_atlas_sequence_parameter_set_id() <= 15);
  bitstream.putUExpGolomb(asps_atlas_sequence_parameter_set_id());

  VERIFY_VPCCBITSTREAM(asps_frame_width() == vps.vps_frame_width(vuh.vuh_atlas_id()));
  bitstream.putUint16(asps_frame_width());

  VERIFY_VPCCBITSTREAM(asps_frame_height() == vps.vps_frame_height(vuh.vuh_atlas_id()));
  bitstream.putUint16(asps_frame_height());

  VERIFY_VPCCBITSTREAM(asps_log2_patch_packing_block_size() <= 7);
  bitstream.writeBits(asps_log2_patch_packing_block_size(), 3);

  VERIFY_VPCCBITSTREAM(asps_log2_max_atlas_frame_order_cnt_lsb_minus4() <= 12);
  bitstream.putUExpGolomb(asps_log2_max_atlas_frame_order_cnt_lsb_minus4());

  bitstream.putUExpGolomb(asps_max_dec_atlas_frame_buffering_minus1());

  bitstream.putFlag(asps_long_term_ref_atlas_frames_flag());

  VERIFY_VPCCBITSTREAM(asps_num_ref_atlas_frame_lists_in_asps() <= 64);
  bitstream.putUExpGolomb(asps_num_ref_atlas_frame_lists_in_asps());

  for (int rlsIdx = 0; rlsIdx < asps_num_ref_atlas_frame_lists_in_asps(); ++rlsIdx) {
    ref_list_struct(rlsIdx).encodeTo(bitstream, *this);
  }

  bitstream.putFlag(asps_use_eight_orientations_flag());

  bitstream.putFlag(asps_extended_projection_enabled_flag());
  if (asps_extended_projection_enabled_flag()) {
    bitstream.putUExpGolomb(asps_max_projections_minus1());
  }

  bitstream.putFlag(asps_normal_axis_limits_quantization_enabled_flag());
  bitstream.putFlag(asps_normal_axis_max_delta_value_enabled_flag());
  bitstream.putFlag(asps_remove_duplicate_point_enabled_flag());

  VERIFY_MIVBITSTREAM(!asps_pixel_deinterleaving_flag());
  bitstream.putFlag(asps_pixel_deinterleaving_flag());

  bitstream.putFlag(asps_patch_precedence_order_flag());
  bitstream.putFlag(asps_patch_size_quantizer_present_flag());

  VERIFY_MIVBITSTREAM(!asps_raw_patch_enabled_flag());
  bitstream.putFlag(asps_raw_patch_enabled_flag());

  VERIFY_MIVBITSTREAM(!asps_eom_patch_enabled_flag());
  bitstream.putFlag(asps_eom_patch_enabled_flag());

  VERIFY_MIVBITSTREAM(!asps_point_local_reconstruction_enabled_flag());
  bitstream.putFlag(asps_point_local_reconstruction_enabled_flag());

  VERIFY_VPCCBITSTREAM(asps_map_count_minus1() == vps.vps_map_count_minus1(vuh.vuh_atlas_id()));
  bitstream.writeBits(asps_map_count_minus1(), 4);

  VERIFY_MIVBITSTREAM(!asps_vui_parameters_present_flag());
  bitstream.putFlag(asps_vui_parameters_present_flag());

  VERIFY_MIVBITSTREAM(asps_extension_present_flag());
  bitstream.putFlag(asps_extension_present_flag());

  VERIFY_MIVBITSTREAM(asps_extension_present_flag());
  bitstream.putFlag(asps_miv_extension_present_flag());

  if (asps_miv_extension_present_flag()) {
    miv_atlas_sequence_params().encodeTo(bitstream, vuh, vps);
  }

  VERIFY_MIVBITSTREAM(!asps_extension2_present_flag());
  bitstream.putFlag(asps_extension2_present_flag());

  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
