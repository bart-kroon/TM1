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

#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/MivDecoderMode.h>
#include <TMIV/MivBitstream/verify.h>

#include <ostream>
#include <utility>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
RefListStruct::RefListStruct(vector<int16_t> deltaAfocSt) : m_deltaAfocSt{move(deltaAfocSt)} {}

auto RefListStruct::num_ref_entries() const noexcept -> size_t { return m_deltaAfocSt.size(); }

auto RefListStruct::deltaAfocSt(size_t i) const noexcept -> int16_t {
  VERIFY_V3CBITSTREAM(i < num_ref_entries());
  return m_deltaAfocSt[i];
}

auto RefListStruct::printTo(ostream &stream, uint8_t rlsIdx) const -> ostream & {
  stream << "num_ref_entries( " << int{rlsIdx} << " )=" << int(num_ref_entries()) << '\n';
  for (size_t i = 0; i < num_ref_entries(); ++i) {
    stream << "DeltaAfocSt( " << int{rlsIdx} << ", " << i << " )=" << deltaAfocSt(i) << '\n';
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
  auto deltaAfocSt = vector<int16_t>(bitstream.getUExpGolomb<size_t>(), 0);

  for (auto &x : deltaAfocSt) {
    const auto abs_delta_afoc_st = bitstream.getUExpGolomb<int>();
    VERIFY_V3CBITSTREAM(0 <= abs_delta_afoc_st && abs_delta_afoc_st <= INT16_MAX);

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

auto operator<<(ostream &stream, const AspsVpccExtension &x) -> ostream & {
  stream << "asps_vpcc_remove_duplicate_point_enabled_flag=" << boolalpha
         << x.asps_vpcc_remove_duplicate_point_enabled_flag() << '\n';
  return stream;
}

auto AspsVpccExtension::decodeFrom(Common::InputBitstream &bitstream,
                                   const AtlasSequenceParameterSetRBSP &asps) -> AspsVpccExtension {
  auto x = AspsVpccExtension{};
  x.asps_vpcc_remove_duplicate_point_enabled_flag(bitstream.getFlag());
  VERIFY_BITSTREAM(!asps.asps_plr_enabled_flag());
  return x;
}

void AspsVpccExtension::encodeTo(Common::OutputBitstream &bitstream,
                                 const AtlasSequenceParameterSetRBSP &asps) const {
  bitstream.putFlag(asps_vpcc_remove_duplicate_point_enabled_flag());
  VERIFY_BITSTREAM(!asps.asps_plr_enabled_flag());
}

auto AspsMivExtension::asme_geometry_scale_factor_x_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(m_asme_geometry_scale_factor_x_minus1.has_value());
  return *m_asme_geometry_scale_factor_x_minus1;
}

auto AspsMivExtension::asme_geometry_scale_factor_y_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(m_asme_geometry_scale_factor_y_minus1.has_value());
  return *m_asme_geometry_scale_factor_y_minus1;
}

auto AspsMivExtension::asme_occupancy_scale_factor_x_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(m_asme_occupancy_scale_factor_x_minus1.has_value());
  return *m_asme_occupancy_scale_factor_x_minus1;
}

auto AspsMivExtension::asme_occupancy_scale_factor_y_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(m_asme_occupancy_scale_factor_y_minus1.has_value());
  return *m_asme_occupancy_scale_factor_y_minus1;
}

auto operator<<(ostream &stream, const AspsMivExtension &x) -> ostream & {
  stream << "asme_group_id=" << x.asme_group_id() << '\n';
  stream << "asme_auxiliary_atlas_flag=" << boolalpha << x.asme_auxiliary_atlas_flag() << '\n';
  stream << "asme_depth_occ_map_threshold_flag=" << boolalpha << x.asme_depth_occ_threshold_flag()
         << '\n';
  if (x.m_asme_geometry_scale_factor_x_minus1 || x.m_asme_geometry_scale_factor_y_minus1) {
    stream << "asme_geometry_scale_factor_x_minus1=" << x.asme_geometry_scale_factor_x_minus1()
           << '\n';
    stream << "asme_geometry_scale_factor_y_minus1=" << x.asme_geometry_scale_factor_y_minus1()
           << '\n';
  }
  if (x.m_asme_occupancy_scale_factor_x_minus1 || x.m_asme_occupancy_scale_factor_y_minus1) {
    stream << "asme_occupancy_scale_factor_x_minus1=" << x.asme_occupancy_scale_factor_x_minus1()
           << '\n';
    stream << "asme_occupancy_scale_factor_y_minus1=" << x.asme_occupancy_scale_factor_y_minus1()
           << '\n';
  }
  stream << "asme_patch_constant_depth_flag=" << boolalpha << x.asme_patch_constant_depth_flag()
         << '\n';
  return stream;
}

auto AspsMivExtension::decodeFrom(InputBitstream &bitstream, const V3cParameterSet &vps)
    -> AspsMivExtension {
  auto x = AspsMivExtension{};
  x.asme_group_id(
      bitstream.getUVar<unsigned>(vps.vps_miv_extension().vme_num_groups_minus1() + uint64_t(1)));
  x.asme_auxiliary_atlas_flag(bitstream.getFlag());
  if (vps.vps_miv_extension().vme_embedded_occupancy_flag()) {
    x.asme_depth_occ_threshold_flag(bitstream.getFlag());
  }
  if (vps.vps_miv_extension().vme_geometry_scale_enabled_flag()) {
    x.asme_geometry_scale_factor_x_minus1(bitstream.getUExpGolomb<uint16_t>());
    x.asme_geometry_scale_factor_y_minus1(bitstream.getUExpGolomb<uint16_t>());
  }
  if (vps.vps_miv_extension().vme_occupancy_scale_enabled_flag()) {
    x.asme_occupancy_scale_factor_x_minus1(bitstream.getUExpGolomb<uint16_t>());
    x.asme_occupancy_scale_factor_y_minus1(bitstream.getUExpGolomb<uint16_t>());
  }
  x.asme_patch_constant_depth_flag(bitstream.getFlag());
  return x;
}

void AspsMivExtension::encodeTo(OutputBitstream &bitstream, const V3cParameterSet &vps) const {
  bitstream.putUVar(asme_group_id(), vps.vps_miv_extension().vme_num_groups_minus1() + uint64_t(1));
  bitstream.putFlag(asme_auxiliary_atlas_flag());
  if (vps.vps_miv_extension().vme_embedded_occupancy_flag()) {
    bitstream.putFlag(asme_depth_occ_threshold_flag());
  }
  if (vps.vps_miv_extension().vme_geometry_scale_enabled_flag()) {
    bitstream.putUExpGolomb(asme_geometry_scale_factor_x_minus1());
    bitstream.putUExpGolomb(asme_geometry_scale_factor_y_minus1());
  }
  if (vps.vps_miv_extension().vme_occupancy_scale_enabled_flag()) {
    bitstream.putUExpGolomb(asme_occupancy_scale_factor_x_minus1());
    bitstream.putUExpGolomb(asme_occupancy_scale_factor_y_minus1());
  }
  bitstream.putFlag(asme_patch_constant_depth_flag());
}

auto AtlasSequenceParameterSetRBSP::asps_num_ref_atlas_frame_lists_in_asps() const noexcept
    -> uint8_t {
  return uint8_t(m_ref_list_structs.size());
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx) const -> const RefListStruct & {
  VERIFY_V3CBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  return m_ref_list_structs[rlsIdx];
}

auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension() const noexcept
    -> const AspsVpccExtension & {
  VERIFY_V3CBITSTREAM(asps_vpcc_extension_flag());
  VERIFY_V3CBITSTREAM(m_asve.has_value());
  return *m_asve;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_extension() const noexcept
    -> const AspsMivExtension & {
  VERIFY_V3CBITSTREAM(asps_miv_extension_flag());
  VERIFY_V3CBITSTREAM(m_asme.has_value());
  return *m_asme;
}

auto AtlasSequenceParameterSetRBSP::aspsExtensionData() const noexcept -> const vector<bool> & {
  VERIFY_V3CBITSTREAM(asps_extension_6bits());
  VERIFY_V3CBITSTREAM(m_aspsExtensionData.has_value());
  return *m_aspsExtensionData;
}

auto AtlasSequenceParameterSetRBSP::asps_log2_max_atlas_frame_order_cnt_lsb_minus4(
    const uint8_t value) noexcept -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(value <= 12);
  m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4 = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_num_ref_atlas_frame_lists_in_asps(const size_t value)
    -> AtlasSequenceParameterSetRBSP & {
  m_ref_list_structs.resize(value);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension_flag(const bool value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(asps_extension_present_flag());
  m_asps_vpcc_extension_flag = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_extension_flag(const bool value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(asps_extension_present_flag());
  m_asps_miv_extension_flag = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_extension_6bits(const uint8_t value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(asps_extension_present_flag());
  VERIFY_V3CBITSTREAM(value < 0x40);
  m_asps_extension_6bits = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::aspsExtensionData(vector<bool> data) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(asps_extension_6bits());
  m_aspsExtensionData = move(data);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx, RefListStruct value)
    -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  m_ref_list_structs[rlsIdx] = move(value);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx) -> RefListStruct & {
  VERIFY_V3CBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  return m_ref_list_structs[rlsIdx];
}

auto AtlasSequenceParameterSetRBSP::asps_max_number_projections_minus1() const noexcept
    -> unsigned {
  VERIFY_V3CBITSTREAM(asps_extended_projection_enabled_flag());
  VERIFY_V3CBITSTREAM(m_asps_max_number_projections_minus1.has_value());
  return *m_asps_max_number_projections_minus1;
}

auto AtlasSequenceParameterSetRBSP::asps_max_number_projections_minus1(
    const unsigned value) noexcept -> AtlasSequenceParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(asps_extended_projection_enabled_flag());
  m_asps_max_number_projections_minus1 = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension() noexcept -> AspsVpccExtension & {
  VERIFY_MIVBITSTREAM(asps_vpcc_extension_flag());
  if (!m_asve) {
    m_asve = AspsVpccExtension{};
  }
  return *m_asve;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_extension() noexcept -> AspsMivExtension & {
  VERIFY_MIVBITSTREAM(asps_miv_extension_flag());
  if (!m_asme) {
    m_asme = AspsMivExtension{};
  }
  return *m_asme;
}

auto operator<<(ostream &stream, const AtlasSequenceParameterSetRBSP &x) -> ostream & {
  stream << "asps_atlas_sequence_parameter_set_id=" << int(x.asps_atlas_sequence_parameter_set_id())
         << '\n';
  stream << "asps_frame_width=" << x.asps_frame_width() << '\n';
  stream << "asps_frame_height=" << x.asps_frame_height() << '\n';
  stream << "asps_geometry_3d_bitdepth_minus1=" << int(x.asps_geometry_3d_bitdepth_minus1())
         << '\n';
  stream << "asps_geometry_2d_bitdepth_minus1=" << int(x.asps_geometry_2d_bitdepth_minus1())
         << '\n';
  stream << "asps_log2_max_atlas_frame_order_cnt_lsb_minus4="
         << int(x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4()) << '\n';
  stream << "asps_max_dec_atlas_frame_buffering_minus1="
         << x.asps_max_dec_atlas_frame_buffering_minus1() << '\n';
  stream << "asps_long_term_ref_atlas_frames_flag=" << boolalpha
         << x.asps_long_term_ref_atlas_frames_flag() << '\n';
  stream << "asps_num_ref_atlas_frame_lists_in_asps="
         << int(x.asps_num_ref_atlas_frame_lists_in_asps()) << '\n';
  for (int i = 0; i < x.asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
    x.ref_list_struct(i).printTo(stream, i);
  }
  stream << "asps_use_eight_orientations_flag=" << boolalpha << x.asps_use_eight_orientations_flag()
         << '\n';
  stream << "asps_extended_projection_enabled_flag=" << boolalpha
         << x.asps_extended_projection_enabled_flag() << '\n';
  if (x.asps_extended_projection_enabled_flag()) {
    stream << "asps_max_number_projections_minus1=" << x.asps_max_number_projections_minus1()
           << '\n';
  }
  stream << "asps_normal_axis_limits_quantization_enabled_flag=" << boolalpha
         << x.asps_normal_axis_limits_quantization_enabled_flag() << '\n';
  stream << "asps_normal_axis_max_delta_value_enabled_flag=" << boolalpha
         << x.asps_normal_axis_max_delta_value_enabled_flag() << '\n';
  stream << "asps_patch_precedence_order_flag=" << boolalpha << x.asps_patch_precedence_order_flag()
         << '\n';
  stream << "asps_log2_patch_packing_block_size=" << int(x.asps_log2_patch_packing_block_size())
         << '\n';
  stream << "asps_patch_size_quantizer_present_flag=" << boolalpha
         << x.asps_patch_size_quantizer_present_flag() << '\n';
  stream << "asps_map_count_minus1=" << int(x.asps_map_count_minus1()) << '\n';
  stream << "asps_pixel_deinterleaving_flag=" << boolalpha << x.asps_pixel_deinterleaving_flag()
         << '\n';
  stream << "asps_eom_patch_enabled_flag=" << boolalpha << x.asps_eom_patch_enabled_flag() << '\n';
  stream << "asps_raw_patch_enabled_flag=" << boolalpha << x.asps_raw_patch_enabled_flag() << '\n';
  stream << "asps_plr_enabled_flag=" << boolalpha << x.asps_plr_enabled_flag() << '\n';
  stream << "asps_vui_parameters_present_flag=" << boolalpha << x.asps_vui_parameters_present_flag()
         << '\n';
  stream << "asps_extension_present_flag=" << boolalpha << x.asps_extension_present_flag() << '\n';
  if (x.asps_extension_present_flag()) {
    stream << "asps_vpcc_extension_flag=" << boolalpha << x.asps_vpcc_extension_flag() << '\n';
    stream << "asps_miv_extension_flag=" << boolalpha << x.asps_miv_extension_flag() << '\n';
    stream << "asps_extension_6bits=" << int(x.asps_extension_6bits()) << '\n';
  }
  if (x.asps_vpcc_extension_flag()) {
    stream << x.asps_vpcc_extension();
  }
  if (x.asps_miv_extension_flag()) {
    stream << x.asps_miv_extension();
  }
  if (x.asps_extension_6bits()) {
    for (bool flag : x.aspsExtensionData()) {
      stream << "asps_extension_data_flag=" << boolalpha << flag << '\n';
    }
  }
  return stream;
}

auto AtlasSequenceParameterSetRBSP::operator==(
    const AtlasSequenceParameterSetRBSP &other) const noexcept -> bool {
  if (asps_atlas_sequence_parameter_set_id() != other.asps_atlas_sequence_parameter_set_id() ||
      asps_frame_width() != other.asps_frame_width() ||
      asps_frame_height() != other.asps_frame_height() ||
      asps_geometry_3d_bitdepth_minus1() != other.asps_geometry_3d_bitdepth_minus1() ||
      asps_geometry_2d_bitdepth_minus1() != other.asps_geometry_2d_bitdepth_minus1() ||
      asps_log2_max_atlas_frame_order_cnt_lsb_minus4() !=
          other.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() ||
      asps_max_dec_atlas_frame_buffering_minus1() !=
          other.asps_max_dec_atlas_frame_buffering_minus1() ||
      asps_long_term_ref_atlas_frames_flag() != other.asps_long_term_ref_atlas_frames_flag() ||
      asps_num_ref_atlas_frame_lists_in_asps() != other.asps_num_ref_atlas_frame_lists_in_asps()) {
    return false;
  }
  for (uint8_t i = 0; i < asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
    if (ref_list_struct(i) != other.ref_list_struct(i)) {
      return false;
    }
  }
  if (asps_use_eight_orientations_flag() != other.asps_use_eight_orientations_flag() ||
      asps_extended_projection_enabled_flag() != other.asps_extended_projection_enabled_flag()) {
    return false;
  }
  if (asps_extended_projection_enabled_flag() &&
      asps_max_number_projections_minus1() != other.asps_max_number_projections_minus1()) {
    return false;
  }
  if (asps_normal_axis_limits_quantization_enabled_flag() !=
          other.asps_normal_axis_limits_quantization_enabled_flag() ||
      asps_normal_axis_max_delta_value_enabled_flag() !=
          other.asps_normal_axis_max_delta_value_enabled_flag() ||
      asps_patch_precedence_order_flag() != other.asps_patch_precedence_order_flag() ||
      m_asps_log2_patch_packing_block_size != other.m_asps_log2_patch_packing_block_size ||
      asps_patch_size_quantizer_present_flag() != other.asps_patch_size_quantizer_present_flag() ||
      m_asps_map_count_minus1 != other.m_asps_map_count_minus1 ||
      asps_pixel_deinterleaving_flag() != other.asps_pixel_deinterleaving_flag() ||
      asps_eom_patch_enabled_flag() != other.asps_eom_patch_enabled_flag() ||
      asps_raw_patch_enabled_flag() != other.asps_raw_patch_enabled_flag() ||
      asps_plr_enabled_flag() != other.asps_plr_enabled_flag() ||
      asps_vui_parameters_present_flag() != other.asps_vui_parameters_present_flag() ||
      asps_extension_present_flag() != other.asps_extension_present_flag() ||
      asps_vpcc_extension_flag() != other.asps_vpcc_extension_flag() ||
      asps_miv_extension_flag() != other.asps_miv_extension_flag() ||
      asps_extension_6bits() != other.asps_extension_6bits()) {
    return false;
  }
  if (asps_vpcc_extension_flag() && asps_vpcc_extension() != other.asps_vpcc_extension()) {
    return false;
  }
  if (asps_miv_extension_flag() && asps_miv_extension() != other.asps_miv_extension()) {
    return false;
  }
  if (asps_extension_6bits() && aspsExtensionData() != other.aspsExtensionData()) {
    return false;
  }
  return true;
}

auto AtlasSequenceParameterSetRBSP::operator!=(
    const AtlasSequenceParameterSetRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto AtlasSequenceParameterSetRBSP::decodeFrom(istream &stream, const V3cUnitHeader &vuh,
                                               const V3cParameterSet &vps)
    -> AtlasSequenceParameterSetRBSP {
  auto x = AtlasSequenceParameterSetRBSP{};
  InputBitstream bitstream{stream};

  x.asps_atlas_sequence_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());

  x.asps_frame_width(bitstream.getUint16());
  const auto atlasIdx = vps.atlasIdxOf(vuh.vuh_atlas_id());
  VERIFY_V3CBITSTREAM(vps.vps_frame_width(atlasIdx) == x.asps_frame_width());

  x.asps_frame_height(bitstream.getUint16());
  VERIFY_V3CBITSTREAM(vps.vps_frame_height(atlasIdx) == x.asps_frame_height());

  x.asps_geometry_3d_bitdepth_minus1(bitstream.readBits<uint8_t>(5));
  x.asps_geometry_2d_bitdepth_minus1(bitstream.readBits<uint8_t>(5));

  x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() <= 12);

  x.asps_max_dec_atlas_frame_buffering_minus1(bitstream.getUExpGolomb<uint8_t>());
  x.asps_long_term_ref_atlas_frames_flag(bitstream.getFlag());

  x.asps_num_ref_atlas_frame_lists_in_asps(bitstream.getUExpGolomb<size_t>());
  VERIFY_V3CBITSTREAM(x.asps_num_ref_atlas_frame_lists_in_asps() <= 64);

  for (int i = 0; i < x.asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
    x.ref_list_struct(i, RefListStruct::decodeFrom(bitstream, x));
  }

  x.asps_use_eight_orientations_flag(bitstream.getFlag());
  x.asps_extended_projection_enabled_flag(bitstream.getFlag());

  if (x.asps_extended_projection_enabled_flag()) {
    x.asps_max_number_projections_minus1(bitstream.getUExpGolomb<unsigned>());
  }

  x.asps_normal_axis_limits_quantization_enabled_flag(bitstream.getFlag());
  x.asps_normal_axis_max_delta_value_enabled_flag(bitstream.getFlag());
  x.asps_patch_precedence_order_flag(bitstream.getFlag());

  x.asps_log2_patch_packing_block_size(bitstream.readBits<uint8_t>(3));
  VERIFY_V3CBITSTREAM(x.asps_log2_patch_packing_block_size() <= 7);

  x.asps_patch_size_quantizer_present_flag(bitstream.getFlag());

  x.asps_map_count_minus1(bitstream.readBits<uint8_t>(4));
  VERIFY_V3CBITSTREAM(x.asps_map_count_minus1() == vps.vps_map_count_minus1(atlasIdx));

  x.asps_pixel_deinterleaving_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_pixel_deinterleaving_flag());

  x.asps_eom_patch_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_eom_patch_enabled_flag());

  x.asps_raw_patch_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_raw_patch_enabled_flag());

  x.asps_plr_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_plr_enabled_flag());

  x.asps_vui_parameters_present_flag(bitstream.getFlag());
  LIMITATION(!x.asps_vui_parameters_present_flag());

  x.asps_extension_present_flag(bitstream.getFlag());

  if (x.asps_extension_present_flag()) {
    x.asps_vpcc_extension_flag(bitstream.getFlag());
    x.asps_miv_extension_flag(bitstream.getFlag());
    x.asps_extension_6bits(bitstream.readBits<uint8_t>(6));
  }
  if (x.asps_vpcc_extension_flag()) {
    x.asps_vpcc_extension() = AspsVpccExtension::decodeFrom(bitstream, x);
  }
  if (x.asps_miv_extension_flag()) {
    x.asps_miv_extension() = AspsMivExtension::decodeFrom(bitstream, vps);
  }
  if (x.asps_extension_6bits() != 0) {
    auto aspsExtensionData = vector<bool>{};
    while (bitstream.moreRbspData()) {
      const auto asps_extension_data_flag = bitstream.getFlag();
      aspsExtensionData.push_back(asps_extension_data_flag);
    }
    x.aspsExtensionData(move(aspsExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void AtlasSequenceParameterSetRBSP::encodeTo(ostream &stream, const V3cUnitHeader &vuh,
                                             const V3cParameterSet &vps) const {
  OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(asps_atlas_sequence_parameter_set_id());

  const auto atlasIdx = vps.atlasIdxOf(vuh.vuh_atlas_id());
  VERIFY_V3CBITSTREAM(asps_frame_width() == vps.vps_frame_width(atlasIdx));
  bitstream.putUint16(asps_frame_width());

  VERIFY_V3CBITSTREAM(asps_frame_height() == vps.vps_frame_height(atlasIdx));
  bitstream.putUint16(asps_frame_height());

  bitstream.writeBits(asps_geometry_3d_bitdepth_minus1(), 5);
  bitstream.writeBits(asps_geometry_2d_bitdepth_minus1(), 5);

  VERIFY_V3CBITSTREAM(asps_log2_max_atlas_frame_order_cnt_lsb_minus4() <= 12);
  bitstream.putUExpGolomb(asps_log2_max_atlas_frame_order_cnt_lsb_minus4());

  bitstream.putUExpGolomb(asps_max_dec_atlas_frame_buffering_minus1());
  bitstream.putFlag(asps_long_term_ref_atlas_frames_flag());

  VERIFY_V3CBITSTREAM(asps_num_ref_atlas_frame_lists_in_asps() <= 64);
  bitstream.putUExpGolomb(asps_num_ref_atlas_frame_lists_in_asps());

  for (int i = 0; i < asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
    ref_list_struct(i).encodeTo(bitstream, *this);
  }

  bitstream.putFlag(asps_use_eight_orientations_flag());
  bitstream.putFlag(asps_extended_projection_enabled_flag());

  if (asps_extended_projection_enabled_flag()) {
    bitstream.putUExpGolomb(asps_max_number_projections_minus1());
  }

  bitstream.putFlag(asps_normal_axis_limits_quantization_enabled_flag());
  bitstream.putFlag(asps_normal_axis_max_delta_value_enabled_flag());
  bitstream.putFlag(asps_patch_precedence_order_flag());

  VERIFY_V3CBITSTREAM(asps_log2_patch_packing_block_size() <= 7);
  bitstream.writeBits(asps_log2_patch_packing_block_size(), 3);

  bitstream.putFlag(asps_patch_size_quantizer_present_flag());

  VERIFY_V3CBITSTREAM(asps_map_count_minus1() == vps.vps_map_count_minus1(atlasIdx));
  bitstream.writeBits(asps_map_count_minus1(), 4);

  VERIFY_MIVBITSTREAM(!asps_pixel_deinterleaving_flag());
  bitstream.putFlag(asps_pixel_deinterleaving_flag());

  VERIFY_MIVBITSTREAM(!asps_eom_patch_enabled_flag());
  bitstream.putFlag(asps_eom_patch_enabled_flag());

  VERIFY_MIVBITSTREAM(!asps_raw_patch_enabled_flag());
  bitstream.putFlag(asps_raw_patch_enabled_flag());

  VERIFY_MIVBITSTREAM(!asps_plr_enabled_flag());
  bitstream.putFlag(asps_plr_enabled_flag());

  LIMITATION(!asps_vui_parameters_present_flag());
  bitstream.putFlag(asps_vui_parameters_present_flag());

  bitstream.putFlag(asps_extension_present_flag());

  if (asps_extension_present_flag()) {
    bitstream.putFlag(asps_vpcc_extension_flag());
    bitstream.putFlag(asps_miv_extension_flag());
    bitstream.writeBits(asps_extension_6bits(), 6);
  }
  if (asps_vpcc_extension_flag()) {
    asps_vpcc_extension().encodeTo(bitstream, *this);
  }
  if (asps_miv_extension_flag()) {
    asps_miv_extension().encodeTo(bitstream, vps);
  }
  if (asps_extension_6bits() != 0) {
    for (auto bit : aspsExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

auto aspsById(const std::vector<AtlasSequenceParameterSetRBSP> &aspsV, int id) noexcept
    -> const AtlasSequenceParameterSetRBSP & {
  for (auto &x : aspsV) {
    if (id == x.asps_atlas_sequence_parameter_set_id()) {
      return x;
    }
  }
  V3CBITSTREAM_ERROR("Unknown ASPS ID");
}
} // namespace TMIV::MivBitstream
