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

#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <ostream>
#include <utility>

namespace TMIV::MivBitstream {
RefListStruct::RefListStruct(std::vector<int16_t> deltaAfocSt)
    : m_deltaAfocSt{std::move(deltaAfocSt)} {}

auto RefListStruct::num_ref_entries() const noexcept -> size_t { return m_deltaAfocSt.size(); }

auto RefListStruct::deltaAfocSt(size_t i) const -> int16_t {
  VERIFY_V3CBITSTREAM(i < num_ref_entries());
  return m_deltaAfocSt[i];
}

auto RefListStruct::printTo(std::ostream &stream, uint8_t rlsIdx) const -> std::ostream & {
  stream << "num_ref_entries( " << int32_t{rlsIdx} << " )=" << num_ref_entries() << '\n';
  for (size_t i = 0; i < num_ref_entries(); ++i) {
    stream << "DeltaAfocSt( " << int32_t{rlsIdx} << ", " << i << " )=" << deltaAfocSt(i) << '\n';
  }
  return stream;
}

auto RefListStruct::operator==(const RefListStruct &other) const noexcept -> bool {
  return m_deltaAfocSt == other.m_deltaAfocSt;
}

auto RefListStruct::operator!=(const RefListStruct &other) const noexcept -> bool {
  return !operator==(other);
}

auto RefListStruct::decodeFrom(Common::InputBitstream &bitstream,
                               const AtlasSequenceParameterSetRBSP &asps) -> RefListStruct {
  VERIFY_MIVBITSTREAM(!asps.asps_long_term_ref_atlas_frames_flag());
  auto deltaAfocSt = std::vector<int16_t>(bitstream.getUExpGolomb<size_t>(), 0);

  for (auto &x : deltaAfocSt) {
    const auto abs_delta_afoc_st = bitstream.getUExpGolomb<int16_t>();
    VERIFY_V3CBITSTREAM(0 <= abs_delta_afoc_st && abs_delta_afoc_st <= INT16_MAX);

    if (abs_delta_afoc_st != 0) {
      const auto strpf_entry_sign_flag = bitstream.getFlag();
      x = Common::verifyDownCast<int16_t>(strpf_entry_sign_flag ? abs_delta_afoc_st
                                                                : -abs_delta_afoc_st);
    }
  }

  return RefListStruct{deltaAfocSt};
}

void RefListStruct::encodeTo(Common::OutputBitstream &bitstream,
                             const AtlasSequenceParameterSetRBSP &asps) const {
  PRECONDITION(!asps.asps_long_term_ref_atlas_frames_flag());
  bitstream.putUExpGolomb(num_ref_entries());

  for (auto x : m_deltaAfocSt) {
    bitstream.putUExpGolomb(std::abs(x));
    if (x != 0) {
      bitstream.putFlag(x > 0);
    }
  }
}

auto operator<<(std::ostream &stream, const AspsVpccExtension &x) -> std::ostream & {
  stream << "asps_vpcc_remove_duplicate_point_enabled_flag=" << std::boolalpha
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
  PRECONDITION(!asps.asps_plr_enabled_flag());
}

auto AspsMivExtension::asme_geometry_scale_factor_x_minus1() const -> uint16_t {
  return m_asme_geometry_scale_factor_x_minus1.value_or(0);
}

auto AspsMivExtension::asme_geometry_scale_factor_y_minus1() const -> uint16_t {
  return m_asme_geometry_scale_factor_y_minus1.value_or(0);
}

auto AspsMivExtension::asme_occupancy_scale_factor_x_minus1() const -> uint16_t {
  return m_asme_occupancy_scale_factor_x_minus1.value_or(0);
}

auto AspsMivExtension::asme_occupancy_scale_factor_y_minus1() const -> uint16_t {
  return m_asme_occupancy_scale_factor_y_minus1.value_or(0);
}

auto AspsMivExtension::asme_patch_texture_offset_bit_depth_minus1() const -> uint16_t {
  VERIFY_MIVBITSTREAM(m_asme_patch_texture_offset_bit_depth_minus1.has_value());
  return *m_asme_patch_texture_offset_bit_depth_minus1;
}

auto operator<<(std::ostream &stream, const AspsMivExtension &x) -> std::ostream & {
  TMIV_FMT::print(stream, "asme_ancillary_atlas_flag={}\n", x.asme_ancillary_atlas_flag());
  TMIV_FMT::print(stream, "asme_embedded_occupancy_enabled_flag={}\n",
                  x.asme_embedded_occupancy_enabled_flag());

  if (x.asme_embedded_occupancy_enabled_flag()) {
    TMIV_FMT::print(stream, "asme_depth_occ_map_threshold_flag={}\n",
                    x.asme_depth_occ_threshold_flag());
  }
  TMIV_FMT::print(stream, "asme_geometry_scale_enabled_flag={}\n",
                  x.asme_geometry_scale_enabled_flag());

  if (x.asme_geometry_scale_enabled_flag()) {
    TMIV_FMT::print(stream, "asme_geometry_scale_factor_x_minus1={}\n",
                    x.asme_geometry_scale_factor_x_minus1());
    TMIV_FMT::print(stream, "asme_geometry_scale_factor_y_minus1={}\n",
                    x.asme_geometry_scale_factor_y_minus1());
  }
  if (!x.asme_embedded_occupancy_enabled_flag()) {
    TMIV_FMT::print(stream, "asme_occupancy_scale_enabled_flag={}\n",
                    x.asme_occupancy_scale_enabled_flag());
  }
  if (!x.asme_embedded_occupancy_enabled_flag() && x.asme_occupancy_scale_enabled_flag()) {
    TMIV_FMT::print(stream, "asme_occupancy_scale_factor_x_minus1={}\n",
                    x.asme_occupancy_scale_factor_x_minus1());
    TMIV_FMT::print(stream, "asme_occupancy_scale_factor_y_minus1={}\n",
                    x.asme_occupancy_scale_factor_y_minus1());
  }
  TMIV_FMT::print(stream, "asme_patch_constant_depth_flag={}\n",
                  x.asme_patch_constant_depth_flag());
  TMIV_FMT::print(stream, "asme_patch_texture_offset_enabled_flag={}\n",
                  x.asme_patch_texture_offset_enabled_flag());

  if (x.asme_patch_texture_offset_enabled_flag()) {
    TMIV_FMT::print(stream, "asme_patch_texture_offset_bit_depth_minus1={}\n",
                    x.asme_patch_texture_offset_bit_depth_minus1());
  }
  TMIV_FMT::print(stream, "asme_max_entity_id={}\n", x.asme_max_entity_id());
  TMIV_FMT::print(stream, "asme_inpaint_enabled_flag={}\n", x.asme_inpaint_enabled_flag());
  return stream;
}

auto AspsMivExtension::decodeFrom(Common::InputBitstream &bitstream) -> AspsMivExtension {
  auto x = AspsMivExtension{};
  x.asme_ancillary_atlas_flag(bitstream.getFlag());
  x.asme_embedded_occupancy_enabled_flag(bitstream.getFlag());
  if (x.asme_embedded_occupancy_enabled_flag()) {
    x.asme_depth_occ_threshold_flag(bitstream.getFlag());
  }
  x.asme_geometry_scale_enabled_flag(bitstream.getFlag());
  if (x.asme_geometry_scale_enabled_flag()) {
    x.asme_geometry_scale_factor_x_minus1(bitstream.getUExpGolomb<uint16_t>());
    x.asme_geometry_scale_factor_y_minus1(bitstream.getUExpGolomb<uint16_t>());
  }
  if (!x.asme_embedded_occupancy_enabled_flag()) {
    x.asme_occupancy_scale_enabled_flag(bitstream.getFlag());
  }
  if (!x.asme_embedded_occupancy_enabled_flag() && x.asme_occupancy_scale_enabled_flag()) {
    x.asme_occupancy_scale_factor_x_minus1(bitstream.getUExpGolomb<uint16_t>());
    x.asme_occupancy_scale_factor_y_minus1(bitstream.getUExpGolomb<uint16_t>());
  }
  x.asme_patch_constant_depth_flag(bitstream.getFlag());
  x.asme_patch_texture_offset_enabled_flag(bitstream.getFlag());
  if (x.asme_patch_texture_offset_enabled_flag()) {
    x.asme_patch_texture_offset_bit_depth_minus1(bitstream.getUExpGolomb<uint16_t>());
  }
  x.asme_max_entity_id(bitstream.getUExpGolomb<uint16_t>());
  x.asme_inpaint_enabled_flag(bitstream.getFlag());
  return x;
}

void AspsMivExtension::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(asme_ancillary_atlas_flag());
  bitstream.putFlag(asme_embedded_occupancy_enabled_flag());
  if (asme_embedded_occupancy_enabled_flag()) {
    bitstream.putFlag(asme_depth_occ_threshold_flag());
  }
  bitstream.putFlag(asme_geometry_scale_enabled_flag());
  if (asme_geometry_scale_enabled_flag()) {
    bitstream.putUExpGolomb(asme_geometry_scale_factor_x_minus1());
    bitstream.putUExpGolomb(asme_geometry_scale_factor_y_minus1());
  }
  if (!asme_embedded_occupancy_enabled_flag()) {
    bitstream.putFlag(asme_occupancy_scale_enabled_flag());
  }
  if (!asme_embedded_occupancy_enabled_flag() && asme_occupancy_scale_enabled_flag()) {
    bitstream.putUExpGolomb(asme_occupancy_scale_factor_x_minus1());
    bitstream.putUExpGolomb(asme_occupancy_scale_factor_y_minus1());
  }
  bitstream.putFlag(asme_patch_constant_depth_flag());
  bitstream.putFlag(asme_patch_texture_offset_enabled_flag());
  if (asme_patch_texture_offset_enabled_flag()) {
    bitstream.putUExpGolomb(asme_patch_texture_offset_bit_depth_minus1());
  }
  bitstream.putUExpGolomb(asme_max_entity_id());
  bitstream.putFlag(asme_inpaint_enabled_flag());
}

auto operator<<(std::ostream &stream, const AspsMiv2Extension &x) -> std::ostream & {
  TMIV_FMT::print(stream, "asme_patch_margin_enabled_flag={}\n",
                  x.asme_patch_margin_enabled_flag());
  return stream;
}

auto AspsMiv2Extension::operator==(const AspsMiv2Extension &other) const noexcept -> bool {
  return asme_patch_margin_enabled_flag() == other.asme_patch_margin_enabled_flag();
}

auto AspsMiv2Extension::operator!=(const AspsMiv2Extension &other) const noexcept -> bool {
  return !operator==(other);
}

auto AspsMiv2Extension::decodeFrom(Common::InputBitstream &bitstream) -> AspsMiv2Extension {
  auto x = AspsMiv2Extension{};

  x.asme_patch_margin_enabled_flag(bitstream.getFlag());

  const auto asme_reserved_zero_8bits = bitstream.getUint8();
  VERIFY_MIVBITSTREAM(asme_reserved_zero_8bits == 0);

  return x;
}

void AspsMiv2Extension::encodeTo(Common::OutputBitstream &bitstream) const {
  static constexpr auto asme_reserved_zero_8bits = 0;

  bitstream.putFlag(asme_patch_margin_enabled_flag());
  bitstream.putUint8(asme_reserved_zero_8bits);
}

auto AtlasSequenceParameterSetRBSP::asps_num_ref_atlas_frame_lists_in_asps() const noexcept
    -> uint8_t {
  return static_cast<uint8_t>(m_ref_list_structs.size());
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx) const -> const RefListStruct & {
  VERIFY_V3CBITSTREAM(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  return m_ref_list_structs[rlsIdx];
}

auto AtlasSequenceParameterSetRBSP::vui_parameters() const -> const VuiParameters & {
  VERIFY_V3CBITSTREAM(asps_vui_parameters_present_flag());
  VERIFY_V3CBITSTREAM(m_vui.has_value());
  return *m_vui;
}

auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension() const -> const AspsVpccExtension & {
  VERIFY_V3CBITSTREAM(asps_vpcc_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_asve.has_value());
  return *m_asve;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_extension() const -> const AspsMivExtension & {
  VERIFY_MIVBITSTREAM(asps_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_asme.has_value());
  return *m_asme;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_2_extension() const -> const AspsMiv2Extension & {
  VERIFY_V3CBITSTREAM(asps_miv_2_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_asme2.has_value());
  return *m_asme2;
}

auto AtlasSequenceParameterSetRBSP::aspsExtensionData() const -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(asps_extension_5bits());
  VERIFY_V3CBITSTREAM(m_aspsExtensionData.has_value());
  return *m_aspsExtensionData;
}

auto AtlasSequenceParameterSetRBSP::asps_log2_max_atlas_frame_order_cnt_lsb_minus4(
    const uint8_t value) noexcept -> AtlasSequenceParameterSetRBSP & {
  PRECONDITION(value <= 12);
  m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4 = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_num_ref_atlas_frame_lists_in_asps(const size_t value)
    -> AtlasSequenceParameterSetRBSP & {
  m_ref_list_structs.resize(value);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension_present_flag(const bool value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  asps_extension_present_flag(true);
  m_asps_vpcc_extension_present_flag = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_extension_present_flag(const bool value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  asps_extension_present_flag(true);
  m_asps_miv_extension_present_flag = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_2_extension_present_flag(const bool value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  asps_extension_present_flag(true);
  m_asps_miv_2_extension_present_flag = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::asps_extension_5bits(const uint8_t value) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  asps_extension_present_flag(true);
  PRECONDITION(value < 0x40);
  m_asps_extension_5bits = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::aspsExtensionData(std::vector<bool> data) noexcept
    -> AtlasSequenceParameterSetRBSP & {
  PRECONDITION(asps_extension_5bits());
  m_aspsExtensionData = std::move(data);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx, RefListStruct value)
    -> AtlasSequenceParameterSetRBSP & {
  PRECONDITION(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  m_ref_list_structs[rlsIdx] = std::move(value);
  return *this;
}

auto AtlasSequenceParameterSetRBSP::ref_list_struct(uint8_t rlsIdx) -> RefListStruct & {
  PRECONDITION(rlsIdx < asps_num_ref_atlas_frame_lists_in_asps());
  return m_ref_list_structs[rlsIdx];
}

auto AtlasSequenceParameterSetRBSP::asps_max_number_projections_minus1(
    const uint32_t value) noexcept -> AtlasSequenceParameterSetRBSP & {
  asps_extended_projection_enabled_flag(true);
  m_asps_max_number_projections_minus1 = value;
  return *this;
}

auto AtlasSequenceParameterSetRBSP::vui_parameters() noexcept -> VuiParameters & {
  asps_vui_parameters_present_flag(true);
  if (!m_vui) {
    m_vui = VuiParameters{};
  }
  return *m_vui;
}

auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension() noexcept -> AspsVpccExtension & {
  asps_vpcc_extension_present_flag(true);
  if (!m_asve) {
    m_asve = AspsVpccExtension{};
  }
  return *m_asve;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_extension() noexcept -> AspsMivExtension & {
  asps_miv_extension_present_flag(true);
  if (!m_asme) {
    m_asme = AspsMivExtension{};
  }
  return *m_asme;
}

auto AtlasSequenceParameterSetRBSP::asps_miv_2_extension() noexcept -> AspsMiv2Extension & {
  asps_miv_2_extension_present_flag(true);
  if (!m_asme2) {
    m_asme2 = AspsMiv2Extension{};
  }
  return *m_asme2;
}

auto operator<<(std::ostream &stream, const AtlasSequenceParameterSetRBSP &x) -> std::ostream & {
  stream << "asps_atlas_sequence_parameter_set_id="
         << int32_t{x.asps_atlas_sequence_parameter_set_id()} << '\n';
  stream << "asps_frame_width=" << x.asps_frame_width() << '\n';
  stream << "asps_frame_height=" << x.asps_frame_height() << '\n';
  stream << "asps_geometry_3d_bit_depth_minus1=" << int32_t{x.asps_geometry_3d_bit_depth_minus1()}
         << '\n';
  stream << "asps_geometry_2d_bit_depth_minus1=" << int32_t{x.asps_geometry_2d_bit_depth_minus1()}
         << '\n';
  stream << "asps_log2_max_atlas_frame_order_cnt_lsb_minus4="
         << int32_t{x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4()} << '\n';
  stream << "asps_max_dec_atlas_frame_buffering_minus1="
         << x.asps_max_dec_atlas_frame_buffering_minus1() << '\n';
  stream << "asps_long_term_ref_atlas_frames_flag=" << std::boolalpha
         << x.asps_long_term_ref_atlas_frames_flag() << '\n';
  stream << "asps_num_ref_atlas_frame_lists_in_asps="
         << int32_t{x.asps_num_ref_atlas_frame_lists_in_asps()} << '\n';
  for (uint8_t i = 0; i < x.asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
    x.ref_list_struct(i).printTo(stream, i);
  }
  stream << "asps_use_eight_orientations_flag=" << std::boolalpha
         << x.asps_use_eight_orientations_flag() << '\n';
  stream << "asps_extended_projection_enabled_flag=" << std::boolalpha
         << x.asps_extended_projection_enabled_flag() << '\n';
  if (x.asps_extended_projection_enabled_flag()) {
    stream << "asps_max_number_projections_minus1=" << x.asps_max_number_projections_minus1()
           << '\n';
  }
  stream << "asps_normal_axis_limits_quantization_enabled_flag=" << std::boolalpha
         << x.asps_normal_axis_limits_quantization_enabled_flag() << '\n';
  stream << "asps_normal_axis_max_delta_value_enabled_flag=" << std::boolalpha
         << x.asps_normal_axis_max_delta_value_enabled_flag() << '\n';
  stream << "asps_patch_precedence_order_flag=" << std::boolalpha
         << x.asps_patch_precedence_order_flag() << '\n';
  stream << "asps_log2_patch_packing_block_size=" << int32_t{x.asps_log2_patch_packing_block_size()}
         << '\n';
  stream << "asps_patch_size_quantizer_present_flag=" << std::boolalpha
         << x.asps_patch_size_quantizer_present_flag() << '\n';
  stream << "asps_map_count_minus1=" << int32_t{x.asps_map_count_minus1()} << '\n';
  stream << "asps_pixel_deinterleaving_enabled_flag=" << std::boolalpha
         << x.asps_pixel_deinterleaving_enabled_flag() << '\n';
  stream << "asps_raw_patch_enabled_flag=" << std::boolalpha << x.asps_raw_patch_enabled_flag()
         << '\n';
  stream << "asps_eom_patch_enabled_flag=" << std::boolalpha << x.asps_eom_patch_enabled_flag()
         << '\n';
  stream << "asps_plr_enabled_flag=" << std::boolalpha << x.asps_plr_enabled_flag() << '\n';
  stream << "asps_vui_parameters_present_flag=" << std::boolalpha
         << x.asps_vui_parameters_present_flag() << '\n';

  if (x.asps_vui_parameters_present_flag()) {
    stream << x.vui_parameters();
  }
  stream << "asps_extension_present_flag=" << std::boolalpha << x.asps_extension_present_flag()
         << '\n';

  if (x.asps_extension_present_flag()) {
    stream << "asps_vpcc_extension_present_flag=" << std::boolalpha
           << x.asps_vpcc_extension_present_flag() << '\n';
    stream << "asps_miv_extension_present_flag=" << std::boolalpha
           << x.asps_miv_extension_present_flag() << '\n';
    stream << "asps_miv_2_extension_present_flag=" << std::boolalpha
           << x.asps_miv_2_extension_present_flag() << '\n';
    stream << "asps_extension_5bits=" << int32_t{x.asps_extension_5bits()} << '\n';
  }
  if (x.asps_vpcc_extension_present_flag()) {
    stream << x.asps_vpcc_extension();
  }
  if (x.asps_miv_extension_present_flag()) {
    stream << x.asps_miv_extension();
  }
  if (x.asps_miv_2_extension_present_flag()) {
    stream << x.asps_miv_2_extension();
  }
  if (x.asps_extension_5bits() != 0U) {
    for (bool flag : x.aspsExtensionData()) {
      stream << "asps_extension_data_flag=" << std::boolalpha << flag << '\n';
    }
  }
  return stream;
}

auto AtlasSequenceParameterSetRBSP::operator==(const AtlasSequenceParameterSetRBSP &other) const
    -> bool {
  if (asps_atlas_sequence_parameter_set_id() != other.asps_atlas_sequence_parameter_set_id() ||
      asps_frame_width() != other.asps_frame_width() ||
      asps_frame_height() != other.asps_frame_height() ||
      asps_geometry_3d_bit_depth_minus1() != other.asps_geometry_3d_bit_depth_minus1() ||
      asps_geometry_2d_bit_depth_minus1() != other.asps_geometry_2d_bit_depth_minus1() ||
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
      asps_pixel_deinterleaving_enabled_flag() != other.asps_pixel_deinterleaving_enabled_flag() ||
      asps_raw_patch_enabled_flag() != other.asps_raw_patch_enabled_flag() ||
      asps_eom_patch_enabled_flag() != other.asps_eom_patch_enabled_flag() ||
      asps_plr_enabled_flag() != other.asps_plr_enabled_flag() ||
      asps_vui_parameters_present_flag() != other.asps_vui_parameters_present_flag() ||
      asps_extension_present_flag() != other.asps_extension_present_flag() ||
      asps_vpcc_extension_present_flag() != other.asps_vpcc_extension_present_flag() ||
      asps_miv_extension_present_flag() != other.asps_miv_extension_present_flag() ||
      asps_miv_2_extension_present_flag() != other.asps_miv_2_extension_present_flag() ||
      asps_extension_5bits() != other.asps_extension_5bits()) {
    return false;
  }
  if (asps_vui_parameters_present_flag() && vui_parameters() != other.vui_parameters()) {
    return false;
  }
  if (asps_vpcc_extension_present_flag() && asps_vpcc_extension() != other.asps_vpcc_extension()) {
    return false;
  }
  if (asps_miv_extension_present_flag() && asps_miv_extension() != other.asps_miv_extension()) {
    return false;
  }
  if (asps_miv_2_extension_present_flag() &&
      asps_miv_2_extension() != other.asps_miv_2_extension()) {
    return false;
  }
  if ((asps_extension_5bits() != 0U) && aspsExtensionData() != other.aspsExtensionData()) {
    return false;
  }
  return true;
}

auto AtlasSequenceParameterSetRBSP::operator!=(const AtlasSequenceParameterSetRBSP &other) const
    -> bool {
  return !operator==(other);
}

auto AtlasSequenceParameterSetRBSP::decodeFrom(std::istream &stream)
    -> AtlasSequenceParameterSetRBSP {
  auto x = AtlasSequenceParameterSetRBSP{};
  Common::InputBitstream bitstream{stream};

  x.asps_atlas_sequence_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  x.asps_frame_width(bitstream.getUExpGolomb<uint16_t>());
  x.asps_frame_height(bitstream.getUExpGolomb<uint16_t>());
  x.asps_geometry_3d_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
  x.asps_geometry_2d_bit_depth_minus1(bitstream.readBits<uint8_t>(5));

  x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() <= 12);

  x.asps_max_dec_atlas_frame_buffering_minus1(bitstream.getUExpGolomb<uint8_t>());
  x.asps_long_term_ref_atlas_frames_flag(bitstream.getFlag());

  x.asps_num_ref_atlas_frame_lists_in_asps(bitstream.getUExpGolomb<size_t>());
  VERIFY_V3CBITSTREAM(x.asps_num_ref_atlas_frame_lists_in_asps() <= 64);

  for (uint8_t i = 0; i < x.asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
    x.ref_list_struct(i, RefListStruct::decodeFrom(bitstream, x));
  }

  x.asps_use_eight_orientations_flag(bitstream.getFlag());
  x.asps_extended_projection_enabled_flag(bitstream.getFlag());

  if (x.asps_extended_projection_enabled_flag()) {
    x.asps_max_number_projections_minus1(bitstream.getUExpGolomb<uint32_t>());
  }

  x.asps_normal_axis_limits_quantization_enabled_flag(bitstream.getFlag());
  x.asps_normal_axis_max_delta_value_enabled_flag(bitstream.getFlag());
  x.asps_patch_precedence_order_flag(bitstream.getFlag());

  x.asps_log2_patch_packing_block_size(bitstream.readBits<uint8_t>(3));
  VERIFY_V3CBITSTREAM(x.asps_log2_patch_packing_block_size() <= 7);

  x.asps_patch_size_quantizer_present_flag(bitstream.getFlag());

  x.asps_map_count_minus1(bitstream.readBits<uint8_t>(4));

  x.asps_pixel_deinterleaving_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_pixel_deinterleaving_enabled_flag());

  x.asps_raw_patch_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_raw_patch_enabled_flag());

  x.asps_eom_patch_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_eom_patch_enabled_flag());

  x.asps_plr_enabled_flag(bitstream.getFlag());
  VERIFY_MIVBITSTREAM(!x.asps_plr_enabled_flag());

  x.asps_vui_parameters_present_flag(bitstream.getFlag());

  if (x.asps_vui_parameters_present_flag()) {
    x.vui_parameters() = VuiParameters::decodeFrom(bitstream, &x);
  }
  x.asps_extension_present_flag(bitstream.getFlag());

  if (x.asps_extension_present_flag()) {
    x.asps_vpcc_extension_present_flag(bitstream.getFlag());
    x.asps_miv_extension_present_flag(bitstream.getFlag());
    x.asps_miv_2_extension_present_flag(bitstream.getFlag());
    x.asps_extension_5bits(bitstream.readBits<uint8_t>(5));
  }
  if (x.asps_vpcc_extension_present_flag()) {
    x.asps_vpcc_extension() = AspsVpccExtension::decodeFrom(bitstream, x);
  }
  if (x.asps_miv_extension_present_flag()) {
    x.asps_miv_extension() = AspsMivExtension::decodeFrom(bitstream);
  }
  if (x.asps_miv_2_extension_present_flag()) {
    x.asps_miv_2_extension() = AspsMiv2Extension::decodeFrom(bitstream);
  }
  if (x.asps_extension_5bits() != 0) {
    auto aspsExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      const auto asps_extension_data_flag = bitstream.getFlag();
      aspsExtensionData.push_back(asps_extension_data_flag);
    }
    x.aspsExtensionData(std::move(aspsExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void AtlasSequenceParameterSetRBSP::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(asps_atlas_sequence_parameter_set_id());
  bitstream.putUExpGolomb(asps_frame_width());
  bitstream.putUExpGolomb(asps_frame_height());
  bitstream.writeBits(asps_geometry_3d_bit_depth_minus1(), 5);
  bitstream.writeBits(asps_geometry_2d_bit_depth_minus1(), 5);

  PRECONDITION(asps_log2_max_atlas_frame_order_cnt_lsb_minus4() <= 12);
  bitstream.putUExpGolomb(asps_log2_max_atlas_frame_order_cnt_lsb_minus4());

  bitstream.putUExpGolomb(asps_max_dec_atlas_frame_buffering_minus1());
  bitstream.putFlag(asps_long_term_ref_atlas_frames_flag());

  PRECONDITION(asps_num_ref_atlas_frame_lists_in_asps() <= 64);
  bitstream.putUExpGolomb(asps_num_ref_atlas_frame_lists_in_asps());

  for (uint8_t i = 0; i < asps_num_ref_atlas_frame_lists_in_asps(); ++i) {
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

  PRECONDITION(asps_log2_patch_packing_block_size() <= 7);
  bitstream.writeBits(asps_log2_patch_packing_block_size(), 3);

  bitstream.putFlag(asps_patch_size_quantizer_present_flag());
  bitstream.writeBits(asps_map_count_minus1(), 4);

  PRECONDITION(!asps_pixel_deinterleaving_enabled_flag());
  bitstream.putFlag(asps_pixel_deinterleaving_enabled_flag());

  PRECONDITION(!asps_raw_patch_enabled_flag());
  bitstream.putFlag(asps_raw_patch_enabled_flag());

  PRECONDITION(!asps_eom_patch_enabled_flag());
  bitstream.putFlag(asps_eom_patch_enabled_flag());

  PRECONDITION(!asps_plr_enabled_flag());
  bitstream.putFlag(asps_plr_enabled_flag());

  bitstream.putFlag(asps_vui_parameters_present_flag());

  if (asps_vui_parameters_present_flag()) {
    vui_parameters().encodeTo(bitstream, this);
  }
  bitstream.putFlag(asps_extension_present_flag());

  if (asps_extension_present_flag()) {
    bitstream.putFlag(asps_vpcc_extension_present_flag());
    bitstream.putFlag(asps_miv_extension_present_flag());
    bitstream.putFlag(asps_miv_2_extension_present_flag());
    bitstream.writeBits(asps_extension_5bits(), 5);
  }
  if (asps_vpcc_extension_present_flag()) {
    asps_vpcc_extension().encodeTo(bitstream, *this);
  }
  if (asps_miv_extension_present_flag()) {
    asps_miv_extension().encodeTo(bitstream);
  }
  if (asps_miv_2_extension_present_flag()) {
    asps_miv_2_extension().encodeTo(bitstream);
  }
  if (asps_extension_5bits() != 0) {
    for (auto bit : aspsExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

auto aspsById(const std::vector<AtlasSequenceParameterSetRBSP> &aspsV, int32_t id)
    -> const AtlasSequenceParameterSetRBSP & {
  for (const auto &x : aspsV) {
    if (id == x.asps_atlas_sequence_parameter_set_id()) {
      return x;
    }
  }
  V3CBITSTREAM_ERROR("Unknown ASPS ID");
}
} // namespace TMIV::MivBitstream
