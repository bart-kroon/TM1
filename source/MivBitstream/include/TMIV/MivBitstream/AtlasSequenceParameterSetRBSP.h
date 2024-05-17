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

#ifndef TMIV_MIVBITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H
#define TMIV_MIVBITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H

#include "V3cUnit.h"
#include "VuiParameters.h"

#include <TMIV/Common/Bitstream.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
class AtlasSequenceParameterSetRBSP;

// 23090-5: ref_list_struct( rlsIdx )
//
// 2309-12 restrictions:
//   * asps_long_term_ref_atlas_frames_flag == 0
class RefListStruct {
public:
  RefListStruct() = default;
  explicit RefListStruct(std::vector<int16_t> deltaAfocSt);

  [[nodiscard]] auto num_ref_entries() const noexcept -> size_t;
  [[nodiscard]] auto deltaAfocSt(size_t i) const -> int16_t;

  auto printTo(std::ostream &stream, uint8_t rlsIdx) const -> std::ostream &;

  auto operator==(const RefListStruct &other) const noexcept -> bool;
  auto operator!=(const RefListStruct &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const AtlasSequenceParameterSetRBSP &asps) -> RefListStruct;

  void encodeTo(Common::OutputBitstream &bitstream,
                const AtlasSequenceParameterSetRBSP &asps) const;

private:
  std::vector<int16_t> m_deltaAfocSt;
};

// 23090-5: asps_vpcc_extension( )
//
// 2309-12 restrictions:
//   * asps_plr_enabled_flag == 0
class AspsVpccExtension {
public:
  [[nodiscard]] constexpr auto asps_vpcc_remove_duplicate_point_enabled_flag() const noexcept;

  constexpr auto asps_vpcc_remove_duplicate_point_enabled_flag(bool value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const AspsVpccExtension & /*x*/) -> std::ostream &;

  constexpr auto operator==(const AspsVpccExtension &other) const noexcept;
  constexpr auto operator!=(const AspsVpccExtension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const AtlasSequenceParameterSetRBSP &asps) -> AspsVpccExtension;

  void encodeTo(Common::OutputBitstream &bitstream,
                const AtlasSequenceParameterSetRBSP &asps) const;

private:
  bool m_asps_vpcc_remove_duplicate_point_enabled_flag{};
};

class AspsMiv2Extension {
public:
  [[nodiscard]] constexpr auto asme_patch_margin_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_background_atlas_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_static_background_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_colorized_geometry_enabled_flag() const noexcept;
  [[nodiscard]] auto asme_colorized_geometry_bit_depth_minus1() const -> uint8_t;

  constexpr auto asme_patch_margin_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asme_background_atlas_flag(bool value) noexcept -> auto &;
  constexpr auto asme_static_background_flag(bool value) noexcept -> auto &;
  constexpr auto asme_colorized_geometry_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asme_colorized_geometry_bit_depth_minus1(uint8_t value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const AspsMiv2Extension & /*x*/) -> std::ostream &;

  auto operator==(const AspsMiv2Extension &other) const noexcept -> bool;
  auto operator!=(const AspsMiv2Extension &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AspsMiv2Extension;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_asme_patch_margin_enabled_flag{};
  bool m_asme_background_atlas_flag{};
  bool m_asme_static_background_flag{};
  bool m_asme_colorized_geometry_enabled_flag{};
  uint8_t m_asme_colorized_geometry_bit_depth_minus1{};
};

// 23090-12: asps_miv_extension( )
class AspsMivExtension {
public:
  [[nodiscard]] constexpr auto asme_ancillary_atlas_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_embedded_occupancy_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_depth_occ_threshold_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_geometry_scale_enabled_flag() const noexcept;
  [[nodiscard]] auto asme_geometry_scale_factor_x_minus1() const -> uint16_t;
  [[nodiscard]] auto asme_geometry_scale_factor_y_minus1() const -> uint16_t;
  [[nodiscard]] constexpr auto asme_occupancy_scale_enabled_flag() const;
  [[nodiscard]] auto asme_occupancy_scale_factor_x_minus1() const -> uint16_t;
  [[nodiscard]] auto asme_occupancy_scale_factor_y_minus1() const -> uint16_t;
  [[nodiscard]] constexpr auto asme_patch_constant_depth_flag() const noexcept;
  [[nodiscard]] constexpr auto asme_patch_texture_offset_enabled_flag() const noexcept;
  [[nodiscard]] auto asme_patch_texture_offset_bit_depth_minus1() const -> uint16_t;
  [[nodiscard]] constexpr auto asme_max_entity_id() const noexcept;
  [[nodiscard]] constexpr auto asme_inpaint_enabled_flag() const noexcept;

  constexpr auto asme_ancillary_atlas_flag(bool value) noexcept -> auto &;
  constexpr auto asme_embedded_occupancy_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asme_depth_occ_threshold_flag(bool value) noexcept -> auto &;
  constexpr auto asme_geometry_scale_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asme_geometry_scale_factor_x_minus1(uint16_t value) noexcept -> auto &;
  constexpr auto asme_geometry_scale_factor_y_minus1(uint16_t value) noexcept -> auto &;
  constexpr auto asme_occupancy_scale_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asme_occupancy_scale_factor_x_minus1(uint16_t value) -> auto &;
  constexpr auto asme_occupancy_scale_factor_y_minus1(uint16_t value) -> auto &;
  constexpr auto asme_patch_constant_depth_flag(bool value) noexcept -> auto &;
  constexpr auto asme_patch_texture_offset_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asme_patch_texture_offset_bit_depth_minus1(uint16_t value) noexcept -> auto &;
  constexpr auto asme_max_entity_id(uint16_t value) noexcept -> auto &;
  constexpr auto asme_inpaint_enabled_flag(bool value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const AspsMivExtension & /*x*/) -> std::ostream &;

  constexpr auto operator==(const AspsMivExtension &other) const;
  constexpr auto operator!=(const AspsMivExtension &other) const;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AspsMivExtension;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_asme_ancillary_atlas_flag{};
  bool m_asme_embedded_occupancy_enabled_flag{};
  bool m_asme_depth_occ_map_threshold_flag{};
  bool m_asme_geometry_scale_enabled_flag{};
  std::optional<uint16_t> m_asme_geometry_scale_factor_x_minus1;
  std::optional<uint16_t> m_asme_geometry_scale_factor_y_minus1;
  bool m_asme_occupancy_scale_enabled_flag{};
  std::optional<uint16_t> m_asme_occupancy_scale_factor_x_minus1;
  std::optional<uint16_t> m_asme_occupancy_scale_factor_y_minus1;
  bool m_asme_patch_constant_depth_flag{};
  bool m_asme_patch_texture_offset_flag{};
  std::optional<uint16_t> m_asme_patch_texture_offset_bit_depth_minus1;
  uint16_t m_asme_max_entity_id{};
  bool m_asme_inpaint_enabled_flag{};
};

// 23090-5: atlas_sequence_parameter_set_rbsp( )
class AtlasSequenceParameterSetRBSP {
public:
  [[nodiscard]] constexpr auto asps_atlas_sequence_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto asps_frame_width() const noexcept;
  [[nodiscard]] constexpr auto asps_frame_height() const noexcept;
  [[nodiscard]] constexpr auto asps_geometry_3d_bit_depth_minus1() const noexcept;
  [[nodiscard]] constexpr auto asps_geometry_2d_bit_depth_minus1() const noexcept;
  [[nodiscard]] constexpr auto asps_log2_max_atlas_frame_order_cnt_lsb_minus4() const noexcept;
  [[nodiscard]] constexpr auto asps_max_dec_atlas_frame_buffering_minus1() const noexcept;
  [[nodiscard]] constexpr auto asps_long_term_ref_atlas_frames_flag() const noexcept;
  [[nodiscard]] auto asps_num_ref_atlas_frame_lists_in_asps() const noexcept -> uint8_t;
  [[nodiscard]] auto ref_list_struct(uint8_t rlsIdx) const -> const RefListStruct &;
  [[nodiscard]] constexpr auto asps_use_eight_orientations_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_extended_projection_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_max_number_projections_minus1() const noexcept;
  [[nodiscard]] constexpr auto asps_normal_axis_limits_quantization_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_normal_axis_max_delta_value_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_patch_precedence_order_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_log2_patch_packing_block_size() const noexcept;
  [[nodiscard]] constexpr auto asps_patch_size_quantizer_present_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_map_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto asps_pixel_deinterleaving_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_raw_patch_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_eom_patch_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_plr_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_vui_parameters_present_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_vpcc_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_miv_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_miv_2_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto asps_extension_5bits() const noexcept;
  [[nodiscard]] auto vui_parameters() const -> const VuiParameters &;
  [[nodiscard]] auto asps_vpcc_extension() const -> const AspsVpccExtension &;
  [[nodiscard]] auto asps_miv_extension() const -> const AspsMivExtension &;
  [[nodiscard]] auto asps_miv_2_extension() const -> const AspsMiv2Extension &;
  [[nodiscard]] auto aspsExtensionData() const -> const std::vector<bool> &;

  constexpr auto asps_atlas_sequence_parameter_set_id(uint8_t value) -> auto &;
  constexpr auto asps_frame_width(int32_t value) noexcept -> auto &;
  constexpr auto asps_frame_height(int32_t value) noexcept -> auto &;
  constexpr auto asps_geometry_3d_bit_depth_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto asps_geometry_2d_bit_depth_minus1(uint8_t value) noexcept -> auto &;
  auto asps_log2_max_atlas_frame_order_cnt_lsb_minus4(uint8_t value) noexcept
      -> AtlasSequenceParameterSetRBSP &;
  constexpr auto asps_max_dec_atlas_frame_buffering_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto asps_long_term_ref_atlas_frames_flag(bool value) noexcept -> auto &;
  auto asps_num_ref_atlas_frame_lists_in_asps(size_t value) -> AtlasSequenceParameterSetRBSP &;
  auto ref_list_struct(uint8_t rlsIdx, RefListStruct value) -> AtlasSequenceParameterSetRBSP &;
  constexpr auto asps_use_eight_orientations_flag(bool value) noexcept -> auto &;
  constexpr auto asps_extended_projection_enabled_flag(bool value) noexcept -> auto &;
  auto asps_max_number_projections_minus1(uint32_t value) noexcept
      -> AtlasSequenceParameterSetRBSP &;
  constexpr auto asps_normal_axis_limits_quantization_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asps_normal_axis_max_delta_value_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asps_patch_precedence_order_flag(bool value) noexcept -> auto &;
  constexpr auto asps_log2_patch_packing_block_size(uint8_t value) noexcept -> auto &;
  constexpr auto asps_patch_size_quantizer_present_flag(bool value) noexcept -> auto &;
  constexpr auto asps_map_count_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto asps_pixel_deinterleaving_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asps_raw_patch_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asps_eom_patch_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asps_plr_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto asps_vui_parameters_present_flag(bool value) noexcept -> auto &;
  constexpr auto asps_extension_present_flag(bool value) noexcept -> auto &;
  auto asps_vpcc_extension_present_flag(bool value) noexcept -> AtlasSequenceParameterSetRBSP &;
  auto asps_miv_extension_present_flag(bool value) noexcept -> AtlasSequenceParameterSetRBSP &;
  auto asps_miv_2_extension_present_flag(bool value) noexcept -> AtlasSequenceParameterSetRBSP &;
  auto asps_extension_5bits(uint8_t value) noexcept -> AtlasSequenceParameterSetRBSP &;
  auto aspsExtensionData(std::vector<bool> data) noexcept -> AtlasSequenceParameterSetRBSP &;

  [[nodiscard]] auto ref_list_struct(uint8_t rlsIdx) -> RefListStruct &;
  [[nodiscard]] auto vui_parameters() noexcept -> VuiParameters &;
  [[nodiscard]] auto asps_vpcc_extension() noexcept -> AspsVpccExtension &;
  [[nodiscard]] auto asps_miv_extension() noexcept -> AspsMivExtension &;
  [[nodiscard]] auto asps_miv_2_extension() noexcept -> AspsMiv2Extension &;

  friend auto operator<<(std::ostream &stream, const AtlasSequenceParameterSetRBSP &x)
      -> std::ostream &;

  auto operator==(const AtlasSequenceParameterSetRBSP &other) const -> bool;
  auto operator!=(const AtlasSequenceParameterSetRBSP &other) const -> bool;

  static auto decodeFrom(std::istream &stream) -> AtlasSequenceParameterSetRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  uint8_t m_asps_atlas_sequence_parameter_set_id{};
  int32_t m_asps_frame_width{};
  int32_t m_asps_frame_height{};
  uint8_t m_asps_geometry_3d_bit_depth_minus1{};
  uint8_t m_asps_geometry_2d_bit_depth_minus1{};
  uint8_t m_asps_log2_patch_packing_block_size{};
  uint8_t m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4{};
  size_t m_asps_max_dec_atlas_frame_buffering_minus1{};
  bool m_asps_long_term_ref_atlas_frames_flag{};
  std::vector<RefListStruct> m_ref_list_structs;
  bool m_asps_use_eight_orientations_flag{};
  bool m_asps_extended_projection_enabled_flag{};
  std::optional<uint32_t> m_asps_max_number_projections_minus1{};
  bool m_asps_normal_axis_limits_quantization_enabled_flag{};
  bool m_asps_normal_axis_max_delta_value_enabled_flag{};
  bool m_asps_pixel_deinterleaving_enabled_flag{};
  bool m_asps_patch_precedence_order_flag{};
  bool m_asps_patch_size_quantizer_present_flag{};
  bool m_asps_raw_patch_enabled_flag{};
  bool m_asps_eom_patch_enabled_flag{};
  bool m_asps_plr_enabled_flag{};
  uint8_t m_asps_map_count_minus1{};
  bool m_asps_vui_parameters_present_flag{};
  bool m_asps_extension_present_flag{};
  std::optional<bool> m_asps_vpcc_extension_present_flag{};
  std::optional<bool> m_asps_miv_extension_present_flag{};
  std::optional<bool> m_asps_miv_2_extension_present_flag{};
  std::optional<uint8_t> m_asps_extension_5bits{};
  std::optional<VuiParameters> m_vui;
  std::optional<AspsVpccExtension> m_asve;
  std::optional<AspsMivExtension> m_asme;
  std::optional<AspsMiv2Extension> m_asme2;
  std::optional<std::vector<bool>> m_aspsExtensionData;
};

auto aspsById(const std::vector<AtlasSequenceParameterSetRBSP> &aspsV, int32_t id)
    -> const AtlasSequenceParameterSetRBSP &;
} // namespace TMIV::MivBitstream

#include "AtlasSequenceParameterSetRBSP.hpp"

#endif
