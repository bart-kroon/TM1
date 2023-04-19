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

#ifndef TMIV_MIVBITSTREAM_VUIPARAMETERS_H
#define TMIV_MIVBITSTREAM_VUIPARAMETERS_H

#include <TMIV/Common/Bitstream.h>

#include <array>
#include <optional>

namespace TMIV::MivBitstream {
// 23090-5: max_coded_video_resolution( )
class MaxCodedVideoResolution {
public:
  [[nodiscard]] auto mcv_occupancy_resolution_present_flag() const -> bool;
  [[nodiscard]] auto mcv_geometry_resolution_present_flag() const -> bool;
  [[nodiscard]] auto mcv_attribute_resolution_present_flag() const -> bool;
  [[nodiscard]] auto mcv_occupancy_width() const -> int32_t;
  [[nodiscard]] auto mcv_occupancy_height() const -> int32_t;
  [[nodiscard]] auto mcv_geometry_width() const -> int32_t;
  [[nodiscard]] auto mcv_geometry_height() const -> int32_t;
  [[nodiscard]] auto mcv_attribute_width() const -> int32_t;
  [[nodiscard]] auto mcv_attribute_height() const -> int32_t;

  auto mcv_occupancy_resolution_present_flag(bool value) -> MaxCodedVideoResolution &;
  auto mcv_geometry_resolution_present_flag(bool value) -> MaxCodedVideoResolution &;
  auto mcv_attribute_resolution_present_flag(bool value) -> MaxCodedVideoResolution &;
  auto mcv_occupancy_width(int32_t value) -> MaxCodedVideoResolution &;
  auto mcv_occupancy_height(int32_t value) -> MaxCodedVideoResolution &;
  auto mcv_geometry_width(int32_t value) -> MaxCodedVideoResolution &;
  auto mcv_geometry_height(int32_t value) -> MaxCodedVideoResolution &;
  auto mcv_attribute_width(int32_t value) -> MaxCodedVideoResolution &;
  auto mcv_attribute_height(int32_t value) -> MaxCodedVideoResolution &;

  friend auto operator<<(std::ostream &stream, const MaxCodedVideoResolution &x) -> std::ostream &;

  auto operator==(const MaxCodedVideoResolution &other) const -> bool;
  auto operator!=(const MaxCodedVideoResolution &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MaxCodedVideoResolution;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_mcv_occupancy_resolution_present_flag{};
  bool m_mcv_geometry_resolution_present_flag{};
  bool m_mcv_attribute_resolution_present_flag{};
  std::optional<int32_t> m_mcv_occupancy_width;
  std::optional<int32_t> m_mcv_occupancy_height;
  std::optional<int32_t> m_mcv_geometry_width;
  std::optional<int32_t> m_mcv_geometry_height;
  std::optional<int32_t> m_mcv_attribute_width;
  std::optional<int32_t> m_mcv_attribute_height;
};

// 23090-5: coordinate_system_parameters()
class CoordinateSystemParameters {
public:
  [[nodiscard]] constexpr auto csp_forward_axis() const noexcept;
  [[nodiscard]] constexpr auto csp_delta_left_axis_minus1() const noexcept;
  [[nodiscard]] constexpr auto csp_forward_sign() const noexcept;
  [[nodiscard]] constexpr auto csp_left_sign() const noexcept;
  [[nodiscard]] constexpr auto csp_up_sign() const noexcept;

  constexpr auto csp_forward_axis(uint8_t value) noexcept -> auto &;
  constexpr auto csp_delta_left_axis_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto csp_forward_sign(bool value) noexcept -> auto &;
  constexpr auto csp_left_sign(bool value) noexcept -> auto &;
  constexpr auto csp_up_sign(bool value) noexcept -> auto &;

  [[nodiscard]] constexpr auto isOmafCas() const noexcept;

  friend auto operator<<(std::ostream &stream, const CoordinateSystemParameters &x)
      -> std::ostream &;

  constexpr auto operator==(const CoordinateSystemParameters &other) const noexcept;
  constexpr auto operator!=(const CoordinateSystemParameters &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CoordinateSystemParameters;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint8_t m_csp_forward_axis{};
  uint8_t m_csp_delta_left_axis_minus1{};
  bool m_csp_forward_sign{true};
  bool m_csp_left_sign{true};
  bool m_csp_up_sign{true};
};

class AtlasSequenceParameterSetRBSP;

// 23090-5: vui_parameters()
//
// Limitations:
//  * vui_hrd_parameters_present_flag = 0
class VuiParameters {
public:
  [[nodiscard]] constexpr auto vui_timing_info_present_flag() const noexcept;
  [[nodiscard]] auto vui_num_units_in_tick() const -> uint32_t;
  [[nodiscard]] auto vui_time_scale() const -> uint32_t;
  [[nodiscard]] auto vui_poc_proportional_to_timing_flag() const -> bool;
  [[nodiscard]] auto vui_num_ticks_poc_diff_one_minus1() const -> uint32_t;
  [[nodiscard]] auto vui_hrd_parameters_present_flag() const -> bool;

  [[nodiscard]] constexpr auto vui_tiles_restriction_present_flag() const noexcept;
  [[nodiscard]] auto vui_fixed_atlas_tile_structure_flag() const -> bool;
  [[nodiscard]] auto vui_fixed_video_tile_structure_flag() const -> bool;
  [[nodiscard]] auto vui_constrained_tiles_across_v3c_components_idc() const -> uint8_t;
  [[nodiscard]] auto vui_max_num_tiles_per_atlas_minus1() const -> uint32_t;

  [[nodiscard]] constexpr auto vui_max_coded_video_resolution_present_flag() const noexcept;
  [[nodiscard]] auto max_coded_video_resolution() const -> const MaxCodedVideoResolution &;

  [[nodiscard]] constexpr auto vui_coordinate_system_parameters_present_flag() const noexcept;
  [[nodiscard]] auto coordinate_system_parameters() const -> const CoordinateSystemParameters &;

  [[nodiscard]] constexpr auto vui_unit_in_metres_flag() const noexcept;

  [[nodiscard]] constexpr auto vui_display_box_info_present_flag() const noexcept;
  [[nodiscard]] auto vui_display_box_origin(int32_t d) const -> uint32_t;
  [[nodiscard]] auto vui_display_box_size(int32_t d) const -> uint32_t;

  [[nodiscard]] constexpr auto vui_anchor_point_present_flag() const noexcept;
  [[nodiscard]] auto vui_anchor_point(int32_t d) const -> uint32_t;

  constexpr auto vui_timing_info_present_flag(bool value) noexcept -> auto &;
  auto vui_num_units_in_tick(uint32_t value) noexcept -> VuiParameters &;
  auto vui_time_scale(uint32_t value) noexcept -> VuiParameters &;
  auto vui_poc_proportional_to_timing_flag(bool value) noexcept -> VuiParameters &;
  auto vui_num_ticks_poc_diff_one_minus1(uint32_t value) noexcept -> VuiParameters &;
  auto vui_hrd_parameters_present_flag(bool value) noexcept -> VuiParameters &;

  constexpr auto vui_tiles_restriction_present_flag(bool value) noexcept -> auto &;
  auto vui_fixed_atlas_tile_structure_flag(bool value) noexcept -> VuiParameters &;
  auto vui_fixed_video_tile_structure_flag(bool value) noexcept -> VuiParameters &;
  auto vui_constrained_tiles_across_v3c_components_idc(uint8_t value) noexcept -> VuiParameters &;
  auto vui_max_num_tiles_per_atlas_minus1(uint32_t value) noexcept -> VuiParameters &;

  constexpr auto vui_max_coded_video_resolution_present_flag(bool value) noexcept -> auto &;
  [[nodiscard]] auto max_coded_video_resolution() noexcept -> MaxCodedVideoResolution &;

  constexpr auto vui_coordinate_system_parameters_present_flag(bool value) noexcept -> auto &;
  [[nodiscard]] auto coordinate_system_parameters() noexcept -> CoordinateSystemParameters &;

  constexpr auto vui_unit_in_metres_flag(bool value) noexcept -> auto &;

  constexpr auto vui_display_box_info_present_flag(bool value) noexcept -> auto &;
  auto vui_display_box_origin(int32_t d, uint32_t value) noexcept -> VuiParameters &;
  auto vui_display_box_size(int32_t d, uint32_t value) noexcept -> VuiParameters &;

  constexpr auto vui_anchor_point_present_flag(bool value) noexcept -> auto &;
  auto vui_anchor_point(int32_t d, uint32_t value) noexcept -> VuiParameters &;

  friend auto operator<<(std::ostream &stream, const VuiParameters &x) -> std::ostream &;

  auto operator==(const VuiParameters &other) const -> bool;
  auto operator!=(const VuiParameters &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const AtlasSequenceParameterSetRBSP *asps) -> VuiParameters;

  void encodeTo(Common::OutputBitstream &bitstream,
                const AtlasSequenceParameterSetRBSP *asps) const;

private:
  bool m_vui_timing_info_present_flag{};
  std::optional<uint32_t> m_vui_num_units_in_tick;
  std::optional<uint32_t> m_vui_time_scale;
  std::optional<bool> m_vui_poc_proportional_to_timing_flag;
  std::optional<uint32_t> m_vui_num_ticks_poc_diff_one_minus1;
  std::optional<bool> m_vui_hrd_parameters_present_flag;

  bool m_vui_tiles_restriction_present_flag{};
  std::optional<bool> m_vui_fixed_atlas_tile_structure_flag;
  std::optional<bool> m_vui_fixed_video_tile_structure_flag;
  std::optional<uint8_t> m_vui_constrained_tiles_across_v3c_components_idc;
  std::optional<uint32_t> m_vui_max_num_tiles_per_atlas_minus1;

  bool m_vui_max_coded_video_resolution_present_flag{};
  std::optional<MaxCodedVideoResolution> m_max_coded_video_resolution;

  bool m_vui_coordinate_system_parameters_present_flag{};
  std::optional<CoordinateSystemParameters> m_coordinate_system_parameters;

  bool m_vui_unit_in_metres_flag{};

  bool m_vui_display_box_info_present_flag{};
  std::optional<std::array<uint32_t, 3>> m_vui_display_box_origin;
  std::optional<std::array<uint32_t, 3>> m_vui_display_box_size;

  bool m_vui_anchor_point_present_flag{};
  std::optional<std::array<uint32_t, 3>> m_vui_anchor_point;
};
} // namespace TMIV::MivBitstream

#include "VuiParameters.hpp"

#endif
