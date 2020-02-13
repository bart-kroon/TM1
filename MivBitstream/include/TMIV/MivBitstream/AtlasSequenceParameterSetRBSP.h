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

#ifndef _TMIV_MIVBITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H_
#define _TMIV_MIVBITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H_

#include <TMIV/Common/Bitstream.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
class AtlasSequenceParameterSetRBSP;

// 23090-5: ref_list_struct( rlsIdx )
class RefListStruct {
public:
  RefListStruct() = default;
  explicit RefListStruct(std::vector<std::int16_t> deltaAfocSt);

  auto num_ref_entries() const noexcept -> std::size_t;
  auto deltaAfocSt(std::size_t i) const noexcept -> std::int16_t;

  auto printTo(std::ostream &stream, std::uint8_t rlsIdx) const -> std::ostream &;

  auto operator==(const RefListStruct &other) const noexcept -> bool;
  auto operator!=(const RefListStruct &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const AtlasSequenceParameterSetRBSP &asps) -> RefListStruct;

  void encodeTo(Common::OutputBitstream &bitstream,
                const AtlasSequenceParameterSetRBSP &asps) const;

private:
  std::vector<std::int16_t> m_deltaAfocSt;
};

// 23090-12: atlas_sequence_parameter_set_rbsp()
class AtlasSequenceParameterSetRBSP {
public:
  constexpr auto asps_atlas_sequence_parameter_set_id() const noexcept;
  constexpr auto asps_frame_width() const noexcept;
  constexpr auto asps_frame_height() const noexcept;
  constexpr auto asps_log2_patch_packing_block_size() const noexcept;
  constexpr auto asps_log2_max_atlas_frame_order_cnt_lsb_minus4() const noexcept;
  constexpr auto asps_max_dec_atlas_frame_buffering_minus1() const noexcept;
  constexpr auto asps_long_term_ref_atlas_frames_flag() const noexcept;
  auto asps_num_ref_atlas_frame_lists_in_asps() const noexcept -> std::uint8_t;
  auto ref_list_struct(std::uint8_t rlsIdx) const -> const RefListStruct &;
  constexpr auto asps_use_eight_orientations_flag() const noexcept;
  constexpr auto asps_extended_projection_enabled_flag() const noexcept;
  auto asps_max_projections_minus1() const noexcept -> unsigned;
  constexpr auto asps_normal_axis_limits_quantization_enabled_flag() const noexcept;
  constexpr auto asps_normal_axis_max_delta_value_enabled_flag() const noexcept;
  constexpr auto asps_remove_duplicate_point_enabled_flag() const noexcept;
  constexpr auto asps_pixel_deinterleaving_flag() const noexcept;
  constexpr auto asps_patch_precedence_order_flag() const noexcept;
  constexpr auto asps_patch_size_quantizer_present_flag() const noexcept;
  constexpr auto asps_raw_patch_enabled_flag() const noexcept;
  constexpr auto asps_eom_patch_enabled_flag() const noexcept;
  constexpr auto asps_point_local_reconstruction_enabled_flag() const noexcept;
  constexpr auto asps_map_count_minus1() const noexcept;
  constexpr auto asps_vui_parameters_present_flag() const noexcept;
  constexpr auto asps_extension_present_flag() const noexcept;

  constexpr auto &asps_atlas_sequence_parameter_set_id(const std::uint8_t value) noexcept;
  constexpr auto &asps_frame_width(const std::uint16_t value) noexcept;
  constexpr auto &asps_frame_height(const std::uint16_t value) noexcept;
  constexpr auto &asps_log2_patch_packing_block_size(const std::uint8_t value) noexcept;
  constexpr auto &asps_log2_max_atlas_frame_order_cnt_lsb_minus4(const std::uint8_t value) noexcept;
  constexpr auto &asps_max_dec_atlas_frame_buffering_minus1(const std::uint8_t value) noexcept;
  constexpr auto &asps_long_term_ref_atlas_frames_flag(const bool value) noexcept;
  auto asps_num_ref_atlas_frame_lists_in_asps(const std::size_t value)
      -> AtlasSequenceParameterSetRBSP &;
  auto ref_list_struct(std::uint8_t rlsIdx, RefListStruct value) -> AtlasSequenceParameterSetRBSP &;
  constexpr auto &asps_use_eight_orientations_flag(const bool value) noexcept;
  constexpr auto &asps_extended_projection_enabled_flag(const bool value) noexcept;
  auto asps_max_projections_minus1(const unsigned value) noexcept
      -> AtlasSequenceParameterSetRBSP &;
  constexpr auto &asps_normal_axis_limits_quantization_enabled_flag(const bool value) noexcept;
  constexpr auto &asps_normal_axis_max_delta_value_enabled_flag(const bool value) noexcept;
  constexpr auto &asps_remove_duplicate_point_enabled_flag(const bool value) noexcept;
  constexpr auto &asps_pixel_deinterleaving_flag(const bool value) noexcept;
  constexpr auto &asps_patch_precedence_order_flag(const bool value) noexcept;
  constexpr auto &asps_patch_size_quantizer_present_flag(const bool value) noexcept;
  constexpr auto &asps_raw_patch_enabled_flag(const bool value) noexcept;
  constexpr auto &asps_eom_patch_enabled_flag(const bool value) noexcept;
  constexpr auto &asps_point_local_reconstruction_enabled_flag(const bool value) noexcept;
  constexpr auto &asps_map_count_minus1(const std::uint8_t value) noexcept;
  constexpr auto &asps_vui_parameters_present_flag(const bool value) noexcept;
  constexpr auto &asps_extension_present_flag(const bool value) noexcept;

  [[nodiscard]] auto ref_list_struct(std::uint8_t rlsIdx) -> RefListStruct &;

  friend auto operator<<(std::ostream &stream, const AtlasSequenceParameterSetRBSP &x)
      -> std::ostream &;

  auto operator==(const AtlasSequenceParameterSetRBSP &other) const noexcept -> bool;
  auto operator!=(const AtlasSequenceParameterSetRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> AtlasSequenceParameterSetRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  std::uint8_t m_asps_atlas_sequence_parameter_set_id{};
  std::uint16_t m_asps_frame_width{};
  std::uint16_t m_asps_frame_height{};
  std::uint8_t m_asps_log2_patch_packing_block_size{};
  std::uint8_t m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4{};
  std::size_t m_asps_max_dec_atlas_frame_buffering_minus1{};
  bool m_asps_long_term_ref_atlas_frames_flag{};
  std::vector<RefListStruct> m_ref_list_structs;
  bool m_asps_use_eight_orientations_flag{};
  bool m_asps_extended_projection_enabled_flag{};
  std::optional<unsigned> m_asps_max_projections_minus1{};
  bool m_asps_normal_axis_limits_quantization_enabled_flag{};
  bool m_asps_normal_axis_max_delta_value_enabled_flag{};
  bool m_asps_remove_duplicate_point_enabled_flag{};
  bool m_asps_pixel_deinterleaving_flag{};
  bool m_asps_patch_precedence_order_flag{};
  bool m_asps_patch_size_quantizer_present_flag{};
  bool m_asps_raw_patch_enabled_flag{};
  bool m_asps_eom_patch_enabled_flag{};
  bool m_asps_point_local_reconstruction_enabled_flag{};
  std::uint8_t m_asps_map_count_minus1{};
  bool m_asps_vui_parameters_present_flag{};
  bool m_asps_extension_present_flag{};
};
} // namespace TMIV::MivBitstream

#include "AtlasSequenceParameterSetRBSP.hpp"

#endif
