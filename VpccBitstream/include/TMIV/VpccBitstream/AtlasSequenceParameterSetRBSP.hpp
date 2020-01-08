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

#ifndef _TMIV_VPCCBITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::VpccBitstream {
constexpr auto RefListStruct::num_ref_entries() const noexcept {
  // NOTE(BK): Only implementing intra coding of atlases (for now)
  return 0;
}

constexpr auto RefListStruct::operator==(const RefListStruct &other) const noexcept {
  return num_ref_entries() == other.num_ref_entries();
}

constexpr auto RefListStruct::operator!=(const RefListStruct &other) const noexcept {
  return !operator==(other);
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_atlas_sequence_parameter_set_id() const
    noexcept {
  return m_asps_atlas_sequence_parameter_set_id;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_frame_width() const noexcept {
  return m_asps_frame_width;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_frame_height() const noexcept {
  return m_asps_frame_height;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_log2_patch_packing_block_size() const noexcept {
  return m_asps_log2_patch_packing_block_size;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_log2_max_atlas_frame_order_cnt_lsb() const
    noexcept {
  return m_asps_log2_max_atlas_frame_order_cnt_lsb;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_max_dec_atlas_frame_buffering() const noexcept {
  return m_asps_max_dec_atlas_frame_buffering;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_long_term_ref_atlas_frames_flag() const
    noexcept {
  return m_asps_long_term_ref_atlas_frames_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_use_eight_orientations_flag() const noexcept {
  return m_asps_use_eight_orientations_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_45degree_projection_patch_present_flag() const
    noexcept {
  return m_asps_45degree_projection_patch_present_flag;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_normal_axis_limits_quantization_enabled_flag() const noexcept {
  return m_asps_normal_axis_limits_quantization_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_normal_axis_max_delta_value_enabled_flag() const
    noexcept {
  return m_asps_normal_axis_max_delta_value_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_remove_duplicate_point_enabled_flag() const
    noexcept {
  return m_asps_remove_duplicate_point_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_pixel_deinterleaving_flag() const noexcept {
  return m_asps_pixel_deinterleaving_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_patch_precedence_order_flag() const noexcept {
  return m_asps_patch_precedence_order_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_patch_size_quantizer_present_flag() const
    noexcept {
  return m_asps_patch_size_quantizer_present_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_enhanced_occupancy_map_for_depth_flag() const
    noexcept {
  return m_asps_enhanced_occupancy_map_for_depth_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_point_local_reconstruction_enabled_flag() const
    noexcept {
  return m_asps_point_local_reconstruction_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_map_count() const noexcept {
  return m_asps_map_count;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_vui_parameters_present_flag() const noexcept {
  return m_asps_vui_parameters_present_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_extension_present_flag() const noexcept {
  return m_asps_extension_present_flag;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_atlas_sequence_parameter_set_id(
    const std::uint8_t value) noexcept {
  m_asps_atlas_sequence_parameter_set_id = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_frame_width(const std::uint16_t value) noexcept {
  m_asps_frame_width = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_frame_height(const std::uint16_t value) noexcept {
  m_asps_frame_height = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_log2_patch_packing_block_size(
    const std::uint8_t value) noexcept {
  m_asps_log2_patch_packing_block_size = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_log2_max_atlas_frame_order_cnt_lsb(
    const std::uint8_t value) noexcept {
  m_asps_log2_max_atlas_frame_order_cnt_lsb = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_max_dec_atlas_frame_buffering(
    const std::uint8_t value) noexcept {
  m_asps_max_dec_atlas_frame_buffering = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_long_term_ref_atlas_frames_flag(const bool value) noexcept {
  m_asps_long_term_ref_atlas_frames_flag = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_use_eight_orientations_flag(const bool value) noexcept {
  m_asps_use_eight_orientations_flag = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_45degree_projection_patch_present_flag(
    const bool value) noexcept {
  m_asps_45degree_projection_patch_present_flag = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_normal_axis_limits_quantization_enabled_flag(
    const bool value) noexcept {
  m_asps_normal_axis_limits_quantization_enabled_flag = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_normal_axis_max_delta_value_enabled_flag(
    const bool value) noexcept {
  m_asps_normal_axis_max_delta_value_enabled_flag = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_remove_duplicate_point_enabled_flag(const bool value) noexcept {
  m_asps_remove_duplicate_point_enabled_flag = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_pixel_deinterleaving_flag(const bool value) noexcept {
  m_asps_pixel_deinterleaving_flag = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_patch_precedence_order_flag(const bool value) noexcept {
  m_asps_patch_precedence_order_flag = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_patch_size_quantizer_present_flag(const bool value) noexcept {
  m_asps_patch_size_quantizer_present_flag = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_enhanced_occupancy_map_for_depth_flag(
    const bool value) noexcept {
  m_asps_enhanced_occupancy_map_for_depth_flag = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_point_local_reconstruction_enabled_flag(
    const bool value) noexcept {
  m_asps_point_local_reconstruction_enabled_flag = value;
  return *this;
}

constexpr auto &AtlasSequenceParameterSetRBSP::asps_map_count(const std::uint8_t value) noexcept {
  m_asps_map_count = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_vui_parameters_present_flag(const bool value) noexcept {
  m_asps_vui_parameters_present_flag = value;
  return *this;
}

constexpr auto &
AtlasSequenceParameterSetRBSP::asps_extension_present_flag(const bool value) noexcept {
  m_asps_extension_present_flag = value;
  return *this;
}
} // namespace TMIV::VpccBitstream
