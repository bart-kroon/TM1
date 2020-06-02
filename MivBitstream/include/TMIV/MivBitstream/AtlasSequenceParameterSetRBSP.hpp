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

#ifndef _TMIV_MIVBITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto AspsVpccExtension::asps_vpcc_remove_duplicate_point_enabled_flag() const noexcept {
  return m_asps_vpcc_remove_duplicate_point_enabled_flag;
}

constexpr auto
AspsVpccExtension::asps_vpcc_remove_duplicate_point_enabled_flag(const unsigned value) noexcept
    -> auto & {
  m_asps_vpcc_remove_duplicate_point_enabled_flag = value;
  return *this;
}

constexpr auto AspsVpccExtension::operator==(const AspsVpccExtension &other) const noexcept {
  return asps_vpcc_remove_duplicate_point_enabled_flag() ==
         other.asps_vpcc_remove_duplicate_point_enabled_flag();
}

<<<<<<< HEAD
constexpr auto MivAtlasSequenceParams::masp_occupancy_scale_present_flag() const noexcept {
  return m_masp_occupancy_scale_present_flag;
}

constexpr auto MivAtlasSequenceParams::masp_group_id(const unsigned value) noexcept -> auto & {
  m_masp_group_id = value;
  return *this;
=======
constexpr auto AspsVpccExtension::operator!=(const AspsVpccExtension &other) const noexcept {
  return !operator==(other);
>>>>>>> integration
}

constexpr auto AspsMivExtension::asme_group_id() const noexcept { return m_asme_group_id; }

constexpr auto AspsMivExtension::asme_auxiliary_atlas_flag() const noexcept {
  return m_asme_auxiliary_atlas_flag;
}

constexpr auto AspsMivExtension::asme_depth_occ_threshold_flag() const noexcept {
  return m_asme_depth_occ_map_threshold_flag;
}

constexpr auto AspsMivExtension::asme_group_id(const unsigned value) noexcept -> auto & {
  m_asme_group_id = value;
  return *this;
}

constexpr auto AspsMivExtension::asme_auxiliary_atlas_flag(const bool value) noexcept -> auto & {
  m_asme_auxiliary_atlas_flag = value;
  return *this;
}

constexpr auto AspsMivExtension::asme_depth_occ_threshold_flag(const bool value) noexcept
    -> auto & {
  m_asme_depth_occ_map_threshold_flag = value;
  return *this;
}

<<<<<<< HEAD
constexpr auto MivAtlasSequenceParams::masp_occupancy_scale_present_flag(const bool value) noexcept
    -> auto & {
  m_masp_occupancy_scale_present_flag = value;
  return *this;
}

constexpr auto
MivAtlasSequenceParams::masp_occupancy_scale_x_minus1(const uint8_t value) noexcept -> auto & {
  m_masp_occupancy_scale_x_minus1 = value;
  return *this;
}

constexpr auto MivAtlasSequenceParams::masp_occupancy_scale_y_minus1(const uint8_t value) noexcept
    -> auto & {
  m_masp_occupancy_scale_y_minus1 = value;
  return *this;
}

constexpr auto
MivAtlasSequenceParams::masp_geometry_frame_width_minus1(const uint16_t value) noexcept -> auto & {
  m_masp_geometry_frame_width_minus1 = value;
=======
constexpr auto AspsMivExtension::asme_geometry_frame_width_minus1(const uint16_t value) noexcept
    -> auto & {
  m_asme_geometry_frame_width_minus1 = value;
>>>>>>> integration
  return *this;
}

constexpr auto AspsMivExtension::asme_geometry_frame_height_minus1(const uint16_t value) noexcept
    -> auto & {
  m_asme_geometry_frame_height_minus1 = value;
  return *this;
}

<<<<<<< HEAD
constexpr auto
MivAtlasSequenceParams::operator==(const MivAtlasSequenceParams &other) const noexcept {
  return m_masp_omaf_v1_compatible_flag == other.m_masp_omaf_v1_compatible_flag &&
         masp_group_id() == other.masp_group_id() &&
         masp_auxiliary_atlas_flag() == other.masp_auxiliary_atlas_flag() &&
         m_masp_depth_occ_map_threshold_flag == other.m_masp_depth_occ_map_threshold_flag &&
         m_masp_occupancy_scale_present_flag == other.m_masp_occupancy_scale_present_flag &&
         m_masp_occupancy_scale_x_minus1 == other.m_masp_occupancy_scale_x_minus1 &&
         m_masp_occupancy_scale_y_minus1 == other.m_masp_occupancy_scale_y_minus1 &&
         m_masp_geometry_frame_width_minus1 == other.m_masp_geometry_frame_width_minus1 &&
         m_masp_geometry_frame_height_minus1 == other.m_masp_geometry_frame_height_minus1;
=======
constexpr auto AspsMivExtension::operator==(const AspsMivExtension &other) const noexcept {
  return asme_group_id() == other.asme_group_id() &&
         asme_auxiliary_atlas_flag() == other.asme_auxiliary_atlas_flag() &&
         asme_depth_occ_threshold_flag() == other.asme_depth_occ_threshold_flag() &&
         m_asme_geometry_frame_width_minus1 == other.m_asme_geometry_frame_width_minus1 &&
         m_asme_geometry_frame_height_minus1 == other.m_asme_geometry_frame_height_minus1;
>>>>>>> integration
}

constexpr auto AspsMivExtension::operator!=(const AspsMivExtension &other) const noexcept {
  return !operator==(other);
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_atlas_sequence_parameter_set_id() const noexcept {
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

constexpr auto
AtlasSequenceParameterSetRBSP::asps_log2_max_atlas_frame_order_cnt_lsb_minus4() const noexcept {
  return m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_max_dec_atlas_frame_buffering_minus1() const noexcept {
  return m_asps_max_dec_atlas_frame_buffering_minus1;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_long_term_ref_atlas_frames_flag() const noexcept {
  return m_asps_long_term_ref_atlas_frames_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_use_eight_orientations_flag() const noexcept {
  return m_asps_use_eight_orientations_flag;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_extended_projection_enabled_flag() const noexcept {
  return m_asps_extended_projection_enabled_flag;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_normal_axis_limits_quantization_enabled_flag() const noexcept {
  return m_asps_normal_axis_limits_quantization_enabled_flag;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_normal_axis_max_delta_value_enabled_flag() const noexcept {
  return m_asps_normal_axis_max_delta_value_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_pixel_deinterleaving_flag() const noexcept {
  return m_asps_pixel_deinterleaving_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_patch_precedence_order_flag() const noexcept {
  return m_asps_patch_precedence_order_flag;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_patch_size_quantizer_present_flag() const noexcept {
  return m_asps_patch_size_quantizer_present_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_raw_patch_enabled_flag() const noexcept {
  return m_asps_raw_patch_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_eom_patch_enabled_flag() const noexcept {
  return m_asps_eom_patch_enabled_flag;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_point_local_reconstruction_enabled_flag() const noexcept {
  return m_asps_point_local_reconstruction_enabled_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_map_count_minus1() const noexcept {
  return m_asps_map_count_minus1;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_vui_parameters_present_flag() const noexcept {
  return m_asps_vui_parameters_present_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_extension_present_flag() const noexcept {
  return m_asps_extension_present_flag;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_vpcc_extension_flag() const noexcept {
  return m_asps_vpcc_extension_flag.value_or(false);
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_miv_extension_flag() const noexcept {
  return m_asps_miv_extension_flag.value_or(false);
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_extension_6bits() const noexcept {
  return m_asps_extension_6bits.value_or(0);
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_atlas_sequence_parameter_set_id(const uint8_t value) noexcept
    -> auto & {
  m_asps_atlas_sequence_parameter_set_id = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_frame_width(const uint16_t value) noexcept
    -> auto & {
  m_asps_frame_width = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_frame_height(const uint16_t value) noexcept
    -> auto & {
  m_asps_frame_height = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_log2_patch_packing_block_size(const uint8_t value) noexcept
    -> auto & {
  m_asps_log2_patch_packing_block_size = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_log2_max_atlas_frame_order_cnt_lsb_minus4(
    const uint8_t value) noexcept -> auto & {
  m_asps_log2_max_atlas_frame_order_cnt_lsb_minus4 = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_max_dec_atlas_frame_buffering_minus1(
    const uint8_t value) noexcept -> auto & {
  m_asps_max_dec_atlas_frame_buffering_minus1 = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_long_term_ref_atlas_frames_flag(const bool value) noexcept
    -> auto & {
  m_asps_long_term_ref_atlas_frames_flag = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_use_eight_orientations_flag(const bool value) noexcept
    -> auto & {
  m_asps_use_eight_orientations_flag = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_extended_projection_enabled_flag(const bool value) noexcept
    -> auto & {
  m_asps_extended_projection_enabled_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_normal_axis_limits_quantization_enabled_flag(
    const bool value) noexcept -> auto & {
  m_asps_normal_axis_limits_quantization_enabled_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_normal_axis_max_delta_value_enabled_flag(
    const bool value) noexcept -> auto & {
  m_asps_normal_axis_max_delta_value_enabled_flag = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_pixel_deinterleaving_flag(const bool value) noexcept -> auto & {
  m_asps_pixel_deinterleaving_flag = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_patch_precedence_order_flag(const bool value) noexcept
    -> auto & {
  m_asps_patch_precedence_order_flag = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_patch_size_quantizer_present_flag(const bool value) noexcept
    -> auto & {
  m_asps_patch_size_quantizer_present_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_raw_patch_enabled_flag(const bool value) noexcept
    -> auto & {
  m_asps_raw_patch_enabled_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_eom_patch_enabled_flag(const bool value) noexcept
    -> auto & {
  m_asps_eom_patch_enabled_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_point_local_reconstruction_enabled_flag(
    const bool value) noexcept -> auto & {
  m_asps_point_local_reconstruction_enabled_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_map_count_minus1(const uint8_t value) noexcept
    -> auto & {
  m_asps_map_count_minus1 = value;
  return *this;
}

constexpr auto
AtlasSequenceParameterSetRBSP::asps_vui_parameters_present_flag(const bool value) noexcept
    -> auto & {
  m_asps_vui_parameters_present_flag = value;
  return *this;
}

constexpr auto AtlasSequenceParameterSetRBSP::asps_extension_present_flag(const bool value) noexcept
    -> auto & {
  m_asps_extension_present_flag = value;
  return *this;
}
} // namespace TMIV::MivBitstream
