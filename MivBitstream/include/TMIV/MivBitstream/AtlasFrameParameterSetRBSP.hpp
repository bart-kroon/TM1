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

#ifndef _TMIV_MIVBITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto AtlasFrameParameterSetRBSP::afps_atlas_frame_parameter_set_id() const noexcept {
  return m_afps_atlas_frame_parameter_set_id;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_atlas_sequence_parameter_set_id() const noexcept {
  return m_afps_atlas_sequence_parameter_set_id;
}

constexpr auto AtlasFrameParameterSetRBSP::atlas_frame_tile_information() const noexcept {
  return m_atlas_frame_tile_information;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_num_ref_idx_default_active() const noexcept {
  return m_afps_num_ref_idx_default_active;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_additional_lt_afoc_lsb_len() const noexcept {
  return m_afps_additional_lt_afoc_lsb_len;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_2d_pos_x_bit_count() const noexcept {
  return m_afps_2d_pos_x_bit_count;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_2d_pos_y_bit_count() const noexcept {
  return m_afps_2d_pos_y_bit_count;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_3d_pos_x_bit_count() const noexcept {
  return m_afps_3d_pos_x_bit_count;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_3d_pos_y_bit_count() const noexcept {
  return m_afps_3d_pos_y_bit_count;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_lod_bit_count() const noexcept {
  return m_afps_lod_bit_count;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_override_eom_for_depth_flag() const noexcept {
  return m_afps_override_eom_for_depth_flag;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_raw_3d_pos_bit_count_explicit_mode_flag() const
    noexcept {
  return m_afps_raw_3d_pos_bit_count_explicit_mode_flag;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_fixed_camera_model_flag() const noexcept {
  return m_afps_fixed_camera_model_flag;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_extension_present_flag() const noexcept {
  return m_afps_extension_present_flag;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_atlas_frame_parameter_set_id(const std::uint8_t value) noexcept {
  m_afps_atlas_frame_parameter_set_id = value;
  return *this;
}

constexpr auto &AtlasFrameParameterSetRBSP::afps_atlas_sequence_parameter_set_id(
    const std::uint8_t value) noexcept {
  m_afps_atlas_sequence_parameter_set_id = value;
  return *this;
}

constexpr auto &AtlasFrameParameterSetRBSP::atlas_frame_tile_information(
    const AtlasFrameTileInformation &value) noexcept {
  m_atlas_frame_tile_information = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_num_ref_idx_default_active(const std::uint8_t value) noexcept {
  m_afps_num_ref_idx_default_active = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_additional_lt_afoc_lsb_len(const std::uint8_t value) noexcept {
  m_afps_additional_lt_afoc_lsb_len = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_2d_pos_x_bit_count(const std::uint8_t value) noexcept {
  m_afps_2d_pos_x_bit_count = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_2d_pos_y_bit_count(const std::uint8_t value) noexcept {
  m_afps_2d_pos_y_bit_count = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_3d_pos_x_bit_count(const std::uint8_t value) noexcept {
  m_afps_3d_pos_x_bit_count = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_3d_pos_y_bit_count(const std::uint8_t value) noexcept {
  m_afps_3d_pos_y_bit_count = value;
  return *this;
}

constexpr auto &AtlasFrameParameterSetRBSP::afps_lod_bit_count(const std::uint8_t value) noexcept {
  m_afps_lod_bit_count = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_override_eom_for_depth_flag(const bool value) noexcept {
  m_afps_override_eom_for_depth_flag = value;
  return *this;
}

constexpr auto &AtlasFrameParameterSetRBSP::afps_raw_3d_pos_bit_count_explicit_mode_flag(
    const bool value) noexcept {
  m_afps_raw_3d_pos_bit_count_explicit_mode_flag = value;
  return *this;
}

constexpr auto &
AtlasFrameParameterSetRBSP::afps_fixed_camera_model_flag(const bool value) noexcept {
  m_afps_fixed_camera_model_flag = value;
  return *this;
}

constexpr auto &AtlasFrameParameterSetRBSP::afps_extension_present_flag(const bool value) noexcept {
  m_afps_extension_present_flag = value;
  return *this;
}

} // namespace TMIV::MivBitstream
