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

#ifndef TMIV_MIVBITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto AtlasFrameTileInformation::afti_single_tile_in_atlas_frame_flag() const noexcept {
  return m_afti_single_tile_in_atlas_frame_flag;
}

constexpr auto AtlasFrameTileInformation::afti_signalled_tile_id_flag() noexcept { return false; }

constexpr auto AtlasFrameTileInformation::afti_uniform_partition_spacing_flag() const noexcept {
  return m_afti_uniform_partition_spacing_flag;
}

constexpr auto AtlasFrameTileInformation::afti_partition_cols_width_minus1() const noexcept {
  return m_afti_partition_cols_width_minus1;
}

constexpr auto AtlasFrameTileInformation::afti_partition_rows_height_minus1() const noexcept {
  return m_afti_partition_rows_height_minus1;
}

inline auto AtlasFrameTileInformation::afti_num_partition_columns_minus1() const {
  VERIFY_V3CBITSTREAM(!afti_uniform_partition_spacing_flag());
  return Common::downCast<uint8_t>(m_afti_partition_column_width_minus1.size());
}

inline auto AtlasFrameTileInformation::afti_num_partition_rows_minus1() const {
  VERIFY_V3CBITSTREAM(!afti_uniform_partition_spacing_flag());
  return Common::downCast<uint8_t>(m_afti_partition_row_height_minus1.size());
}

inline auto AtlasFrameTileInformation::afti_partition_column_width_minus1(uint8_t i) const {
  VERIFY_V3CBITSTREAM(i < afti_num_partition_columns_minus1());
  return m_afti_partition_column_width_minus1[i];
}

inline auto AtlasFrameTileInformation::afti_partition_row_height_minus1(uint8_t i) const {
  VERIFY_V3CBITSTREAM(i < afti_num_partition_rows_minus1());
  return m_afti_partition_row_height_minus1[i];
}

inline auto AtlasFrameTileInformation::afti_num_tiles_in_atlas_frame_minus1() const {
  return Common::downCast<uint8_t>(m_tiles.size() - 1);
}

constexpr auto AtlasFrameTileInformation::afti_single_partition_per_tile_flag() const noexcept {
  return m_afti_single_partition_per_tile_flag;
}

inline auto AtlasFrameTileInformation::afti_top_left_partition_idx(uint8_t i) const {
  VERIFY_V3CBITSTREAM(i <= afti_num_tiles_in_atlas_frame_minus1());
  return m_tiles[i].afti_top_left_partition_idx;
}

inline auto AtlasFrameTileInformation::afti_bottom_right_partition_column_offset(uint8_t i) const {
  VERIFY_V3CBITSTREAM(i <= afti_num_tiles_in_atlas_frame_minus1());
  return m_tiles[i].afti_bottom_right_partition_column_offset;
}

inline auto AtlasFrameTileInformation::afti_bottom_right_partition_row_offset(uint8_t i) const {
  VERIFY_V3CBITSTREAM(i <= afti_num_tiles_in_atlas_frame_minus1());
  return m_tiles[i].afti_bottom_right_partition_row_offset;
}

constexpr auto
AtlasFrameTileInformation::afti_single_tile_in_atlas_frame_flag(const bool value) noexcept
    -> AtlasFrameTileInformation & {
  m_afti_single_tile_in_atlas_frame_flag = value;
  return *this;
}

constexpr auto
AtlasFrameTileInformation::afti_uniform_partition_spacing_flag(const bool value) noexcept
    -> AtlasFrameTileInformation & {
  m_afti_uniform_partition_spacing_flag = value;
  return *this;
}

constexpr auto
AtlasFrameTileInformation::afti_partition_cols_width_minus1(const int32_t value) noexcept
    -> AtlasFrameTileInformation & {
  m_afti_partition_cols_width_minus1 = value;
  return *this;
}

constexpr auto
AtlasFrameTileInformation::afti_partition_rows_height_minus1(const int32_t value) noexcept
    -> AtlasFrameTileInformation & {
  m_afti_partition_rows_height_minus1 = value;
  return *this;
}

inline auto AtlasFrameTileInformation::afti_num_partition_columns_minus1(uint8_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(!afti_uniform_partition_spacing_flag());
  m_afti_partition_column_width_minus1.resize(value);
  return *this;
}

inline auto AtlasFrameTileInformation::afti_num_partition_rows_minus1(uint8_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(!afti_uniform_partition_spacing_flag());
  m_afti_partition_row_height_minus1.resize(value);
  return *this;
}

inline auto AtlasFrameTileInformation::afti_partition_column_width_minus1(uint8_t i, int32_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(!afti_uniform_partition_spacing_flag());
  VERIFY_V3CBITSTREAM(i < afti_num_partition_columns_minus1());
  m_afti_partition_column_width_minus1[i] = value;
  return *this;
}

inline auto AtlasFrameTileInformation::afti_partition_row_height_minus1(uint8_t i, int32_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(!afti_uniform_partition_spacing_flag());
  VERIFY_V3CBITSTREAM(i < afti_num_partition_rows_minus1());
  m_afti_partition_row_height_minus1[i] = value;
  return *this;
}

constexpr auto
AtlasFrameTileInformation::afti_single_partition_per_tile_flag(const bool value) noexcept
    -> AtlasFrameTileInformation & {
  m_afti_single_partition_per_tile_flag = value;
  return *this;
}

inline auto AtlasFrameTileInformation::afti_num_tiles_in_atlas_frame_minus1(uint8_t value)
    -> AtlasFrameTileInformation & {
  m_tiles.resize(value + size_t{1});
  return *this;
}

inline auto AtlasFrameTileInformation::afti_top_left_partition_idx(uint8_t i, int32_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(i <= afti_num_tiles_in_atlas_frame_minus1());
  m_tiles[i].afti_top_left_partition_idx = value;
  return *this;
}

inline auto AtlasFrameTileInformation::afti_bottom_right_partition_column_offset(uint8_t i,
                                                                                 int32_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(i <= afti_num_tiles_in_atlas_frame_minus1());
  m_tiles[i].afti_bottom_right_partition_column_offset = value;
  return *this;
}

inline auto AtlasFrameTileInformation::afti_bottom_right_partition_row_offset(uint8_t i,
                                                                              int32_t value)
    -> AtlasFrameTileInformation & {
  VERIFY_V3CBITSTREAM(i <= afti_num_tiles_in_atlas_frame_minus1());
  m_tiles[i].afti_bottom_right_partition_row_offset = value;
  return *this;
}

inline auto AtlasFrameTileInformation::aftiSignalledTileIDBitCount() const -> uint8_t {
  LIMITATION(!afti_signalled_tile_id_flag());
  return Common::ceilLog2(afti_num_tiles_in_atlas_frame_minus1() + 1);
}

inline auto
AtlasFrameTileInformation::numPartitionColumns(const AtlasSequenceParameterSetRBSP &asps) const {
  if (afti_uniform_partition_spacing_flag()) {
    const auto partitionWidth = (afti_partition_cols_width_minus1() + 1) * 64;
    VERIFY_V3CBITSTREAM(0 < asps.asps_frame_width());
    VERIFY_V3CBITSTREAM(asps.asps_frame_width() % partitionWidth == 0);
    return Common::downCast<size_t>(asps.asps_frame_width() / partitionWidth);
  }
  return afti_num_partition_columns_minus1() + size_t{1};
}

inline auto
AtlasFrameTileInformation::numPartitionRows(const AtlasSequenceParameterSetRBSP &asps) const {
  if (afti_uniform_partition_spacing_flag()) {
    const auto partitionHeight = (afti_partition_rows_height_minus1() + 1) * 64;
    VERIFY_V3CBITSTREAM(0 < asps.asps_frame_height());
    VERIFY_V3CBITSTREAM(asps.asps_frame_height() % partitionHeight == 0);
    return Common::downCast<size_t>(asps.asps_frame_height() / partitionHeight);
  }
  return afti_num_partition_rows_minus1() + size_t{1};
}

inline auto AtlasFrameTileInformation::numPartitionsInAtlasFrame(
    const AtlasSequenceParameterSetRBSP &asps) const {
  return numPartitionColumns(asps) * numPartitionRows(asps);
}

constexpr auto AfpsMivExtension::afme_inpaint_lod_enabled_flag() const noexcept {
  return m_afme_inpaint_lod_enabled_flag.value_or(false);
}

constexpr auto AfpsMivExtension::afme_inpaint_lod_scale_x_minus1() const noexcept {
  return m_afme_inpaint_lod_scale_x_minus1.value_or(0);
}

constexpr auto AfpsMivExtension::afme_inpaint_lod_scale_y_idc() const noexcept {
  return m_afme_inpaint_lod_scale_y_idc.value_or(0);
}

constexpr auto AfpsMivExtension::afme_inpaint_lod_enabled_flag(bool value) noexcept -> auto & {
  m_afme_inpaint_lod_enabled_flag = value;
  return *this;
}

constexpr auto AfpsMivExtension::operator==(const AfpsMivExtension &other) const noexcept {
  return m_afme_inpaint_lod_enabled_flag == other.m_afme_inpaint_lod_enabled_flag &&
         m_afme_inpaint_lod_scale_x_minus1 == other.m_afme_inpaint_lod_scale_x_minus1 &&
         m_afme_inpaint_lod_scale_y_idc == other.m_afme_inpaint_lod_scale_y_idc;
}

constexpr auto AfpsMivExtension::operator!=(const AfpsMivExtension &other) const noexcept {
  return !operator==(other);
}

constexpr auto AtlasFrameParameterSetRBSP::afps_atlas_frame_parameter_set_id() const noexcept {
  return m_afps_atlas_frame_parameter_set_id;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_atlas_sequence_parameter_set_id() const noexcept {
  return m_afps_atlas_sequence_parameter_set_id;
}

inline auto AtlasFrameParameterSetRBSP::atlas_frame_tile_information() const
    -> const AtlasFrameTileInformation & {
  return m_atlas_frame_tile_information;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_output_flag_present_flag() const noexcept {
  return m_afps_output_flag_present_flag;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_num_ref_idx_default_active_minus1() const noexcept {
  return m_afps_num_ref_idx_default_active_minus1;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_additional_lt_afoc_lsb_len() const noexcept {
  return m_afps_additional_lt_afoc_lsb_len;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_lod_mode_enabled_flag() const noexcept {
  return m_afps_lod_enabled_flag;
}

constexpr auto
AtlasFrameParameterSetRBSP::afps_raw_3d_offset_bit_count_explicit_mode_flag() const noexcept {
  return m_afps_raw_3d_offset_bit_count_explicit_mode_flag;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_extension_present_flag() const noexcept {
  return m_afps_extension_present_flag;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_atlas_frame_parameter_set_id(const uint8_t value)
    -> auto & {
  VERIFY_MIVBITSTREAM(value <= 63U);
  m_afps_atlas_frame_parameter_set_id = value;
  return *this;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_atlas_sequence_parameter_set_id(const uint8_t value)
    -> auto & {
  VERIFY_MIVBITSTREAM(value <= 63U);
  m_afps_atlas_sequence_parameter_set_id = value;
  return *this;
}

inline auto
AtlasFrameParameterSetRBSP::atlas_frame_tile_information(const AtlasFrameTileInformation &value)
    -> auto & {
  m_atlas_frame_tile_information = value;
  return *this;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_output_flag_present_flag(const bool value) noexcept
    -> auto & {
  m_afps_output_flag_present_flag = value;
  return *this;
}

constexpr auto
AtlasFrameParameterSetRBSP::afps_num_ref_idx_default_active_minus1(const uint8_t value) noexcept
    -> auto & {
  m_afps_num_ref_idx_default_active_minus1 = value;
  return *this;
}

constexpr auto
AtlasFrameParameterSetRBSP::afps_additional_lt_afoc_lsb_len(const uint8_t value) noexcept
    -> auto & {
  m_afps_additional_lt_afoc_lsb_len = value;
  return *this;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_lod_mode_enabled_flag(const bool value) noexcept
    -> auto & {
  m_afps_lod_enabled_flag = value;
  return *this;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_raw_3d_offset_bit_count_explicit_mode_flag(
    const bool value) noexcept -> auto & {
  m_afps_raw_3d_offset_bit_count_explicit_mode_flag = value;
  return *this;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_extension_present_flag(const bool value) noexcept
    -> auto & {
  m_afps_extension_present_flag = value;
  return *this;
}

constexpr auto AtlasFrameParameterSetRBSP::afps_miv_extension_present_flag() const noexcept {
  return m_afps_miv_extension_present_flag.value_or(false);
}

constexpr auto AtlasFrameParameterSetRBSP::afps_extension_7bits() const noexcept {
  return m_afps_extension_7bits.value_or(0);
}
} // namespace TMIV::MivBitstream
