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

#ifndef _TMIV_MIVBITSTREAM_VPCCPARAMETERSET_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto ProfileTierLevel::ptl_tier_flag() const noexcept { return m_ptl_tier_flag; }

constexpr auto ProfileTierLevel::ptl_profile_codec_group_idc() const noexcept {
  return m_ptl_profile_codec_group_idc;
}

constexpr auto ProfileTierLevel::ptl_profile_pcc_toolset_idc() const noexcept {
  return m_ptl_profile_pcc_toolset_idc;
}

constexpr auto ProfileTierLevel::ptl_profile_reconstruction_idc() const noexcept {
  return m_ptl_profile_reconstruction_idc;
}

constexpr auto ProfileTierLevel::ptl_level_idc() const noexcept { return m_ptl_level_idc; }

constexpr auto &ProfileTierLevel::ptl_tier_flag(bool value) noexcept {
  m_ptl_tier_flag = value;
  return *this;
}

constexpr auto &
ProfileTierLevel::ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc value) noexcept {
  m_ptl_profile_codec_group_idc = value;
  return *this;
}

constexpr auto &
ProfileTierLevel::ptl_profile_pcc_toolset_idc(PtlProfilePccToolsetIdc value) noexcept {
  m_ptl_profile_pcc_toolset_idc = value;
  return *this;
}

constexpr auto &
ProfileTierLevel::ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc value) noexcept {
  m_ptl_profile_reconstruction_idc = value;
  return *this;
}

constexpr auto &ProfileTierLevel::ptl_level_idc(PtlLevelIdc value) noexcept {
  m_ptl_level_idc = value;
  return *this;
}

constexpr auto OccupancyInformation::oi_occupancy_codec_id() const noexcept {
  return m_oi_occupancy_codec_id;
}

constexpr auto OccupancyInformation::oi_lossy_occupancy_map_compression_threshold() const noexcept {
  return m_oi_lossy_occupancy_map_compression_threshold;
}

constexpr auto OccupancyInformation::oi_occupancy_nominal_2d_bitdepth() const noexcept {
  return m_oi_occupancy_nominal_2d_bitdepth;
}

constexpr auto OccupancyInformation::oi_occupancy_MSB_align_flag() const noexcept {
  return m_oi_occupancy_MSB_align_flag;
}

constexpr auto &OccupancyInformation::oi_occupancy_codec_id(std::uint8_t value) noexcept {
  m_oi_occupancy_codec_id = value;
  return *this;
}

constexpr auto &
OccupancyInformation::oi_lossy_occupancy_map_compression_threshold(std::uint8_t value) noexcept {
  m_oi_lossy_occupancy_map_compression_threshold = value;
  return *this;
}

constexpr auto &
OccupancyInformation::oi_occupancy_nominal_2d_bitdepth(std::uint8_t value) noexcept {
  m_oi_occupancy_nominal_2d_bitdepth = value;
  return *this;
}

constexpr auto &OccupancyInformation::oi_occupancy_MSB_align_flag(bool value) noexcept {
  m_oi_occupancy_MSB_align_flag = value;
  return *this;
}

constexpr auto GeometryInformation::gi_geometry_codec_id() const noexcept {
  return m_gi_geometry_codec_id;
}

constexpr auto GeometryInformation::gi_geometry_nominal_2d_bitdepth() const noexcept {
  return m_gi_geometry_nominal_2d_bitdepth;
}

constexpr auto GeometryInformation::gi_geometry_MSB_align_flag() const noexcept {
  return m_gi_geometry_MSB_align_flag;
}

constexpr auto GeometryInformation::gi_geometry_3d_coordinates_bitdepth() const noexcept {
  return m_gi_geometry_3d_coordinates_bitdepth;
}

constexpr auto &GeometryInformation::gi_geometry_codec_id(std::uint8_t value) noexcept {
  m_gi_geometry_codec_id = value;
  return *this;
}

constexpr auto &GeometryInformation::gi_geometry_nominal_2d_bitdepth(std::uint8_t value) noexcept {
  m_gi_geometry_nominal_2d_bitdepth = value;
  return *this;
}

constexpr auto &GeometryInformation::gi_geometry_MSB_align_flag(bool value) noexcept {
  m_gi_geometry_MSB_align_flag = value;
  return *this;
}

constexpr auto &
GeometryInformation::gi_geometry_3d_coordinates_bitdepth(std::uint8_t value) noexcept {
  m_gi_geometry_3d_coordinates_bitdepth = value;
  return *this;
}

constexpr auto MivSequenceParams::msp_depth_low_quality_flag() const noexcept {
  return m_msp_depth_low_quality_flag;
}

constexpr auto MivSequenceParams::msp_geometry_scale_enabled_flag() const noexcept {
  return m_msp_geometry_scale_enabled_flag;
}

constexpr auto MivSequenceParams::msp_num_groups_minus1() const noexcept {
  return m_msp_num_groups_minus1;
}

constexpr auto MivSequenceParams::msp_max_entities_minus1() const noexcept {
  return m_msp_max_entities_minus1;
}

constexpr auto &MivSequenceParams::msp_depth_low_quality_flag(const bool value) noexcept {
  m_msp_depth_low_quality_flag = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_geometry_scale_enabled_flag(const bool value) noexcept {
  m_msp_geometry_scale_enabled_flag = value;
  return *this;
}

constexpr auto &MivSequenceParams::msp_num_groups_minus1(const unsigned value) noexcept {
  m_msp_num_groups_minus1 = value;
  return *this;
}
constexpr auto &MivSequenceParams::msp_max_entities_minus1(const unsigned value) noexcept {
  m_msp_max_entities_minus1 = value;
  return *this;
}

constexpr auto MivSequenceParams::operator==(const MivSequenceParams &other) const noexcept {
  return msp_depth_low_quality_flag() == other.msp_depth_low_quality_flag() &&
         msp_geometry_scale_enabled_flag() == other.msp_geometry_scale_enabled_flag() &&
         msp_num_groups_minus1() == other.msp_num_groups_minus1() &&
         msp_max_entities_minus1() == other.msp_max_entities_minus1();
}

constexpr auto MivSequenceParams::operator!=(const MivSequenceParams &other) const noexcept {
  return !operator==(other);
}

constexpr auto VpccParameterSet::profile_tier_level() const noexcept {
  return m_profile_tier_level;
}

constexpr auto VpccParameterSet::vps_vpcc_parameter_set_id() const noexcept {
  return m_vps_vpcc_parameter_set_id;
}

constexpr auto VpccParameterSet::vps_miv_mode_flag() const noexcept { return m_vps_miv_mode_flag; }

constexpr auto VpccParameterSet::vps_extension_present_flag() const noexcept {
  return m_vps_extension_present_flag;
}

constexpr auto &VpccParameterSet::profile_tier_level(ProfileTierLevel value) noexcept {
  m_profile_tier_level = value;
  return *this;
}

constexpr auto &VpccParameterSet::vps_vpcc_parameter_set_id(std::uint8_t value) noexcept {
  m_vps_vpcc_parameter_set_id = value;
  return *this;
}

constexpr auto &VpccParameterSet::vps_miv_mode_flag(const bool value) noexcept {
  m_vps_miv_mode_flag = value;
  return *this;
}

constexpr auto &VpccParameterSet::vps_extension_present_flag(bool value) noexcept {
  m_vps_extension_present_flag = value;
  return *this;
}

constexpr auto &VpccParameterSet::profile_tier_level() noexcept { return m_profile_tier_level; }
} // namespace TMIV::MivBitstream
