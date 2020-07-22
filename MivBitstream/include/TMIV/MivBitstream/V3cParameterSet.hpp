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

#ifndef _TMIV_MIVBITSTREAM_V3CPARAMETERSET_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto ProfileTierLevel::ptl_tier_flag() const noexcept { return m_ptl_tier_flag; }

constexpr auto ProfileTierLevel::ptl_profile_codec_group_idc() const noexcept {
  return m_ptl_profile_codec_group_idc;
}

constexpr auto ProfileTierLevel::ptl_profile_toolset_idc() const noexcept {
  return m_ptl_profile_toolset_idc;
}

constexpr auto ProfileTierLevel::ptl_profile_reconstruction_idc() const noexcept {
  return m_ptl_profile_reconstruction_idc;
}

constexpr auto ProfileTierLevel::ptl_max_decodes_idc() const noexcept {
  return m_ptl_max_decodes_idc;
}

constexpr auto ProfileTierLevel::ptl_level_idc() const noexcept { return m_ptl_level_idc; }

constexpr auto ProfileTierLevel::ptl_extended_sub_profile_flag() const noexcept {
  return m_ptl_extended_sub_profile_flag;
}

constexpr auto ProfileTierLevel::ptl_tool_constraints_present_flag() const noexcept {
  return m_ptl_tool_constraints_present_flag;
}

constexpr auto ProfileTierLevel::ptl_tier_flag(bool value) noexcept -> auto & {
  m_ptl_tier_flag = value;
  return *this;
}

constexpr auto ProfileTierLevel::ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc value) noexcept
    -> auto & {
  m_ptl_profile_codec_group_idc = value;
  return *this;
}

constexpr auto ProfileTierLevel::ptl_profile_toolset_idc(PtlProfilePccToolsetIdc value) noexcept
    -> auto & {
  m_ptl_profile_toolset_idc = value;
  return *this;
}

constexpr auto
ProfileTierLevel::ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc value) noexcept
    -> auto & {
  m_ptl_profile_reconstruction_idc = value;
  return *this;
}

constexpr auto ProfileTierLevel::ptl_max_decodes_idc(PtlMaxDecodesIdc value) noexcept
    -> auto & {
  m_ptl_max_decodes_idc = value;
  return *this;
}

constexpr auto ProfileTierLevel::ptl_level_idc(PtlLevelIdc value) noexcept -> auto & {
  m_ptl_level_idc = value;
  return *this;
}

constexpr auto ProfileTierLevel::ptl_tool_constraints_present_flag(bool value) noexcept -> auto & {
  m_ptl_tool_constraints_present_flag = value;
  return *this;
}

constexpr auto OccupancyInformation::oi_occupancy_codec_id() const noexcept {
  return m_oi_occupancy_codec_id;
}

constexpr auto OccupancyInformation::oi_lossy_occupancy_map_compression_threshold() const noexcept {
  return m_oi_lossy_occupancy_map_compression_threshold;
}

constexpr auto OccupancyInformation::oi_occupancy_nominal_2d_bitdepth_minus1() const noexcept {
  return m_oi_occupancy_nominal_2d_bitdepth_minus1;
}

constexpr auto OccupancyInformation::oi_occupancy_MSB_align_flag() const noexcept {
  return m_oi_occupancy_MSB_align_flag;
}

constexpr auto OccupancyInformation::oi_occupancy_codec_id(std::uint8_t value) noexcept -> auto & {
  m_oi_occupancy_codec_id = value;
  return *this;
}

constexpr auto
OccupancyInformation::oi_lossy_occupancy_map_compression_threshold(std::uint8_t value) noexcept
    -> auto & {
  m_oi_lossy_occupancy_map_compression_threshold = value;
  return *this;
}

constexpr auto
OccupancyInformation::oi_occupancy_nominal_2d_bitdepth_minus1(std::uint8_t value) noexcept
    -> auto & {
  m_oi_occupancy_nominal_2d_bitdepth_minus1 = value;
  return *this;
}

constexpr auto OccupancyInformation::oi_occupancy_MSB_align_flag(bool value) noexcept -> auto & {
  m_oi_occupancy_MSB_align_flag = value;
  return *this;
}

constexpr auto GeometryInformation::gi_geometry_codec_id() const noexcept {
  return m_gi_geometry_codec_id;
}

constexpr auto GeometryInformation::gi_geometry_nominal_2d_bitdepth_minus1() const noexcept {
  return m_gi_geometry_nominal_2d_bitdepth_minus1;
}

constexpr auto GeometryInformation::gi_geometry_MSB_align_flag() const noexcept {
  return m_gi_geometry_MSB_align_flag;
}

constexpr auto GeometryInformation::gi_geometry_3d_coordinates_bitdepth_minus1() const noexcept {
  return m_gi_geometry_3d_coordinates_bitdepth_minus1;
}

constexpr auto GeometryInformation::gi_geometry_codec_id(std::uint8_t value) noexcept -> auto & {
  m_gi_geometry_codec_id = value;
  return *this;
}

constexpr auto
GeometryInformation::gi_geometry_nominal_2d_bitdepth_minus1(std::uint8_t value) noexcept -> auto & {
  m_gi_geometry_nominal_2d_bitdepth_minus1 = value;
  return *this;
}

constexpr auto GeometryInformation::gi_geometry_MSB_align_flag(bool value) noexcept -> auto & {
  m_gi_geometry_MSB_align_flag = value;
  return *this;
}

constexpr auto
GeometryInformation::gi_geometry_3d_coordinates_bitdepth_minus1(std::uint8_t value) noexcept
    -> auto & {
  m_gi_geometry_3d_coordinates_bitdepth_minus1 = value;
  return *this;
}

constexpr decltype(auto) operator<<(std::ostream &stream, const VpsVpccExtension & /* x */) {
  return stream;
}

constexpr auto VpsVpccExtension::operator==(const VpsVpccExtension & /* other */) const noexcept {
  return true;
}
constexpr auto VpsVpccExtension::operator!=(const VpsVpccExtension & /* other */) const noexcept {
  return false;
}

constexpr auto VpsVpccExtension::decodeFrom(Common::InputBitstream & /* bitstream */)
    -> VpsVpccExtension {
  return {};
}

constexpr void VpsVpccExtension::encodeTo(Common::OutputBitstream & /* bitstream */) const {}

constexpr auto VpsMivExtension::vme_depth_low_quality_flag() const noexcept {
  return m_vme_depth_low_quality_flag;
}

constexpr auto VpsMivExtension::vme_geometry_scale_enabled_flag() const noexcept {
  return m_vme_geometry_scale_enabled_flag;
}

constexpr auto VpsMivExtension::vme_num_groups_minus1() const noexcept {
  return m_vme_num_groups_minus1;
}

constexpr auto VpsMivExtension::vme_max_entities_minus1() const noexcept {
  return m_vme_max_entities_minus1;
}

constexpr auto VpsMivExtension::vme_embedded_occupancy_flag() const noexcept {
  return m_vme_embedded_occupancy_flag;
}

constexpr auto VpsMivExtension::vme_occupancy_scale_enabled_flag() const noexcept {
  return m_vme_occupancy_scale_enabled_flag;
}

constexpr auto VpsMivExtension::vme_vui_params_present_flag() const noexcept {
  return m_vme_vui_params_present_flag;
}

constexpr auto VpsMivExtension::vme_depth_low_quality_flag(const bool value) noexcept -> auto & {
  m_vme_depth_low_quality_flag = value;
  return *this;
}

constexpr auto VpsMivExtension::vme_geometry_scale_enabled_flag(const bool value) noexcept
    -> auto & {
  m_vme_geometry_scale_enabled_flag = value;
  return *this;
}

constexpr auto VpsMivExtension::vme_num_groups_minus1(const unsigned value) noexcept -> auto & {
  m_vme_num_groups_minus1 = value;
  return *this;
}
constexpr auto VpsMivExtension::vme_max_entities_minus1(const unsigned value) noexcept -> auto & {
  m_vme_max_entities_minus1 = value;
  return *this;
}

constexpr auto VpsMivExtension::vme_embedded_occupancy_flag(const bool value) noexcept
    -> auto & {
  m_vme_embedded_occupancy_flag = value;
  return *this;
}

constexpr auto VpsMivExtension::vme_occupancy_scale_enabled_flag(const bool value) noexcept
    -> auto & {
  m_vme_occupancy_scale_enabled_flag = value;
  return *this;
}

constexpr auto VpsMivExtension::vme_vui_params_present_flag(bool value) noexcept -> auto & {
  m_vme_vui_params_present_flag = value;
  return *this;
}

constexpr auto VpsMivExtension::operator==(const VpsMivExtension &other) const noexcept {
  return vme_depth_low_quality_flag() == other.vme_depth_low_quality_flag() &&
         vme_geometry_scale_enabled_flag() == other.vme_geometry_scale_enabled_flag() &&
         vme_num_groups_minus1() == other.vme_num_groups_minus1() &&
         vme_max_entities_minus1() == other.vme_max_entities_minus1() &&
         vme_embedded_occupancy_flag() == other.vme_embedded_occupancy_flag() &&
         vme_occupancy_scale_enabled_flag() == other.vme_occupancy_scale_enabled_flag();
}

constexpr auto VpsMivExtension::operator!=(const VpsMivExtension &other) const noexcept {
  return !operator==(other);
}

constexpr auto V3cParameterSet::vps_v3c_parameter_set_id() const noexcept {
  return m_vps_v3c_parameter_set_id;
}

constexpr auto V3cParameterSet::vps_extension_present_flag() const noexcept {
  return m_vps_extension_present_flag;
}

constexpr auto V3cParameterSet::vps_vpcc_extension_flag() const noexcept {
  return m_vps_vpcc_extension_flag.value_or(false);
}

constexpr auto V3cParameterSet::vps_miv_extension_flag() const noexcept {
  return m_vps_miv_extension_flag.value_or(false);
}

constexpr auto V3cParameterSet::vps_extension_6bits() const noexcept {
  return m_vps_extension_6bits.value_or(0);
}

constexpr auto V3cParameterSet::vps_v3c_parameter_set_id(std::uint8_t value) noexcept -> auto & {
  m_vps_v3c_parameter_set_id = value;
  return *this;
}

constexpr auto V3cParameterSet::vps_extension_present_flag(bool value) noexcept -> auto & {
  m_vps_extension_present_flag = value;
  return *this;
}

constexpr auto V3cParameterSet::profile_tier_level() noexcept -> auto & {
  return m_profile_tier_level;
}
} // namespace TMIV::MivBitstream
