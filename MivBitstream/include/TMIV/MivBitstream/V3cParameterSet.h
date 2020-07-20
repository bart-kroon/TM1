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
#define _TMIV_MIVBITSTREAM_V3CPARAMETERSET_H_

#include <TMIV/MivBitstream/MivVuiParams.h>

#include <TMIV/Common/Bitstream.h>

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace TMIV::MivBitstream {
enum class PtlProfileCodecGroupIdc : std::uint8_t {
  AVC_Progressive_High,
  HEVC_Main10,
  HEVC444,
  MP4RA,
  count
};

enum class PtlProfilePccToolsetIdc : std::uint8_t { Basic, Extended, MIV_Main = 64 };
enum class PtlProfileReconstructionIdc : std::uint8_t { Rec0, Rec1, Unconstrained, MIV_Main = 64 };
enum class PtlLevelIdc : std::uint8_t { Level_1_0 = 30, Level_2_0 = 60, Level_3_0 = 90 };

enum class AiAttributeTypeId : std::uint8_t {
  ATTR_TEXTURE,
  ATTR_MATERIAL_ID,
  ATTR_TRANSPARENCY,
  ATTR_REFLECTANCE,
  ATTR_NORMAL,
  ATTR_UNSPECIFIED = 15
};

auto operator<<(std::ostream &stream, const PtlProfileCodecGroupIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlProfilePccToolsetIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlProfileReconstructionIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlLevelIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const AiAttributeTypeId &x) -> std::ostream &;

// TMIV-internal filename convention
auto codeOf(AiAttributeTypeId typeId) -> char;

// 23090-5: profile_tier_level()
//
// Limitations of this implementation:
//   * ptl_tool_constraints_present_flag == 0
class ProfileTierLevel {
public:
  [[nodiscard]] constexpr auto ptl_tier_flag() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_codec_group_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_toolset_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_reconstruction_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_level_idc() const noexcept;
  [[nodiscard]] auto ptl_num_sub_profiles() const noexcept -> std::uint8_t;
  [[nodiscard]] constexpr auto ptl_extended_sub_profile_flag() const noexcept;
  [[nodiscard]] auto ptl_sub_profile_idc(std::uint8_t i) const noexcept -> std::uint64_t;
  [[nodiscard]] constexpr auto ptl_tool_constraints_present_flag() const noexcept;

  constexpr auto ptl_tier_flag(bool value) noexcept -> auto &;
  constexpr auto ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_toolset_idc(PtlProfilePccToolsetIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc value) noexcept
      -> auto &;
  constexpr auto ptl_level_idc(PtlLevelIdc value) noexcept -> auto &;
  auto ptl_num_sub_profiles(std::uint8_t value) noexcept -> ProfileTierLevel &;
  auto ptl_extended_sub_profile_flag(bool value) noexcept -> ProfileTierLevel &;
  auto ptl_sub_profile_idc(std::uint8_t i, std::uint64_t value) noexcept -> ProfileTierLevel &;
  constexpr auto ptl_tool_constraints_present_flag(bool value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const ProfileTierLevel &x) -> std::ostream &;

  auto operator==(const ProfileTierLevel &other) const noexcept -> bool;
  auto operator!=(const ProfileTierLevel &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ProfileTierLevel;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_ptl_tier_flag{};
  PtlProfileCodecGroupIdc m_ptl_profile_codec_group_idc{};
  PtlProfilePccToolsetIdc m_ptl_profile_toolset_idc{};
  PtlProfileReconstructionIdc m_ptl_profile_reconstruction_idc{};
  PtlLevelIdc m_ptl_level_idc{};
  std::vector<std::uint64_t> m_subProfileIdcs;
  bool m_ptl_extended_sub_profile_flag{};
  bool m_ptl_tool_constraints_present_flag{};
};

// 23090-5: occupancy_information( atlasIdx )
class OccupancyInformation {
public:
  [[nodiscard]] constexpr auto oi_occupancy_codec_id() const noexcept;
  [[nodiscard]] constexpr auto oi_lossy_occupancy_map_compression_threshold() const noexcept;
  [[nodiscard]] constexpr auto oi_occupancy_nominal_2d_bitdepth_minus1() const noexcept;
  [[nodiscard]] constexpr auto oi_occupancy_MSB_align_flag() const noexcept;

  constexpr auto oi_occupancy_codec_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto oi_lossy_occupancy_map_compression_threshold(std::uint8_t value) noexcept
      -> auto &;
  constexpr auto oi_occupancy_nominal_2d_bitdepth_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto oi_occupancy_MSB_align_flag(bool value) noexcept -> auto &;

  auto printTo(std::ostream &stream, std::uint8_t atlasIdx) const -> std::ostream &;

  auto operator==(const OccupancyInformation &other) const noexcept -> bool;
  auto operator!=(const OccupancyInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> OccupancyInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::uint8_t m_oi_occupancy_codec_id{};
  std::uint8_t m_oi_lossy_occupancy_map_compression_threshold{};
  std::uint8_t m_oi_occupancy_nominal_2d_bitdepth_minus1{};
  bool m_oi_occupancy_MSB_align_flag{};
};

class V3cParameterSet;

// 23090-5: geometry_information( atlasIdx )
//
// 23090-12 restrictions:
//   * vps_auxiliary_video_present_flag[ ] == 0
class GeometryInformation {
public:
  [[nodiscard]] constexpr auto gi_geometry_codec_id() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_nominal_2d_bitdepth_minus1() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_MSB_align_flag() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_3d_coordinates_bitdepth_minus1() const noexcept;

  constexpr auto gi_geometry_codec_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto gi_geometry_nominal_2d_bitdepth_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto gi_geometry_MSB_align_flag(bool value) noexcept -> auto &;
  constexpr auto gi_geometry_3d_coordinates_bitdepth_minus1(std::uint8_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, std::uint8_t atlasIdx) const -> std::ostream &;

  auto operator==(const GeometryInformation &other) const noexcept -> bool;
  auto operator!=(const GeometryInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                         uint8_t atlasIdx) -> GeometryInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                uint8_t atlasIdx) const;

private:
  std::uint8_t m_gi_geometry_codec_id{};
  std::uint8_t m_gi_geometry_nominal_2d_bitdepth_minus1{};
  bool m_gi_geometry_MSB_align_flag{};
  std::uint8_t m_gi_geometry_3d_coordinates_bitdepth_minus1{};
};

// 23090-5: attribute_information( atlasIdx )
//
// 23090-12 restrictions:
//   * vps_auxiliary_video_present_flag[ ] == 0
//   * ai_attribute_dimension_partitions_minus1[ ][ ] == 0
class AttributeInformation {
public:
  [[nodiscard]] auto ai_attribute_count() const noexcept -> std::uint8_t;
  [[nodiscard]] auto ai_attribute_type_id(std::uint8_t attributeId) const -> AiAttributeTypeId;
  [[nodiscard]] auto ai_attribute_codec_id(std::uint8_t attributeId) const -> std::uint8_t;
  [[nodiscard]] auto
  ai_attribute_map_absolute_coding_persistence_flag(std::uint8_t attributeId) const -> bool;
  [[nodiscard]] auto ai_attribute_dimension_minus1(std::uint8_t attributeId) const -> std::uint8_t;
  [[nodiscard]] auto ai_attribute_nominal_2d_bitdepth_minus1(std::uint8_t attributeId) const
      -> std::uint8_t;
  [[nodiscard]] auto ai_attribute_MSB_align_flag(std::uint8_t attributeId) const -> bool;

  auto ai_attribute_count(std::uint8_t value) -> AttributeInformation &;
  auto ai_attribute_type_id(std::uint8_t attributeId, AiAttributeTypeId value)
      -> AttributeInformation &;
  auto ai_attribute_codec_id(std::uint8_t attributeId, std::uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_map_absolute_coding_persistence_flag(std::uint8_t attributeId, bool value)
      -> AttributeInformation &;
  auto ai_attribute_dimension_minus1(std::uint8_t attributeId, std::uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_nominal_2d_bitdepth_minus1(std::uint8_t attributeId, std::uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_MSB_align_flag(std::uint8_t attributeId, bool value) -> AttributeInformation &;

  auto printTo(std::ostream &stream, std::uint8_t atlasIdx) const -> std::ostream &;

  auto operator==(const AttributeInformation &other) const noexcept -> bool;
  auto operator!=(const AttributeInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                         std::uint8_t atlasIdx) -> AttributeInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                std::uint8_t atlasIdx) const;

private:
  struct AiAttribute {
    AiAttributeTypeId ai_attribute_type_id{};
    std::uint8_t ai_attribute_codec_id{};
    std::optional<bool> ai_attribute_map_absolute_coding_persistence_flag{};
    std::uint8_t ai_attribute_dimension_minus1{};
    std::uint8_t ai_attribute_nominal_2d_bitdepth_minus1{};
    bool ai_attribute_MSB_align_flag{};
  };

  std::vector<AiAttribute> m_aiAttributes; // 23090-5: ai_attribute_count
};

// 23090-5: vps_vpcc_extension()
class VpsVpccExtension {
public:
  friend constexpr decltype(auto) operator<<(std::ostream &stream, const VpsVpccExtension &x);

  constexpr auto operator==(const VpsVpccExtension &other) const noexcept;
  constexpr auto operator!=(const VpsVpccExtension &other) const noexcept;

  static constexpr auto decodeFrom(Common::InputBitstream &bitstream) -> VpsVpccExtension;

  constexpr void encodeTo(Common::OutputBitstream &bitstream) const;
};

// 23090-12: vps_miv_extension()
class VpsMivExtension {
public:
  [[nodiscard]] constexpr auto vme_depth_low_quality_flag() const noexcept;
  [[nodiscard]] constexpr auto vme_geometry_scale_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto vme_num_groups_minus1() const noexcept;
  [[nodiscard]] constexpr auto vme_max_entities_minus1() const noexcept;
  [[nodiscard]] constexpr auto vme_vui_params_present_flag() const noexcept;
  [[nodiscard]] auto miv_vui_parameters() const noexcept -> const MivVuiParams &;

  constexpr auto vme_depth_low_quality_flag(const bool value) noexcept -> auto &;
  constexpr auto vme_geometry_scale_enabled_flag(const bool value) noexcept -> auto &;
  constexpr auto vme_num_groups_minus1(const unsigned value) noexcept -> auto &;
  constexpr auto vme_max_entities_minus1(const unsigned value) noexcept -> auto &;
  constexpr auto vme_vui_params_present_flag(bool value) noexcept -> auto &;
  auto miv_vui_parameters(const MivVuiParams &value) noexcept -> VpsMivExtension &;

  friend auto operator<<(std::ostream &stream, const VpsMivExtension &x) -> std::ostream &;

  constexpr auto operator==(const VpsMivExtension &other) const noexcept;
  constexpr auto operator!=(const VpsMivExtension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> VpsMivExtension;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_vme_depth_low_quality_flag{};
  bool m_vme_geometry_scale_enabled_flag{};
  unsigned m_vme_num_groups_minus1{};
  unsigned m_vme_max_entities_minus1{};
  bool m_vme_vui_params_present_flag{};
  std::optional<MivVuiParams> m_mvp;
};

// 23090-5: v3c_parameter_set()
//
// 23090-12 restrictions:
//   * vps_multiple_map_streams_present_flag[ ] == 0
class V3cParameterSet {
public:
  [[nodiscard]] auto profile_tier_level() const noexcept -> const ProfileTierLevel &;
  [[nodiscard]] constexpr auto vps_v3c_parameter_set_id() const noexcept;
  [[nodiscard]] auto vps_atlas_count_minus1() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vps_atlas_id(std::uint8_t j) const -> std::uint8_t;
  [[nodiscard]] auto vps_frame_width(std::uint8_t j) const -> std::uint16_t;
  [[nodiscard]] auto vps_frame_height(std::uint8_t j) const -> std::uint16_t;
  [[nodiscard]] auto vps_map_count_minus1(std::uint8_t j) const -> std::uint8_t;
  [[nodiscard]] auto vps_auxiliary_video_present_flag(std::uint8_t j) const -> bool;
  [[nodiscard]] auto vps_occupancy_video_present_flag(std::uint8_t j) const -> bool;
  [[nodiscard]] auto vps_geometry_video_present_flag(std::uint8_t j) const -> bool;
  [[nodiscard]] auto vps_attribute_video_present_flag(std::uint8_t j) const -> bool;
  [[nodiscard]] auto occupancy_information(std::uint8_t j) const -> const OccupancyInformation &;
  [[nodiscard]] auto geometry_information(std::uint8_t j) const -> const GeometryInformation &;
  [[nodiscard]] auto attribute_information(std::uint8_t j) const -> const AttributeInformation &;
  [[nodiscard]] constexpr auto vps_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto vps_vpcc_extension_flag() const noexcept;
  [[nodiscard]] constexpr auto vps_miv_extension_flag() const noexcept;
  [[nodiscard]] constexpr auto vps_extension_6bits() const noexcept;
  [[nodiscard]] auto vps_vpcc_extension() const noexcept -> const VpsVpccExtension &;
  [[nodiscard]] auto vps_miv_extension() const noexcept -> const VpsMivExtension &;
  [[nodiscard]] auto vps_extension_length_minus1() const noexcept -> std::size_t;
  [[nodiscard]] auto vpsExtensionData() const noexcept -> const std::vector<std::uint8_t> &;

  auto profile_tier_level(ProfileTierLevel value) noexcept -> V3cParameterSet &;
  constexpr auto vps_v3c_parameter_set_id(std::uint8_t value) noexcept -> auto &;
  auto vps_atlas_count_minus1(std::uint8_t value) -> V3cParameterSet &;
  auto vps_atlas_id(std::uint8_t j, std::uint8_t value) -> V3cParameterSet &;
  auto vps_frame_width(std::uint8_t j, std::uint16_t value) -> V3cParameterSet &;
  auto vps_frame_height(std::uint8_t j, std::uint16_t value) -> V3cParameterSet &;
  auto vps_map_count_minus1(std::uint8_t j, std::uint8_t value) -> V3cParameterSet &;
  auto vps_auxiliary_video_present_flag(std::uint8_t j, bool value) -> V3cParameterSet &;
  auto vps_occupancy_video_present_flag(std::uint8_t j, bool value) -> V3cParameterSet &;
  auto vps_geometry_video_present_flag(std::uint8_t j, bool value) -> V3cParameterSet &;
  auto vps_attribute_video_present_flag(std::uint8_t j, bool value) -> V3cParameterSet &;
  auto occupancy_information(std::uint8_t j, OccupancyInformation value) -> V3cParameterSet &;
  auto geometry_information(std::uint8_t j, GeometryInformation value) -> V3cParameterSet &;
  auto attribute_information(std::uint8_t j, AttributeInformation value) -> V3cParameterSet &;
  constexpr auto vps_extension_present_flag(bool value) noexcept -> auto &;
  auto vps_vpcc_extension_flag(bool value) noexcept -> V3cParameterSet &;
  auto vps_miv_extension_flag(bool value) noexcept -> V3cParameterSet &;
  auto vps_extension_6bits(std::uint8_t value) noexcept -> V3cParameterSet &;
  auto vps_vpcc_extension(VpsVpccExtension value) noexcept -> V3cParameterSet &;
  auto vps_miv_extension(VpsMivExtension value) noexcept -> V3cParameterSet &;
  auto vpsExtensionData(std::vector<std::uint8_t> value) noexcept -> V3cParameterSet &;

  constexpr auto profile_tier_level() noexcept -> auto &;
  [[nodiscard]] auto occupancy_information(std::uint8_t j) -> OccupancyInformation &;
  [[nodiscard]] auto geometry_information(std::uint8_t j) -> GeometryInformation &;
  [[nodiscard]] auto attribute_information(std::uint8_t j) -> AttributeInformation &;
  [[nodiscard]] auto vps_miv_extension() noexcept -> VpsMivExtension &;

  [[nodiscard]] auto atlasIdxOf(std::uint8_t atlasId) const noexcept -> std::uint8_t;

  friend auto operator<<(std::ostream &stream, const V3cParameterSet &x) -> std::ostream &;

  auto operator==(const V3cParameterSet &other) const noexcept -> bool;
  auto operator!=(const V3cParameterSet &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> V3cParameterSet;

  void encodeTo(std::ostream &stream) const;

  friend auto merge(const std::vector<const V3cParameterSet *> &vps) -> V3cParameterSet;

private:
  struct VpsAtlas {
    std::uint8_t vps_atlas_id{};
    std::uint16_t vps_frame_width{};
    std::uint16_t vps_frame_height{};
    std::uint8_t vps_map_count_minus1{};
    bool vps_auxiliary_video_present_flag{};
    bool vps_occupancy_video_present_flag{};
    bool vps_geometry_video_present_flag{};
    bool vps_attribute_video_present_flag{};
    std::optional<OccupancyInformation> occupancy_information{};
    std::optional<GeometryInformation> geometry_information{};
    std::optional<AttributeInformation> attribute_information{};
  };

  ProfileTierLevel m_profile_tier_level;
  std::uint8_t m_vps_v3c_parameter_set_id{};
  std::vector<VpsAtlas> m_vpsAtlases{VpsAtlas{}};
  bool m_vps_extension_present_flag{};
  std::optional<bool> m_vps_vpcc_extension_flag{};
  std::optional<bool> m_vps_miv_extension_flag{};
  std::optional<std::uint8_t> m_vps_extension_6bits{};
  std::optional<VpsVpccExtension> m_vps_vpcc_extension;
  std::optional<VpsMivExtension> m_vps_miv_extension;
  std::optional<std::vector<std::uint8_t>> m_vpsExtensionData;
};
} // namespace TMIV::MivBitstream

#include "V3cParameterSet.hpp"

#endif
