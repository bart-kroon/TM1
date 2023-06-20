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

#ifndef TMIV_MIVBITSTREAM_V3CPARAMETERSET_H
#define TMIV_MIVBITSTREAM_V3CPARAMETERSET_H

#include "AtlasId.h"

#include <TMIV/Common/Common.h>
#include <TMIV/Common/FlatMap.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace TMIV::MivBitstream {
enum class PtlProfileCodecGroupIdc : uint8_t {
  AVC_Progressive_High,
  HEVC_Main10,
  HEVC444,
  VVC_Main10,
  MP4RA = 127
};

static constexpr auto knownCodecGroupIdcs =
    std::array{PtlProfileCodecGroupIdc::AVC_Progressive_High, PtlProfileCodecGroupIdc::HEVC_Main10,
               PtlProfileCodecGroupIdc::HEVC444, PtlProfileCodecGroupIdc::VVC_Main10,
               PtlProfileCodecGroupIdc::MP4RA};

enum class PtlProfileToolsetIdc : uint8_t {
  VPCC_Basic,
  VPCC_Extended,
  MIV_Main = 64,
  MIV_Extended,
  MIV_Geometry_Absent
};

static constexpr auto knownToolsetIdcs =
    std::array{PtlProfileToolsetIdc::VPCC_Basic, PtlProfileToolsetIdc::VPCC_Extended,
               PtlProfileToolsetIdc::MIV_Main, PtlProfileToolsetIdc::MIV_Extended,
               PtlProfileToolsetIdc::MIV_Geometry_Absent};

// Clause 3: MIV toolset profile component
static constexpr auto mivToolsetProfileComponents =
    std::array{PtlProfileToolsetIdc::MIV_Main, PtlProfileToolsetIdc::MIV_Extended,
               PtlProfileToolsetIdc::MIV_Geometry_Absent};

enum class PtlProfileReconstructionIdc : uint8_t {
  VPCC_Rec0,              // ISO/IEC DIS 23090-5(2E):2021 Table H-4 (V-PCC)
  VPCC_Rec1,              // ISO/IEC DIS 23090-5(2E):2021 Table H-4 (V-PCC)
  VPCC_Rec2,              // ISO/IEC DIS 23090-5(2E):2021 Table H-4 (V-PCC)
  Rec_Unconstrained = 255 // ISO/IEC DIS 23090-5(2E):2021 Table A-4 (V3C)
};

static constexpr auto knownReconstructionIdcs = std::array{
    PtlProfileReconstructionIdc::VPCC_Rec0, PtlProfileReconstructionIdc::VPCC_Rec1,
    PtlProfileReconstructionIdc::VPCC_Rec2, PtlProfileReconstructionIdc::Rec_Unconstrained};

enum class PtlMaxDecodesIdc : uint8_t {
  max_1,
  max_2,
  max_3,
  max_4,
  max_6,
  max_12,
  max_16,
  max_24,
  max_32,
  unconstrained = 15
};

enum class PtlLevelIdc : uint8_t {
  Level_1_0 = 30,
  Level_1_5 = 45,
  Level_2_0 = 60,
  Level_2_5 = 75,
  Level_3_0 = 90,
  Level_3_5 = 105,
  Level_4_0 = 120,
  Level_4_5 = 135,
  Level_8_5 = 255
};

static constexpr auto knownLevelIdcs =
    std::array{PtlLevelIdc::Level_1_0, PtlLevelIdc::Level_1_5, PtlLevelIdc::Level_2_0,
               PtlLevelIdc::Level_2_5, PtlLevelIdc::Level_3_0, PtlLevelIdc::Level_3_5,
               PtlLevelIdc::Level_4_0, PtlLevelIdc::Level_4_5, PtlLevelIdc::Level_8_5};

enum class VuhUnitType : uint8_t { V3C_VPS, V3C_AD, V3C_OVD, V3C_GVD, V3C_AVD, V3C_PVD, V3C_CAD };

enum class AiAttributeTypeId : uint8_t {
  ATTR_TEXTURE,
  ATTR_MATERIAL_ID,
  ATTR_TRANSPARENCY,
  ATTR_REFLECTANCE,
  ATTR_NORMAL,
  ATTR_UNSPECIFIED = 15
};

enum class VpsExtensionType : uint8_t {
  VPS_EXT_UNSPECIFIED,
  VPS_EXT_PACKED,
  VPS_EXT_MIV,
  VPS_EXT_MIV_2
};

auto operator<<(std::ostream &stream, PtlProfileCodecGroupIdc x) -> std::ostream &;
auto operator<<(std::ostream &stream, PtlProfileToolsetIdc x) -> std::ostream &;
auto operator<<(std::ostream &stream, PtlProfileReconstructionIdc x) -> std::ostream &;
auto operator<<(std::ostream &stream, PtlMaxDecodesIdc x) -> std::ostream &;
auto operator<<(std::ostream &stream, PtlLevelIdc x) -> std::ostream &;
auto operator<<(std::ostream &stream, VuhUnitType x) -> std::ostream &;
auto operator<<(std::ostream &stream, AiAttributeTypeId x) -> std::ostream &;
auto operator<<(std::ostream &stream, VpsExtensionType x) -> std::ostream &;

// TMIV-internal filename convention
auto codeOf(AiAttributeTypeId typeId) -> char;

// 23090-5: profile_toolset_constraints_information
//
// 23090-12 restrictions:
//   * ptc_num_reserved_constraint_bytes[ ] == 0
class ProfileToolsetConstraintsInformation {
public:
  [[nodiscard]] constexpr auto ptc_one_v3c_frame_only_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_eom_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_max_map_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_max_atlas_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_multiple_map_streams_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_plr_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_attribute_max_dimension_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_attribute_max_dimension_partitions_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_no_eight_orientations_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_no_45degree_projection_patch_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_restricted_geometry_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_num_reserved_constraint_bytes() const noexcept;

  constexpr auto ptc_one_v3c_frame_only_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_eom_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_max_map_count_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto ptc_max_atlas_count_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto ptc_multiple_map_streams_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_plr_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_attribute_max_dimension_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto ptc_attribute_max_dimension_partitions_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto ptc_no_eight_orientations_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_no_45degree_projection_patch_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_restricted_geometry_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_num_reserved_constraint_bytes(uint8_t value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const ProfileToolsetConstraintsInformation &x)
      -> std::ostream &;

  auto operator==(const ProfileToolsetConstraintsInformation &other) const noexcept -> bool;
  auto operator!=(const ProfileToolsetConstraintsInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ProfileToolsetConstraintsInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_ptc_one_v3c_frame_only_flag{};
  bool m_ptc_eom_constraint_flag{};
  uint8_t m_ptc_max_map_count_minus1{};
  uint8_t m_ptc_max_atlas_count_minus1{};
  bool m_ptc_multiple_map_streams_constraint_flag{};
  bool m_ptc_plr_constraint_flag{};
  uint8_t m_ptc_attribute_max_dimension_minus1{};
  uint8_t m_ptc_attribute_max_dimension_partitions_minus1{};
  bool m_ptc_no_eight_orientations_constraint_flag{};
  bool m_ptc_no_45degree_projection_patch_constraint_flag{};
  bool m_ptc_restricted_geometry_flag{};
  uint8_t m_ptc_num_reserved_constraint_bytes{};
};

// 23090-5: profile_tier_level()
//
class ProfileTierLevel {
public:
  [[nodiscard]] constexpr auto ptl_tier_flag() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_codec_group_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_toolset_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_reconstruction_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_max_decodes_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_level_idc() const noexcept;
  [[nodiscard]] auto ptl_num_sub_profiles() const -> uint8_t;
  [[nodiscard]] constexpr auto ptl_extended_sub_profile_flag() const noexcept;
  [[nodiscard]] auto ptl_sub_profile_idc(uint8_t i) const -> uint64_t;
  [[nodiscard]] constexpr auto ptl_toolset_constraints_present_flag() const noexcept;
  [[nodiscard]] auto ptl_profile_toolset_constraints_information() const
      -> const ProfileToolsetConstraintsInformation &;

  constexpr auto ptl_tier_flag(bool value) noexcept -> auto &;
  constexpr auto ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_toolset_idc(PtlProfileToolsetIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc value) noexcept
      -> auto &;
  constexpr auto ptl_max_decodes_idc(PtlMaxDecodesIdc value) noexcept -> auto &;
  constexpr auto ptl_level_idc(PtlLevelIdc value) noexcept -> auto &;
  auto ptl_num_sub_profiles(uint8_t value) noexcept -> ProfileTierLevel &;
  auto ptl_extended_sub_profile_flag(bool value) -> ProfileTierLevel &;
  auto ptl_sub_profile_idc(uint8_t i, uint64_t value) -> ProfileTierLevel &;
  constexpr auto ptl_toolset_constraints_present_flag(bool value) noexcept -> auto &;
  auto
  ptl_profile_toolset_constraints_information(ProfileToolsetConstraintsInformation value) noexcept
      -> ProfileTierLevel &;

  friend auto operator<<(std::ostream &stream, const ProfileTierLevel &x) -> std::ostream &;

  auto operator==(const ProfileTierLevel &other) const noexcept -> bool;
  auto operator!=(const ProfileTierLevel &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ProfileTierLevel;

  void encodeTo(Common::OutputBitstream &bitstream) const;

  // The profile name as specified in ISO/IEC DIS 23090-5(2E):2021 A.2
  [[nodiscard]] auto profile() const -> std::string;

private:
  bool m_ptl_tier_flag{};
  PtlProfileCodecGroupIdc m_ptl_profile_codec_group_idc{};
  PtlProfileToolsetIdc m_ptl_profile_toolset_idc{};
  PtlProfileReconstructionIdc m_ptl_profile_reconstruction_idc{};
  PtlMaxDecodesIdc m_ptl_max_decodes_idc{PtlMaxDecodesIdc::unconstrained};
  PtlLevelIdc m_ptl_level_idc{};
  std::vector<uint64_t> m_subProfileIdcs;
  bool m_ptl_extended_sub_profile_flag{};
  bool m_ptl_toolset_constraints_present_flag{};
  std::optional<ProfileToolsetConstraintsInformation>
      m_ptl_profile_toolset_constraints_information{};
};

// 23090-5: occupancy_information( atlasID )
class OccupancyInformation {
public:
  [[nodiscard]] constexpr auto oi_occupancy_codec_id() const noexcept;
  [[nodiscard]] constexpr auto oi_lossy_occupancy_compression_threshold() const noexcept;
  [[nodiscard]] constexpr auto oi_occupancy_2d_bit_depth_minus1() const noexcept;
  [[nodiscard]] constexpr auto oi_occupancy_MSB_align_flag() const noexcept;

  constexpr auto oi_occupancy_codec_id(uint8_t value) noexcept -> auto &;
  constexpr auto oi_lossy_occupancy_compression_threshold(uint8_t value) noexcept -> auto &;
  constexpr auto oi_occupancy_2d_bit_depth_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto oi_occupancy_MSB_align_flag(bool value) noexcept -> auto &;

  auto printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream &;

  auto operator==(const OccupancyInformation &other) const noexcept -> bool;
  auto operator!=(const OccupancyInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> OccupancyInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint8_t m_oi_occupancy_codec_id{};
  uint8_t m_oi_lossy_occupancy_compression_threshold{};
  uint8_t m_oi_occupancy_2d_bit_depth_minus1{};
  bool m_oi_occupancy_MSB_align_flag{};
};

class V3cParameterSet;

// 23090-5: geometry_information( atlasID )
class GeometryInformation {
public:
  [[nodiscard]] constexpr auto gi_geometry_codec_id() const noexcept;
  [[nodiscard]] constexpr auto gi_auxiliary_geometry_codec_id() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_2d_bit_depth_minus1() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_msb_align_flag() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_3d_coordinates_bit_depth_minus1() const noexcept;

  constexpr auto gi_geometry_codec_id(uint8_t value) noexcept -> auto &;
  constexpr auto gi_auxiliary_geometry_codec_id(uint8_t value) noexcept -> auto &;
  constexpr auto gi_geometry_2d_bit_depth_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto gi_geometry_msb_align_flag(bool value) noexcept -> auto &;
  constexpr auto gi_geometry_3d_coordinates_bit_depth_minus1(uint8_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream &;

  auto operator==(const GeometryInformation &other) const noexcept -> bool;
  auto operator!=(const GeometryInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                         AtlasId atlasId) -> GeometryInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                AtlasId atlasId) const;

private:
  uint8_t m_gi_geometry_codec_id{};
  std::optional<uint8_t> m_gi_auxiliary_geometry_codec_id{};
  uint8_t m_gi_geometry_2d_bit_depth_minus1{};
  bool m_gi_geometry_MSB_align_flag{};
  uint8_t m_gi_geometry_3d_coordinates_bit_depth_minus1{};
};

// 23090-5: attribute_information( atlasId )
class AttributeInformation {
public:
  [[nodiscard]] auto ai_attribute_count() const noexcept -> uint8_t;
  [[nodiscard]] auto ai_attribute_type_id(uint8_t i) const -> AiAttributeTypeId;
  [[nodiscard]] auto ai_attribute_codec_id(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto ai_auxiliary_attribute_codec_id(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto ai_attribute_map_absolute_coding_persistence_flag(uint8_t i) const -> bool;
  [[nodiscard]] auto ai_attribute_dimension_minus1(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto ai_attribute_dimension_partitions_minus1(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto ai_attribute_partition_channels_minus1(uint8_t i, uint8_t k) const -> uint8_t;
  [[nodiscard]] auto ai_attribute_2d_bit_depth_minus1(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto ai_attribute_msb_align_flag(uint8_t i) const -> bool;

  auto ai_attribute_count(uint8_t value) -> AttributeInformation &;
  auto ai_attribute_type_id(uint8_t i, AiAttributeTypeId value) -> AttributeInformation &;
  auto ai_attribute_codec_id(uint8_t i, uint8_t value) -> AttributeInformation &;
  auto ai_auxiliary_attribute_codec_id(uint8_t i, uint8_t value) -> AttributeInformation &;
  auto ai_attribute_map_absolute_coding_persistence_flag(uint8_t i, bool value)
      -> AttributeInformation &;
  auto ai_attribute_dimension_minus1(uint8_t i, uint8_t value) -> AttributeInformation &;
  auto ai_attribute_dimension_partitions_minus1(uint8_t i, uint8_t value) -> AttributeInformation &;
  auto ai_attribute_partition_channels_minus1(uint8_t i, uint8_t k, uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_2d_bit_depth_minus1(uint8_t i, uint8_t value) -> AttributeInformation &;
  auto ai_attribute_msb_align_flag(uint8_t i, bool value) -> AttributeInformation &;

  auto printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream &;

  auto operator==(const AttributeInformation &other) const -> bool;
  auto operator!=(const AttributeInformation &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                         AtlasId atlasId) -> AttributeInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                AtlasId atlasId) const;

private:
  struct AiAttribute {
    AiAttributeTypeId ai_attribute_type_id{};
    uint8_t ai_attribute_codec_id{};
    std::optional<uint8_t> ai_auxiliary_attribute_codec_id;
    std::optional<bool> ai_attribute_map_absolute_coding_persistence_flag{};
    uint8_t ai_attribute_dimension_minus1{};

    // size: ai_attribute_dimension_partitions_minus1[ atlasID ][ i ] + 1
    std::vector<uint8_t> ai_attribute_partition_channels_minus1{{}};

    uint8_t ai_attribute_2d_bit_depth_minus1{};
    bool ai_attribute_msb_align_flag{};
  };

  std::vector<AiAttribute> m_aiAttributes; // size: ai_attribute_count
};

struct PinRegion {
  auto operator==(const PinRegion &other) const noexcept -> bool {
    return (pin_region_tile_id == other.pin_region_tile_id) &&
           (pin_region_type_id_minus2 == other.pin_region_type_id_minus2) &&
           (pin_region_top_left_x == other.pin_region_top_left_x) &&
           (pin_region_top_left_y == other.pin_region_top_left_y) &&
           (pin_region_width_minus1 == other.pin_region_width_minus1) &&
           (pin_region_height_minus1 == other.pin_region_height_minus1) &&
           (pin_region_unpack_top_left_x == other.pin_region_unpack_top_left_x) &&
           (pin_region_unpack_top_left_y == other.pin_region_unpack_top_left_y) &&
           (pin_region_rotation_flag == other.pin_region_rotation_flag) &&
           (pin_region_map_index == other.pin_region_map_index) &&
           (pin_region_auxiliary_data_flag == other.pin_region_auxiliary_data_flag) &&
           (pin_region_attr_index == other.pin_region_attr_index) &&
           (pin_region_attr_partition_index == other.pin_region_attr_partition_index);
  }

  uint8_t pin_region_tile_id{};
  uint8_t pin_region_type_id_minus2{};
  uint16_t pin_region_top_left_x{};
  uint16_t pin_region_top_left_y{};
  uint16_t pin_region_width_minus1{};
  uint16_t pin_region_height_minus1{};
  uint16_t pin_region_unpack_top_left_x{};
  uint16_t pin_region_unpack_top_left_y{};
  bool pin_region_rotation_flag{};
  std::optional<uint8_t> pin_region_map_index{};
  std::optional<bool> pin_region_auxiliary_data_flag{};
  std::optional<uint8_t> pin_region_attr_index{};
  std::optional<uint8_t> pin_region_attr_partition_index{};

  [[nodiscard]] constexpr auto pinRegionTypeId() const noexcept {
    return static_cast<VuhUnitType>(pin_region_type_id_minus2 + 2);
  }
};

struct PinAttributeInformation {
  auto operator==(const PinAttributeInformation &other) const noexcept -> bool {
    return (pin_attribute_type_id == other.pin_attribute_type_id) &&
           (pin_attribute_2d_bit_depth_minus1 == other.pin_attribute_2d_bit_depth_minus1) &&
           (pin_attribute_MSB_align_flag == other.pin_attribute_MSB_align_flag) &&
           (pin_attribute_map_absolute_coding_persistence_flag ==
            other.pin_attribute_map_absolute_coding_persistence_flag) &&
           (pin_attribute_dimension_minus1 == other.pin_attribute_dimension_minus1) &&
           (pin_attribute_dimension_partitions_minus1 ==
            other.pin_attribute_dimension_partitions_minus1) &&
           (pin_attribute_partition_channels_minus1 ==
            other.pin_attribute_partition_channels_minus1);
  }

  AiAttributeTypeId pin_attribute_type_id{};
  uint8_t pin_attribute_2d_bit_depth_minus1{};
  bool pin_attribute_MSB_align_flag{};
  bool pin_attribute_map_absolute_coding_persistence_flag{};
  uint8_t pin_attribute_dimension_minus1{};
  std::optional<uint8_t> pin_attribute_dimension_partitions_minus1{};
  std::optional<std::vector<uint8_t>> pin_attribute_partition_channels_minus1{};
};

// 23090-5: packing_information( j )
class PackingInformation {
public:
  [[nodiscard]] auto pin_codec_id() const noexcept -> uint8_t;
  [[nodiscard]] auto pin_occupancy_present_flag() const -> bool;
  [[nodiscard]] auto pin_geometry_present_flag() const -> bool;
  [[nodiscard]] auto pin_attribute_present_flag() const -> bool;
  [[nodiscard]] auto pin_occupancy_2d_bit_depth_minus1() const -> uint8_t;
  [[nodiscard]] auto pin_occupancy_MSB_align_flag() const -> bool;
  [[nodiscard]] auto pin_lossy_occupancy_compression_threshold() const -> uint8_t;
  [[nodiscard]] auto pin_geometry_2d_bit_depth_minus1() const -> uint8_t;
  [[nodiscard]] auto pin_geometry_MSB_align_flag() const -> bool;
  [[nodiscard]] auto pin_geometry_3d_coordinates_bit_depth_minus1() const -> uint8_t;
  [[nodiscard]] auto pin_attribute_count() const -> uint8_t;
  [[nodiscard]] auto pin_attribute_type_id(size_t i) const -> AiAttributeTypeId;
  [[nodiscard]] auto pin_attribute_2d_bit_depth_minus1(size_t i) const -> uint8_t;
  [[nodiscard]] auto pin_attribute_MSB_align_flag(size_t i) const -> bool;
  [[nodiscard]] auto pin_attribute_map_absolute_coding_persistence_flag(size_t i) const -> bool;
  [[nodiscard]] auto pin_attribute_dimension_minus1(size_t i) const -> uint8_t;
  [[nodiscard]] auto pin_attribute_dimension_partitions_minus1(size_t i) const -> uint8_t;
  [[nodiscard]] auto pin_attribute_partition_channels_minus1(size_t i, uint8_t l) const -> uint8_t;
  [[nodiscard]] auto pin_regions_count_minus1() const -> uint8_t;
  [[nodiscard]] auto pin_region_tile_id(size_t i) const -> uint8_t;
  [[nodiscard]] auto pin_region_type_id_minus2(size_t i) const -> uint8_t;
  [[nodiscard]] auto pinRegionTypeId(size_t i) const -> VuhUnitType;
  [[nodiscard]] auto pin_region_top_left_x(size_t i) const -> uint16_t;
  [[nodiscard]] auto pin_region_top_left_y(size_t i) const -> uint16_t;
  [[nodiscard]] auto pin_region_width_minus1(size_t i) const -> uint16_t;
  [[nodiscard]] auto pin_region_height_minus1(size_t i) const -> uint16_t;
  [[nodiscard]] auto pin_region_unpack_top_left_x(size_t i) const -> uint16_t;
  [[nodiscard]] auto pin_region_unpack_top_left_y(size_t i) const -> uint16_t;
  [[nodiscard]] auto pin_region_rotation_flag(size_t i) const -> bool;
  [[nodiscard]] auto pin_region_map_index(size_t i) const -> uint8_t;
  [[nodiscard]] auto pin_region_auxiliary_data_flag(size_t i) const -> bool;
  [[nodiscard]] auto pin_region_attr_index(size_t i) const -> uint8_t;
  [[nodiscard]] auto pin_region_attr_partition_index(size_t i) const -> uint8_t;

  constexpr auto pin_codec_id(uint8_t value) noexcept -> auto &;
  auto pin_occupancy_present_flag(bool value) -> auto &;
  auto pin_geometry_present_flag(bool value) -> auto &;
  auto pin_attribute_present_flag(bool value) -> auto &;
  auto pin_occupancy_2d_bit_depth_minus1(uint8_t value) -> auto &;
  auto pin_occupancy_MSB_align_flag(bool value) -> auto &;
  auto pin_lossy_occupancy_compression_threshold(uint8_t value) -> auto &;
  auto pin_geometry_2d_bit_depth_minus1(uint8_t value) -> auto &;
  auto pin_geometry_MSB_align_flag(bool value) -> auto &;
  auto pin_geometry_3d_coordinates_bit_depth_minus1(uint8_t value) -> auto &;
  auto pin_attribute_count(uint8_t value) -> auto &;
  auto pin_attribute_type_id(size_t i, AiAttributeTypeId value) -> auto &;
  auto pin_attribute_2d_bit_depth_minus1(size_t i, uint8_t value) -> auto &;
  auto pin_attribute_MSB_align_flag(size_t i, bool value) -> auto &;
  auto pin_attribute_map_absolute_coding_persistence_flag(size_t i, bool value) -> auto &;
  auto pin_attribute_dimension_minus1(size_t i, uint8_t value) -> auto &;
  auto pin_attribute_dimension_partitions_minus1(size_t i, uint8_t value) -> auto &;
  auto pin_attribute_partition_channels_minus1(size_t i, uint8_t l, uint8_t value) -> auto &;
  auto pin_regions_count_minus1(uint8_t value) -> auto &;
  auto pin_region_tile_id(size_t i, uint8_t value) -> auto &;
  auto pin_region_type_id_minus2(size_t i, uint8_t value) -> auto &;
  auto pinRegionTypeId(size_t i, VuhUnitType value) -> auto &;
  auto pin_region_top_left_x(size_t i, uint16_t value) -> auto &;
  auto pin_region_top_left_y(size_t i, uint16_t value) -> auto &;
  auto pin_region_width_minus1(size_t i, uint16_t value) -> auto &;
  auto pin_region_height_minus1(size_t i, uint16_t value) -> auto &;
  auto pin_region_unpack_top_left_x(size_t i, uint16_t value) -> auto &;
  auto pin_region_unpack_top_left_y(size_t i, uint16_t value) -> auto &;
  auto pin_region_map_index(size_t i, uint8_t value) -> auto &;
  auto pin_region_rotation_flag(size_t i, bool value) -> auto &;
  auto pin_region_auxiliary_data_flag(size_t i, bool value) -> auto &;
  auto pin_region_attr_index(size_t i, uint8_t value) -> auto &;
  auto pin_region_attr_partition_index(size_t i, uint8_t value) -> auto &;

  auto printTo(std::ostream &stream, AtlasId j) const -> std::ostream &;

  auto operator==(const PackingInformation &other) const noexcept -> bool;
  auto operator!=(const PackingInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> PackingInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint8_t m_pin_codec_id{};
  bool m_pin_occupancy_present_flag{};
  bool m_pin_geometry_present_flag{};
  bool m_pin_attribute_present_flag{};
  std::optional<uint8_t> m_pin_occupancy_2d_bit_depth_minus1{};
  std::optional<bool> m_pin_occupancy_MSB_align_flag{};
  std::optional<uint8_t> m_pin_lossy_occupancy_compression_threshold{};
  std::optional<uint8_t> m_pin_geometry_2d_bit_depth_minus1{};
  std::optional<bool> m_pin_geometry_MSB_align_flag{};
  std::optional<uint8_t> m_pin_geometry_3d_coordinates_bit_depth_minus1{};
  std::optional<uint8_t> m_pin_attribute_count{};
  std::optional<std::vector<PinAttributeInformation>> m_pinAttributeInformation{};
  uint8_t m_pin_regions_count_minus1{};
  std::vector<PinRegion> m_pinRegions{std::vector<PinRegion>(1U)};
};

// 23090-12: group_mapping()
class GroupMapping {
public:
  [[nodiscard]] constexpr auto gm_group_count() const noexcept;
  [[nodiscard]] auto gm_group_id(size_t i) const -> uint8_t;

  constexpr auto gm_group_count(uint8_t value) noexcept -> auto &;
  auto gm_group_id(size_t i, uint8_t value) -> GroupMapping &;

  friend auto operator<<(std::ostream &stream, const GroupMapping &x) -> std::ostream &;

  auto operator==(const GroupMapping &other) const noexcept -> bool;
  auto operator!=(const GroupMapping &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
      -> GroupMapping;
  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps) const;

private:
  uint8_t m_gm_group_count{};
  std::vector<uint8_t> m_gm_group_id;
};

// 23090-12: vps_miv_extension()
class VpsMivExtension {
public:
  [[nodiscard]] constexpr auto vme_geometry_scale_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto vme_embedded_occupancy_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto vme_occupancy_scale_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto group_mapping() const noexcept -> const GroupMapping &;

  constexpr auto vme_geometry_scale_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto vme_embedded_occupancy_enabled_flag(bool value) noexcept -> auto &;
  auto vme_occupancy_scale_enabled_flag(bool value) noexcept -> VpsMivExtension &;

  [[nodiscard]] constexpr auto group_mapping() noexcept -> GroupMapping &;

  friend auto operator<<(std::ostream &stream, const VpsMivExtension &x) -> std::ostream &;

  constexpr auto operator==(const VpsMivExtension &other) const noexcept;
  constexpr auto operator!=(const VpsMivExtension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
      -> VpsMivExtension;
  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps) const;

private:
  bool m_vme_geometry_scale_enabled_flag{};
  bool m_vme_embedded_occupancy_enabled_flag{};
  bool m_vme_occupancy_scale_enabled_flag{};
  GroupMapping m_group_mapping;
};

// 23090-12: vps_miv_2_extension()
class VpsMiv2Extension {
public:
  [[nodiscard]] constexpr auto vps_miv_extension() const noexcept -> const auto &;
  [[nodiscard]] constexpr auto vme_decoder_side_depth_estimation_flag() const noexcept;

  [[nodiscard]] constexpr auto vps_miv_extension() noexcept -> auto &;
  constexpr auto vme_decoder_side_depth_estimation_flag(bool value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const VpsMiv2Extension &x) -> std::ostream &;

  constexpr auto operator==(const VpsMiv2Extension &other) const noexcept;
  constexpr auto operator!=(const VpsMiv2Extension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
      -> VpsMiv2Extension;
  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps) const;

private:
  VpsMivExtension m_vps_miv_extension;
  bool m_vme_decoder_side_depth_estimation_flag{};
};

// 23090-5 + m59327: vps_packed_video_extension( )
class VpsPackedVideoExtension {
public:
  [[nodiscard]] auto vps_packed_video_present_flag(AtlasId atlasId) const -> bool;
  [[nodiscard]] auto packing_information(AtlasId atlasId) const -> const PackingInformation &;

  auto vps_packed_video_present_flag(AtlasId atlasId, bool value) -> VpsPackedVideoExtension &;
  auto packing_information(AtlasId atlasId) -> PackingInformation &;

  friend auto operator<<(std::ostream &stream, const VpsPackedVideoExtension &x) -> std::ostream &;

  auto operator==(const VpsPackedVideoExtension &other) const -> bool;
  auto operator!=(const VpsPackedVideoExtension &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &stream, const V3cParameterSet &vps)
      -> VpsPackedVideoExtension;

  void encodeTo(Common::OutputBitstream &stream, const V3cParameterSet &vps) const;

private:
  Common::FlatMap<AtlasId, std::optional<PackingInformation>> m_packing_information;
};

using VpsExtensionDataBytes = std::vector<uint8_t>;

// 23090-5 + m59327: vps_extension( extension_type, extension_length )
class VpsExtension {
public:
  [[nodiscard]] auto vps_packed_video_extension() const -> const VpsPackedVideoExtension &;
  [[nodiscard]] auto vps_miv_extension() const -> const VpsMivExtension &;
  [[nodiscard]] auto vps_miv_2_extension() const -> const VpsMiv2Extension &;
  [[nodiscard]] auto vps_extension_data_byte() const -> const VpsExtensionDataBytes &;

  auto vps_packed_video_extension() -> VpsPackedVideoExtension &;
  auto vps_miv_extension() -> VpsMivExtension &;
  auto vps_miv_2_extension() -> VpsMiv2Extension &;
  [[nodiscard]] auto vps_extension_data_byte() -> VpsExtensionDataBytes &;

  friend auto operator<<(std::ostream &stream, const VpsExtension &x) -> std::ostream &;

  auto operator==(const VpsExtension &other) const -> bool;
  auto operator!=(const VpsExtension &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &stream, VpsExtensionType extensionType,
                         uint16_t extensionLength, const V3cParameterSet &vps) -> VpsExtension;

  [[nodiscard]] auto extensionLength(const V3cParameterSet &vps) const -> uint16_t;
  void encodeTo(Common::OutputBitstream &stream, VpsExtensionType extensionType,
                uint16_t extensionLength, const V3cParameterSet &vps) const;

private:
  std::variant<VpsExtensionDataBytes,   // VPS_EXT_UNSPECIFIED (0), Reserved (4..255)
               VpsPackedVideoExtension, // VPS_EXT_PACKED (1)
               VpsMivExtension,         // VPS_EXT_MIV (2)
               VpsMiv2Extension>        // VPS_EXT_MIV_2 (3)
      m_extension;
};

// 23090-5: v3c_parameter_set()
class V3cParameterSet {
public:
  [[nodiscard]] auto profile_tier_level() const noexcept -> const ProfileTierLevel &;
  [[nodiscard]] constexpr auto vps_v3c_parameter_set_id() const noexcept;
  [[nodiscard]] auto vps_atlas_count_minus1() const noexcept -> uint8_t;
  [[nodiscard]] auto vps_atlas_id(size_t k) const -> AtlasId;

  [[nodiscard]] auto vps_frame_width(AtlasId j) const -> int32_t;
  [[nodiscard]] auto vps_frame_height(AtlasId j) const -> int32_t;
  [[nodiscard]] auto vps_map_count_minus1(AtlasId j) const -> uint8_t;
  [[nodiscard]] auto vps_multiple_map_streams_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_map_absolute_coding_enabled_flag(AtlasId j, uint8_t i) const -> bool;
  [[nodiscard]] auto vps_map_predictor_index_diff(AtlasId j, uint8_t i) const -> uint8_t;
  [[nodiscard]] auto vps_auxiliary_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_occupancy_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_geometry_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_attribute_video_present_flag(AtlasId j) const -> bool;

  [[nodiscard]] auto occupancy_information(AtlasId j) const -> const OccupancyInformation &;
  [[nodiscard]] auto geometry_information(AtlasId j) const -> const GeometryInformation &;
  [[nodiscard]] auto attribute_information(AtlasId j) const -> const AttributeInformation &;

  [[nodiscard]] auto vps_extension_present_flag() const noexcept -> bool;
  [[nodiscard]] auto vps_extension_count() const noexcept -> uint8_t;
  [[nodiscard]] auto vps_extensions_length_minus1() const -> uint32_t;
  [[nodiscard]] auto vps_extension_type(uint8_t n) const -> VpsExtensionType;
  [[nodiscard]] auto vps_extension_length(uint8_t n) const -> uint16_t;
  [[nodiscard]] auto vps_extension(VpsExtensionType vet) const -> const VpsExtension &;

  [[nodiscard]] auto vpsPackingInformationPresentFlag() const -> bool;
  [[nodiscard]] auto vpsMivExtensionPresentFlag() const -> bool;
  [[nodiscard]] auto vpsMiv2ExtensionPresentFlag() const -> bool;
  [[nodiscard]] auto vpsMivOrMiv2ExtensionPresentFlag() const -> bool;

  [[nodiscard]] auto vps_packed_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto packing_information(AtlasId j) const -> const PackingInformation &;
  [[nodiscard]] auto vps_miv_extension() const -> const VpsMivExtension &;
  [[nodiscard]] auto vps_miv_2_extension() const -> const VpsMiv2Extension &;

  auto profile_tier_level(ProfileTierLevel value) noexcept -> V3cParameterSet &;
  constexpr auto vps_v3c_parameter_set_id(uint8_t value) noexcept -> auto &;
  auto vps_atlas_count_minus1(uint8_t value) -> V3cParameterSet &;
  auto vps_atlas_id(size_t k, AtlasId value) -> V3cParameterSet &;

  auto vps_frame_width(AtlasId j, int32_t value) -> V3cParameterSet &;
  auto vps_frame_height(AtlasId j, int32_t value) -> V3cParameterSet &;
  auto vps_map_count_minus1(AtlasId j, uint8_t value) -> V3cParameterSet &;
  auto vps_multiple_map_streams_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_map_absolute_coding_enabled_flag(AtlasId j, uint8_t i, bool value) -> V3cParameterSet &;
  auto vps_map_predictor_index_diff(AtlasId j, uint8_t i, uint8_t value) -> V3cParameterSet &;
  auto vps_auxiliary_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_occupancy_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_geometry_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_attribute_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;

  auto occupancy_information(AtlasId j, OccupancyInformation value) -> V3cParameterSet &;
  auto geometry_information(AtlasId j, GeometryInformation value) -> V3cParameterSet &;
  auto attribute_information(AtlasId j, AttributeInformation value) -> V3cParameterSet &;

  auto vps_extension(VpsExtensionType vet) -> VpsExtension &;
  void removeVpsExtension(VpsExtensionType vet);

  auto vps_packed_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto packing_information(AtlasId j, PackingInformation value) -> V3cParameterSet &;
  auto vps_miv_extension(const VpsMivExtension &value) -> V3cParameterSet &;
  auto vps_miv_2_extension(const VpsMiv2Extension &value) -> V3cParameterSet &;

  constexpr auto profile_tier_level() noexcept -> auto &;
  [[nodiscard]] auto occupancy_information(AtlasId j) -> OccupancyInformation &;
  [[nodiscard]] auto geometry_information(AtlasId j) -> GeometryInformation &;
  [[nodiscard]] auto attribute_information(AtlasId j) -> AttributeInformation &;
  [[nodiscard]] auto vps_miv_extension() -> VpsMivExtension &;
  [[nodiscard]] auto vps_miv_2_extension() -> VpsMiv2Extension &;

  // Convenience function
  [[nodiscard]] auto indexOf(AtlasId atlasId) const -> size_t;

  // Convenience function
  [[nodiscard]] auto attrIdxOf(AtlasId atlasId, AiAttributeTypeId attrTypeId) const
      -> std::optional<uint8_t>;

  [[nodiscard]] auto attrCount(AtlasId atlasId) const;
  [[nodiscard]] auto attrTypeId(AtlasId atlasId, uint8_t attrIdx) const;
  [[nodiscard]] auto attr2dBitDepthMinus1(AtlasId atlasId, uint8_t attrIdx) const;

  friend auto operator<<(std::ostream &stream, const V3cParameterSet &x) -> std::ostream &;

  [[nodiscard]] auto summary() const -> std::string;

  auto operator==(const V3cParameterSet &other) const -> bool;
  auto operator!=(const V3cParameterSet &other) const -> bool;

  static auto decodeFrom(std::istream &stream) -> V3cParameterSet;

  void calculateExtensionLengths();
  void encodeTo(std::ostream &stream) const;

private:
  struct VpsMap {
    bool vps_map_absolute_coding_enabled_flag{true};
    uint8_t vps_map_predictor_index_diff{};
  };

  struct VpsAtlas {
    AtlasId vps_atlas_id{};
    int32_t vps_frame_width{};
    int32_t vps_frame_height{};
    uint8_t vps_map_count_minus1{};
    std::optional<bool> vps_multiple_map_streams_present_flag{};
    std::vector<VpsMap> m_vpsMaps{{}};
    bool vps_auxiliary_video_present_flag{};
    bool vps_occupancy_video_present_flag{};
    bool vps_geometry_video_present_flag{};
    bool vps_attribute_video_present_flag{};
    std::optional<OccupancyInformation> occupancy_information{};
    std::optional<GeometryInformation> geometry_information{};
    std::optional<AttributeInformation> attribute_information{};
  };

  [[nodiscard]] auto atlas(AtlasId atlasId) const -> const VpsAtlas &;
  [[nodiscard]] auto atlas(AtlasId atlasId) -> VpsAtlas &;
  [[nodiscard]] auto map(AtlasId atlasId, uint8_t mapIdx) const -> const VpsMap &;
  [[nodiscard]] auto map(AtlasId atlasId, uint8_t mapIdx) -> VpsMap &;
  [[nodiscard]] auto isAtlasEqual(const V3cParameterSet &other, AtlasId j) const -> bool;
  [[nodiscard]] auto areExtensionsEqual(const V3cParameterSet &other) const -> bool;

  ProfileTierLevel m_profile_tier_level;
  uint8_t m_vps_v3c_parameter_set_id{};
  std::vector<VpsAtlas> m_vpsAtlases{{}};

  uint8_t m_vps_extension_count{};
  std::optional<uint32_t> m_vps_extensions_length_minus1{};
  std::vector<VpsExtensionType> m_vps_extension_type;
  std::vector<uint16_t> m_vps_extension_length;
  std::vector<VpsExtension> m_vps_extension;
};
} // namespace TMIV::MivBitstream

#include "V3cParameterSet.hpp"

#endif
