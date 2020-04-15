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

#ifndef _TMIV_MIVBITSTREAM_VPCCPARAMETERSET_H_
#define _TMIV_MIVBITSTREAM_VPCCPARAMETERSET_H_

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
  MP4RA
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

// 23090-5: profile_tier_level()
class ProfileTierLevel {
public:
  [[nodiscard]] constexpr auto ptl_tier_flag() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_codec_group_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_pcc_toolset_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_reconstruction_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_level_idc() const noexcept;

  constexpr auto ptl_tier_flag(bool value) noexcept -> auto &;
  constexpr auto ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_pcc_toolset_idc(PtlProfilePccToolsetIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc value) noexcept
      -> auto &;
  constexpr auto ptl_level_idc(PtlLevelIdc value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const ProfileTierLevel &x) -> std::ostream &;

  auto operator==(const ProfileTierLevel &other) const noexcept -> bool;
  auto operator!=(const ProfileTierLevel &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ProfileTierLevel;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_ptl_tier_flag{};
  PtlProfileCodecGroupIdc m_ptl_profile_codec_group_idc{};
  PtlProfilePccToolsetIdc m_ptl_profile_pcc_toolset_idc{};
  PtlProfileReconstructionIdc m_ptl_profile_reconstruction_idc{};
  PtlLevelIdc m_ptl_level_idc{};
};

// 23090-5: occupancy_information( atlasId )
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

  auto printTo(std::ostream &stream, std::uint8_t atlasId) const -> std::ostream &;

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

class VpccParameterSet;

// 23090-5: geometry_information( atlasId )
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

  auto printTo(std::ostream &stream, std::uint8_t atlasId) const -> std::ostream &;

  auto operator==(const GeometryInformation &other) const noexcept -> bool;
  auto operator!=(const GeometryInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const VpccParameterSet &vps,
                         uint8_t atlasId) -> GeometryInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const VpccParameterSet &vps,
                uint8_t atlasId) const;

private:
  std::uint8_t m_gi_geometry_codec_id{};
  std::uint8_t m_gi_geometry_nominal_2d_bitdepth_minus1{};
  bool m_gi_geometry_MSB_align_flag{};
  std::uint8_t m_gi_geometry_3d_coordinates_bitdepth_minus1{};
};

// 23090-5: attribute_information( atlasId )
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

  auto printTo(std::ostream &stream, std::uint8_t atlasId) const -> std::ostream &;

  auto operator==(const AttributeInformation &other) const noexcept -> bool;
  auto operator!=(const AttributeInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const VpccParameterSet &vps,
                         std::uint8_t atlasId) -> AttributeInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const VpccParameterSet &vps,
                std::uint8_t atlasId) const;

private:
  struct AiAttribute {
    AiAttributeTypeId ai_attribute_type_id{};
    std::uint8_t ai_attribute_codec_id{};
    std::optional<bool> ai_attribute_map_absolute_coding_persistence_flag{};
    std::uint8_t ai_attribute_dimension_minus1{};
    std::uint8_t ai_attribute_nominal_2d_bitdepth_minus1{};
    bool ai_attribute_MSB_align_flag{};
  };

  std::vector<AiAttribute> m_ai_attributes; // 23090-5: ai_attribute_count
};

// 23090-12: miv_sequence_params()
class MivSequenceParams {
public:
  [[nodiscard]] constexpr auto msp_depth_low_quality_flag() const noexcept;
  [[nodiscard]] constexpr auto msp_geometry_scale_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto msp_num_groups_minus1() const noexcept;
  [[nodiscard]] constexpr auto msp_max_entities_minus1() const noexcept;
  [[nodiscard]] inline auto msp_fully_occupied_flag(std::uint8_t atlasIndex) const noexcept;
  [[nodiscard]] inline auto
  msp_occupancy_subbitstream_present_flag(std::uint8_t atlasIndex) const noexcept;

  constexpr auto msp_depth_low_quality_flag(const bool value) noexcept -> auto &;
  constexpr auto msp_geometry_scale_enabled_flag(const bool value) noexcept -> auto &;
  constexpr auto msp_num_groups_minus1(const unsigned value) noexcept -> auto &;
  constexpr auto msp_max_entities_minus1(const unsigned value) noexcept -> auto &;
  inline auto msp_fully_occupied_flag(std::uint8_t atlasIndex, const bool value) noexcept
      -> auto &;
  inline auto
  msp_occupancy_subbitstream_present_flag(std::uint8_t atlasIndex, const bool value) noexcept
      -> auto &;

  inline void allocateFlagVectors(std::uint8_t size);

  inline void insertFlagVectors(const MivSequenceParams &other);

  friend auto operator<<(std::ostream &stream, const MivSequenceParams &x) -> std::ostream &;

  constexpr auto operator==(const MivSequenceParams &other) const noexcept;
  constexpr auto operator!=(const MivSequenceParams &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, std::uint8_t vps_atlas_count_minus1)
      -> MivSequenceParams;

  void encodeTo(Common::OutputBitstream &bitstream, std::uint8_t vps_atlas_count_minus1) const;

private:
  bool m_msp_depth_low_quality_flag{};
  bool m_msp_geometry_scale_enabled_flag{};
  unsigned m_msp_num_groups_minus1{};
  unsigned m_msp_max_entities_minus1{};
  std::vector<bool> m_msp_fully_occupied_flag{};
  std::vector<bool> m_msp_occupancy_subbitstream_present_flag{};
};

// 23090-5: vpcc_parameter_set()
class VpccParameterSet {
public:
  [[nodiscard]] constexpr auto profile_tier_level() const noexcept;
  [[nodiscard]] constexpr auto vps_vpcc_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto vps_miv_mode_flag() const noexcept;
  [[nodiscard]] auto vps_atlas_count_minus1() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vps_frame_width(std::uint8_t atlasId) const -> std::uint16_t;
  [[nodiscard]] auto vps_frame_height(std::uint8_t atlasId) const -> std::uint16_t;
  [[nodiscard]] auto vps_map_count_minus1(std::uint8_t atlasId) const -> std::uint8_t;
  [[nodiscard]] auto occupancy_information(std::uint8_t atlasId) const
      -> const OccupancyInformation &;
  [[nodiscard]] auto geometry_information(std::uint8_t atlasId) const
      -> const GeometryInformation &;
  [[nodiscard]] auto attribute_information(std::uint8_t atlasId) const
      -> const AttributeInformation &;
  [[nodiscard]] auto vps_auxiliary_video_present_flag(std::uint8_t atlasId) const -> bool;
  [[nodiscard]] constexpr auto vps_extension_present_flag() const noexcept; // Only 23090-5
  [[nodiscard]] auto vps_miv_extension_flag() const noexcept -> bool;
  [[nodiscard]] auto miv_sequence_params() const noexcept -> const MivSequenceParams &;
  [[nodiscard]] auto vps_miv_sequence_vui_params_present_flag() const noexcept -> bool;
  [[nodiscard]] auto miv_vui_params() const noexcept -> const MivVuiParams &;

  constexpr auto profile_tier_level(ProfileTierLevel value) noexcept -> auto &;
  constexpr auto vps_vpcc_parameter_set_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto vps_miv_mode_flag(const bool value) noexcept -> auto &;
  auto vps_atlas_count_minus1(std::uint8_t value) -> VpccParameterSet &;
  auto vps_frame_width(std::uint8_t atlasId, std::uint16_t value) -> VpccParameterSet &;
  auto vps_frame_height(std::uint8_t atlasId, std::uint16_t value) -> VpccParameterSet &;
  auto vps_map_count_minus1(std::uint8_t atlasId, std::uint8_t value) -> VpccParameterSet &;
  auto occupancy_information(std::uint8_t atlasId, OccupancyInformation value)
      -> VpccParameterSet &;
  auto geometry_information(std::uint8_t atlasId, GeometryInformation value) -> VpccParameterSet &;
  auto attribute_information(std::uint8_t atlasId, AttributeInformation value)
      -> VpccParameterSet &;
  auto vps_auxiliary_video_present_flag(std::uint8_t atlasId, bool value) -> VpccParameterSet &;
  constexpr auto vps_extension_present_flag(bool value) noexcept -> auto &; // Only 23090-5
  auto vps_miv_extension_flag(bool value) noexcept -> VpccParameterSet &;
  auto vps_miv_sequence_vui_params_present_flag(bool value) noexcept -> VpccParameterSet &;

  constexpr auto profile_tier_level() noexcept -> auto &;
  [[nodiscard]] auto occupancy_information(std::uint8_t atlasId) -> OccupancyInformation &;
  [[nodiscard]] auto geometry_information(std::uint8_t atlasId) -> GeometryInformation &;
  [[nodiscard]] auto attribute_information(std::uint8_t atlasId) -> AttributeInformation &;
  [[nodiscard]] auto miv_sequence_params() noexcept -> MivSequenceParams &;
  [[nodiscard]] auto miv_vui_params() noexcept -> MivVuiParams &;

  friend auto operator<<(std::ostream &stream, const VpccParameterSet &x) -> std::ostream &;

  auto operator==(const VpccParameterSet &other) const noexcept -> bool;
  auto operator!=(const VpccParameterSet &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> VpccParameterSet;

  void encodeTo(std::ostream &stream) const;

  friend auto merge(const std::vector<const VpccParameterSet *> &vps) -> VpccParameterSet;

private:
  struct VpsAtlas {
    std::uint16_t vps_frame_width{};
    std::uint16_t vps_frame_height{};
    std::uint8_t vps_map_count_minus1{};
    OccupancyInformation occupancy_information;
    GeometryInformation geometry_information;
    AttributeInformation attribute_information;
    bool vps_auxiliary_video_present_flag{};
  };

  ProfileTierLevel m_profile_tier_level;
  std::uint8_t m_vps_vpcc_parameter_set_id{};
  bool m_vps_miv_mode_flag{};
  std::vector<VpsAtlas> m_vps_atlases{VpsAtlas{}};
  bool m_vps_extension_present_flag{};
  bool m_vps_miv_extension_flag{};
  std::optional<MivSequenceParams> m_miv_sequence_params;
  std::optional<bool> m_vps_miv_sequence_vui_params_present_flag;
  std::optional<MivVuiParams> m_miv_vui_params;
};
} // namespace TMIV::MivBitstream

#include "VpccParameterSet.hpp"

#endif
