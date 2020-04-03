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

#include <TMIV/MivBitstream/VpccParameterSet.h>

#include <TMIV/MivBitstream/MivDecoder.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const PtlProfileCodecGroupIdc &x) -> ostream & {
  switch (x) {
  case PtlProfileCodecGroupIdc::AVC_Progressive_High:
    return stream << "AVC Progressive High";
  case PtlProfileCodecGroupIdc::HEVC_Main10:
    return stream << "HEVC Main10";
  case PtlProfileCodecGroupIdc::HEVC444:
    return stream << "HEVC444";
  case PtlProfileCodecGroupIdc::MP4RA:
    return stream << "MP4RA";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(ostream &stream, const PtlProfilePccToolsetIdc &x) -> ostream & {
  switch (x) {
  case PtlProfilePccToolsetIdc::Basic:
    return stream << "Basic";
  case PtlProfilePccToolsetIdc::Extended:
    return stream << "Extended";
  case PtlProfilePccToolsetIdc::MIV_Main:
    return stream << "MIV Main";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(ostream &stream, const PtlProfileReconstructionIdc &x) -> ostream & {
  switch (x) {
  case PtlProfileReconstructionIdc::Rec0:
    return stream << "Rec0";
  case PtlProfileReconstructionIdc::Rec1:
    return stream << "Rec1";
  case PtlProfileReconstructionIdc::Unconstrained:
    return stream << "Unconstrained";
  case PtlProfileReconstructionIdc::MIV_Main:
    return stream << "MIV Main";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(ostream &stream, const PtlLevelIdc &x) -> ostream & {
  switch (x) {
  case PtlLevelIdc::Level_1_0:
    return stream << "Level 1.0";
  case PtlLevelIdc::Level_2_0:
    return stream << "Level 2.0";
  case PtlLevelIdc::Level_3_0:
    return stream << "Level 3.0";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(ostream &stream, const AiAttributeTypeId &x) -> ostream & {
  switch (x) {
  case AiAttributeTypeId::ATTR_TEXTURE:
    return stream << "ATTR_TEXTURE";
  case AiAttributeTypeId::ATTR_MATERIAL_ID:
    return stream << "ATTR_MATERIAL_ID";
  case AiAttributeTypeId::ATTR_TRANSPARENCY:
    return stream << "ATTR_TRANSPARENCY";
  case AiAttributeTypeId::ATTR_REFLECTANCE:
    return stream << "ATTR_REFLECTANCE";
  case AiAttributeTypeId::ATTR_NORMAL:
    return stream << "ATTR_NORMAL";
  case AiAttributeTypeId::ATTR_UNSPECIFIED:
    return stream << "ATTR_UNSPECIFIED";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(ostream &stream, const ProfileTierLevel &x) -> ostream & {
  return stream << "ptl_tier_flag=" << boolalpha << x.ptl_tier_flag()
                << "\nptl_profile_codec_group_idc=" << x.ptl_profile_codec_group_idc()
                << "\nptl_profile_pcc_toolset_idc=" << x.ptl_profile_pcc_toolset_idc()
                << "\nptl_profile_reconstruction_idc=" << x.ptl_profile_reconstruction_idc()
                << "\nptl_level_idc=" << x.ptl_level_idc() << '\n';
}

auto ProfileTierLevel::operator==(const ProfileTierLevel &other) const noexcept -> bool {
  return ptl_tier_flag() == other.ptl_tier_flag() &&
         ptl_profile_codec_group_idc() == other.ptl_profile_codec_group_idc() &&
         ptl_profile_pcc_toolset_idc() == other.ptl_profile_pcc_toolset_idc() &&
         ptl_profile_reconstruction_idc() == other.ptl_profile_reconstruction_idc() &&
         ptl_level_idc() == other.ptl_level_idc();
}

auto ProfileTierLevel::operator!=(const ProfileTierLevel &other) const noexcept -> bool {
  return !operator==(other);
}

auto ProfileTierLevel::decodeFrom(InputBitstream &bitstream) -> ProfileTierLevel {
  auto x = ProfileTierLevel{};
  x.ptl_tier_flag(bitstream.getFlag());
  x.ptl_profile_codec_group_idc(bitstream.readBits<PtlProfileCodecGroupIdc>(7));
  x.ptl_profile_pcc_toolset_idc(bitstream.readBits<PtlProfilePccToolsetIdc>(8));
  x.ptl_profile_reconstruction_idc(bitstream.readBits<PtlProfileReconstructionIdc>(8));
  bitstream.getUint32();
  x.ptl_level_idc(bitstream.readBits<PtlLevelIdc>(8));
  return x;
}

void ProfileTierLevel::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFlag(ptl_tier_flag());
  bitstream.writeBits(ptl_profile_codec_group_idc(), 7);
  bitstream.writeBits(ptl_profile_pcc_toolset_idc(), 8);
  bitstream.writeBits(ptl_profile_reconstruction_idc(), 8);
  bitstream.putUint32(0);
  bitstream.writeBits(ptl_level_idc(), 8);
}

auto OccupancyInformation::printTo(ostream &stream, uint8_t atlasId) const -> ostream & {
  return stream << "oi_occupancy_codec_id( " << int(atlasId)
                << " )=" << int(oi_occupancy_codec_id())
                << "\noi_lossy_occupancy_map_compression_threshold( " << int(atlasId)
                << " )=" << int(oi_lossy_occupancy_map_compression_threshold())
                << "\noi_occupancy_nominal_2d_bitdepth_minus1( " << int(atlasId)
                << " )=" << int(oi_occupancy_nominal_2d_bitdepth_minus1())
                << "\noi_occupancy_MSB_align_flag( " << int(atlasId) << " )=" << boolalpha
                << oi_occupancy_MSB_align_flag() << '\n';
}

auto OccupancyInformation::operator==(const OccupancyInformation &other) const noexcept -> bool {
  return oi_occupancy_codec_id() == other.oi_occupancy_codec_id() &&
         oi_lossy_occupancy_map_compression_threshold() ==
             other.oi_lossy_occupancy_map_compression_threshold() &&
         oi_occupancy_nominal_2d_bitdepth_minus1() ==
             other.oi_occupancy_nominal_2d_bitdepth_minus1() &&
         oi_occupancy_MSB_align_flag() == other.oi_occupancy_MSB_align_flag();
}

auto OccupancyInformation::operator!=(const OccupancyInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto OccupancyInformation::decodeFrom(InputBitstream &bitstream) -> OccupancyInformation {
  auto x = OccupancyInformation{};
  x.oi_occupancy_codec_id(bitstream.getUint8());
  x.oi_lossy_occupancy_map_compression_threshold(bitstream.getUint8());
  x.oi_occupancy_nominal_2d_bitdepth_minus1(bitstream.readBits<uint8_t>(5));
  x.oi_occupancy_MSB_align_flag(bitstream.getFlag());
  return x;
}

void OccupancyInformation::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint8(oi_occupancy_codec_id());
  bitstream.putUint8(oi_lossy_occupancy_map_compression_threshold());
  bitstream.writeBits(oi_occupancy_nominal_2d_bitdepth_minus1(), 5);
  bitstream.putFlag(oi_occupancy_MSB_align_flag());
}

auto GeometryInformation::printTo(ostream &stream, uint8_t atlasId) const -> ostream & {
  return stream << "gi_geometry_codec_id( " << int(atlasId) << " )=" << int(gi_geometry_codec_id())
                << "\ngi_geometry_nominal_2d_bitdepth_minus1( " << int(atlasId)
                << " )=" << int(gi_geometry_nominal_2d_bitdepth_minus1())
                << "\ngi_geometry_MSB_align_flag( " << int(atlasId) << " )=" << boolalpha
                << gi_geometry_MSB_align_flag() << "\ngi_geometry_3d_coordinates_bitdepth_minus1( "
                << int(atlasId) << " )=" << int(gi_geometry_3d_coordinates_bitdepth_minus1())
                << '\n';
}

auto GeometryInformation::operator==(const GeometryInformation &other) const noexcept -> bool {
  return gi_geometry_codec_id() == other.gi_geometry_codec_id() &&
         gi_geometry_nominal_2d_bitdepth_minus1() ==
             other.gi_geometry_nominal_2d_bitdepth_minus1() &&
         gi_geometry_MSB_align_flag() == other.gi_geometry_MSB_align_flag() &&
         gi_geometry_3d_coordinates_bitdepth_minus1() ==
             other.gi_geometry_3d_coordinates_bitdepth_minus1();
}

auto GeometryInformation::operator!=(const GeometryInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto GeometryInformation::decodeFrom(InputBitstream &bitstream, const VpccParameterSet &vps,
                                     uint8_t atlasId) -> GeometryInformation {
  auto x = GeometryInformation{};
  x.gi_geometry_codec_id(bitstream.getUint8());
  x.gi_geometry_nominal_2d_bitdepth_minus1(bitstream.readBits<uint8_t>(5));
  x.gi_geometry_MSB_align_flag(bitstream.getFlag());
  x.gi_geometry_3d_coordinates_bitdepth_minus1(bitstream.readBits<uint8_t>(5));
  VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasId));
  return x;
}

void GeometryInformation::encodeTo(OutputBitstream &bitstream, const VpccParameterSet &vps,
                                   uint8_t atlasId) const {
  bitstream.putUint8(gi_geometry_codec_id());
  bitstream.writeBits(gi_geometry_nominal_2d_bitdepth_minus1(), 5);
  bitstream.putFlag(gi_geometry_MSB_align_flag());
  bitstream.writeBits(gi_geometry_3d_coordinates_bitdepth_minus1(), 5);
  VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasId));
}

auto AttributeInformation::ai_attribute_count() const noexcept -> uint8_t {
  return uint8_t(m_ai_attributes.size());
}

auto AttributeInformation::ai_attribute_type_id(uint8_t attributeId) const -> AiAttributeTypeId {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  return m_ai_attributes[attributeId].ai_attribute_type_id;
}

auto AttributeInformation::ai_attribute_codec_id(uint8_t attributeId) const -> uint8_t {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  return m_ai_attributes[attributeId].ai_attribute_codec_id;
}

auto AttributeInformation::ai_attribute_dimension_minus1(uint8_t attributeId) const -> uint8_t {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  return m_ai_attributes[attributeId].ai_attribute_dimension_minus1;
}

auto AttributeInformation::ai_attribute_nominal_2d_bitdepth_minus1(uint8_t attributeId) const
    -> uint8_t {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  return m_ai_attributes[attributeId].ai_attribute_nominal_2d_bitdepth_minus1;
}

auto AttributeInformation::ai_attribute_MSB_align_flag() const noexcept -> bool {
  VERIFY_VPCCBITSTREAM(1 <= ai_attribute_count());
  return m_ai_attribute_MSB_align_flag;
}

auto AttributeInformation::ai_attribute_count(std::uint8_t value) -> AttributeInformation & {
  m_ai_attributes.resize(value);
  return *this;
}

auto AttributeInformation::ai_attribute_type_id(uint8_t attributeId, AiAttributeTypeId value)
    -> AttributeInformation & {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  m_ai_attributes[attributeId].ai_attribute_type_id = value;
  return *this;
}

auto AttributeInformation::ai_attribute_codec_id(uint8_t attributeId, uint8_t value)
    -> AttributeInformation & {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  m_ai_attributes[attributeId].ai_attribute_codec_id = value;
  return *this;
}

auto AttributeInformation::ai_attribute_dimension_minus1(uint8_t attributeId, uint8_t value)
    -> AttributeInformation & {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  m_ai_attributes[attributeId].ai_attribute_dimension_minus1 = value;
  return *this;
}

auto AttributeInformation::ai_attribute_nominal_2d_bitdepth_minus1(uint8_t attributeId,
                                                                   uint8_t value)
    -> AttributeInformation & {
  VERIFY_VPCCBITSTREAM(attributeId < ai_attribute_count());
  m_ai_attributes[attributeId].ai_attribute_nominal_2d_bitdepth_minus1 = value;
  return *this;
}

auto AttributeInformation::ai_attribute_MSB_align_flag(bool value) noexcept
    -> AttributeInformation & {
  VERIFY_VPCCBITSTREAM(1 <= ai_attribute_count());
  m_ai_attribute_MSB_align_flag = value;
  return *this;
}

auto AttributeInformation::printTo(ostream &stream, uint8_t atlasId) const -> ostream & {
  stream << "ai_attribute_count( " << int(atlasId) << " )=" << int(ai_attribute_count());
  if (1 <= ai_attribute_count()) {
    stream << "\nai_attribute_MSB_align_flag( " << int(atlasId) << " )=" << boolalpha
           << ai_attribute_MSB_align_flag();
  }
  for (auto i = 0; i < ai_attribute_count(); ++i) {
    stream << "\nai_attribute_type_id( " << int(atlasId) << ", " << i
           << " )=" << ai_attribute_type_id(i) << "\nai_attribute_codec_id( " << int(atlasId)
           << ", " << i << " )=" << int(ai_attribute_codec_id(i))
           << "\nai_attribute_dimension_minus1( " << int(atlasId) << ", " << i
           << " )=" << int(ai_attribute_dimension_minus1(i))
           << "\nai_attribute_nominal_2d_bitdepth_minus1( " << int(atlasId) << ", " << i
           << " )=" << int(ai_attribute_nominal_2d_bitdepth_minus1(i));
  }
  return stream << '\n';
}

auto AttributeInformation::operator==(const AttributeInformation &other) const noexcept -> bool {
  if (ai_attribute_count() != other.ai_attribute_count()) {
    return false;
  }
  if (ai_attribute_count() == 0) {
    return true;
  }
  if (ai_attribute_MSB_align_flag() != other.ai_attribute_MSB_align_flag()) {
    return false;
  }
  for (auto i = 0; i < ai_attribute_count(); ++i) {
    if (ai_attribute_type_id(i) != other.ai_attribute_type_id(i) ||
        ai_attribute_codec_id(i) != other.ai_attribute_codec_id(i) ||
        ai_attribute_dimension_minus1(i) != other.ai_attribute_dimension_minus1(i) ||
        ai_attribute_nominal_2d_bitdepth_minus1(i) !=
            other.ai_attribute_nominal_2d_bitdepth_minus1(i)) {
      return false;
    }
  }
  return true;
}

auto AttributeInformation::operator!=(const AttributeInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto AttributeInformation::decodeFrom(InputBitstream &bitstream, const VpccParameterSet & /*vps*/,
                                      uint8_t /*atlasId*/) -> AttributeInformation {
  auto x = AttributeInformation{};
  x.ai_attribute_count(bitstream.readBits<uint8_t>(7));
  for (auto i = 0; i < x.ai_attribute_count(); ++i) {
    x.ai_attribute_type_id(i, bitstream.readBits<AiAttributeTypeId>(4));
    x.ai_attribute_codec_id(i, bitstream.getUint8());
    x.ai_attribute_dimension_minus1(i, bitstream.readBits<uint8_t>(6));

    const auto ai_attribute_dimension_partitions_minus1 = bitstream.readBits<uint8_t>(6);
    VERIFY_MIVBITSTREAM(ai_attribute_dimension_partitions_minus1 == 0);

    x.ai_attribute_nominal_2d_bitdepth_minus1(i, bitstream.readBits<uint8_t>(5));
  }
  if (0 < x.ai_attribute_count()) {
    x.ai_attribute_MSB_align_flag(bitstream.getFlag());
  }
  return x;
}

void AttributeInformation::encodeTo(OutputBitstream &bitstream, const VpccParameterSet &vps,
                                    uint8_t atlasId) const {
  bitstream.writeBits(ai_attribute_count(), 7);
  if (ai_attribute_count() == 0) {
    return;
  }
  for (auto i = 0; i < ai_attribute_count(); ++i) {
    bitstream.writeBits(ai_attribute_type_id(i), 4);
    bitstream.writeBits(ai_attribute_codec_id(i), 8);

    VERIFY_MIVBITSTREAM(vps.vps_map_count_minus1(atlasId) == 0);

    VERIFY_VPCCBITSTREAM(ai_attribute_dimension_minus1(i) < 64);
    bitstream.writeBits(ai_attribute_dimension_minus1(i), 6);

    constexpr auto ai_attribute_dimension_partitions_minus1 = 0;
    bitstream.writeBits(ai_attribute_dimension_partitions_minus1, 6);

    VERIFY_VPCCBITSTREAM(ai_attribute_nominal_2d_bitdepth_minus1(i) < 32);
    bitstream.writeBits(ai_attribute_nominal_2d_bitdepth_minus1(i), 5);
  }
  bitstream.putFlag(ai_attribute_MSB_align_flag());
}

auto operator<<(std::ostream &stream, const MivSequenceParams &x) -> std::ostream & {
  return stream << "msp_depth_low_quality_flag=" << boolalpha << x.msp_depth_low_quality_flag()
                << "\nmsp_geometry_scale_enabled_flag=" << boolalpha
                << x.msp_geometry_scale_enabled_flag()
                << "\nmsp_num_groups_minus1=" << x.msp_num_groups_minus1()
                << "\nmsp_max_entities_minus1=" << x.msp_max_entities_minus1() << '\n';
}

auto MivSequenceParams::decodeFrom(InputBitstream &bitstream) -> MivSequenceParams {
  auto x = MivSequenceParams{};
  x.msp_depth_low_quality_flag(bitstream.getFlag());
  x.msp_geometry_scale_enabled_flag(bitstream.getFlag());
  x.msp_num_groups_minus1(bitstream.getUExpGolomb<unsigned>());
  x.msp_max_entities_minus1(bitstream.getUExpGolomb<unsigned>());
  return x;
}

void MivSequenceParams::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFlag(msp_depth_low_quality_flag());
  bitstream.putFlag(msp_geometry_scale_enabled_flag());
  bitstream.putUExpGolomb(msp_num_groups_minus1());
  bitstream.putUExpGolomb(msp_max_entities_minus1());
}

auto VpccParameterSet::vps_atlas_count_minus1() const noexcept -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(!m_vps_atlases.empty());
  return uint8_t(m_vps_atlases.size() - 1U);
}

auto VpccParameterSet::vps_frame_width(std::uint8_t atlasId) const -> std::uint16_t {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].vps_frame_width;
}

auto VpccParameterSet::vps_frame_height(std::uint8_t atlasId) const -> std::uint16_t {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].vps_frame_height;
}

auto VpccParameterSet::vps_map_count_minus1(std::uint8_t atlasId) const -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].vps_map_count_minus1;
}

auto VpccParameterSet::occupancy_information(std::uint8_t atlasId) const
    -> const OccupancyInformation & {
  VERIFY_VPCCBITSTREAM(!vps_miv_mode_flag() && atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].occupancy_information;
}

auto VpccParameterSet::geometry_information(std::uint8_t atlasId) const
    -> const GeometryInformation & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].geometry_information;
}

auto VpccParameterSet::attribute_information(std::uint8_t atlasId) const
    -> const AttributeInformation & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].attribute_information;
}

auto VpccParameterSet::vps_auxiliary_video_present_flag(std::uint8_t atlasId) const -> bool {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].vps_auxiliary_video_present_flag;
}

auto VpccParameterSet::vps_miv_extension_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(vps_extension_present_flag());
  return m_vps_miv_extension_flag;
}

auto VpccParameterSet::miv_sequence_params() const noexcept -> const MivSequenceParams & {
  VERIFY_MIVBITSTREAM(m_miv_sequence_params.has_value());
  return *m_miv_sequence_params;
}

auto VpccParameterSet::vps_miv_sequence_vui_params_present_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(m_vps_miv_sequence_vui_params_present_flag.has_value());
  return *m_vps_miv_sequence_vui_params_present_flag;
}

auto VpccParameterSet::miv_vui_params() const noexcept -> const MivVuiParams & {
  VERIFY_MIVBITSTREAM(m_miv_vui_params.has_value());
  return *m_miv_vui_params;
}

auto VpccParameterSet::vps_atlas_count_minus1(std::uint8_t value) -> VpccParameterSet & {
  m_vps_atlases.resize(value + 1U);
  return *this;
}

auto VpccParameterSet::vps_frame_width(std::uint8_t atlasId, std::uint16_t value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].vps_frame_width = value;
  return *this;
}

auto VpccParameterSet::vps_frame_height(std::uint8_t atlasId, std::uint16_t value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].vps_frame_height = value;
  return *this;
}

auto VpccParameterSet::vps_map_count_minus1(std::uint8_t atlasId, std::uint8_t value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].vps_map_count_minus1 = value;
  return *this;
}

auto VpccParameterSet::occupancy_information(std::uint8_t atlasId, OccupancyInformation value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(!vps_miv_mode_flag() && atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].occupancy_information = value;
  return *this;
}

auto VpccParameterSet::geometry_information(std::uint8_t atlasId, GeometryInformation value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].geometry_information = value;
  return *this;
}

auto VpccParameterSet::attribute_information(std::uint8_t atlasId, AttributeInformation value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].attribute_information = move(value);
  return *this;
}

auto VpccParameterSet::vps_auxiliary_video_present_flag(std::uint8_t atlasId, bool value)
    -> VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  m_vps_atlases[atlasId].vps_auxiliary_video_present_flag = value;
  return *this;
}

auto VpccParameterSet::vps_miv_extension_flag(bool value) noexcept -> VpccParameterSet & {
  VERIFY_MIVBITSTREAM(vps_extension_present_flag());
  m_vps_miv_extension_flag = value;
  return *this;
}

auto VpccParameterSet::vps_miv_sequence_vui_params_present_flag(bool value) noexcept
    -> VpccParameterSet & {
  VERIFY_MIVBITSTREAM(vps_miv_extension_flag());
  m_vps_miv_sequence_vui_params_present_flag = value;
  return *this;
}

auto VpccParameterSet::occupancy_information(std::uint8_t atlasId) -> OccupancyInformation & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].occupancy_information;
}

auto VpccParameterSet::geometry_information(std::uint8_t atlasId) -> GeometryInformation & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].geometry_information;
}

auto VpccParameterSet::attribute_information(std::uint8_t atlasId) -> AttributeInformation & {
  VERIFY_VPCCBITSTREAM(atlasId <= vps_atlas_count_minus1());
  return m_vps_atlases[atlasId].attribute_information;
}

auto VpccParameterSet::miv_sequence_params() noexcept -> MivSequenceParams & {
  VERIFY_MIVBITSTREAM(vps_miv_extension_flag());
  if (!m_miv_sequence_params) {
    m_miv_sequence_params = MivSequenceParams{};
  }
  return *m_miv_sequence_params;
}

auto VpccParameterSet::miv_vui_params() noexcept -> MivVuiParams & {
  VERIFY_MIVBITSTREAM(vps_miv_sequence_vui_params_present_flag());
  if (!m_miv_vui_params) {
    m_miv_vui_params = MivVuiParams{};
  }
  return *m_miv_vui_params;
}

auto operator<<(ostream &stream, const VpccParameterSet &x) -> ostream & {
  stream << x.profile_tier_level()
         << "vps_vpcc_parameter_set_id=" << int(x.vps_vpcc_parameter_set_id())
         << "\nvps_miv_mode_flag=" << boolalpha << x.vps_miv_mode_flag()
         << "\nvps_atlas_count_minus1=" << int(x.vps_atlas_count_minus1()) << '\n';
  for (int j = 0; j <= x.vps_atlas_count_minus1(); ++j) {
    stream << "vps_frame_width( " << j << " )=" << x.vps_frame_width(j);
    stream << "\nvps_frame_height( " << j << " )=" << x.vps_frame_height(j);
    stream << "\nvps_map_count_minus1( " << j << " )=" << int(x.vps_map_count_minus1(j));
    stream << "\nvps_auxiliary_video_present_flag( " << j << " )=" << boolalpha
           << x.vps_auxiliary_video_present_flag(j) << '\n';
    if (!x.vps_miv_mode_flag()) {
      x.occupancy_information(j).printTo(stream, j);
    }
    x.geometry_information(j).printTo(stream, j);
    x.attribute_information(j).printTo(stream, j);
  }
  stream << "vps_extension_present_flag=" << boolalpha << x.vps_extension_present_flag() << '\n';

  if (x.vps_extension_present_flag()) {
    stream << "vps_miv_extension_flag=" << boolalpha << x.vps_miv_extension_flag() << '\n';

    if (x.vps_miv_extension_flag()) {
      stream << x.miv_sequence_params();
      stream << "vps_miv_sequence_vui_params_present_flag=" << boolalpha
             << x.vps_miv_sequence_vui_params_present_flag() << '\n';
      if (x.vps_miv_sequence_vui_params_present_flag()) {
        stream << x.miv_vui_params();
      }
    }
  }
  return stream;
}

auto VpccParameterSet::operator==(const VpccParameterSet &other) const noexcept -> bool {
  if (profile_tier_level() != other.profile_tier_level() ||
      vps_miv_mode_flag() != other.vps_miv_mode_flag() ||
      vps_atlas_count_minus1() != other.vps_atlas_count_minus1() ||
      vps_extension_present_flag() != other.vps_extension_present_flag() ||
      vps_miv_extension_flag() != other.vps_miv_extension_flag() ||
      m_miv_sequence_params != other.m_miv_sequence_params ||
      m_vps_miv_sequence_vui_params_present_flag !=
          other.m_vps_miv_sequence_vui_params_present_flag ||
      m_miv_vui_params != other.m_miv_vui_params) {
    return false;
  }
  for (int j = 0; j <= vps_atlas_count_minus1(); ++j) {
    if (vps_frame_width(j) != other.vps_frame_width(j) ||
        vps_frame_height(j) != other.vps_frame_height(j) ||
        vps_map_count_minus1(j) != other.vps_map_count_minus1(j) ||
        vps_auxiliary_video_present_flag(j) != other.vps_auxiliary_video_present_flag(j) ||
        (!vps_miv_mode_flag() && occupancy_information(j) != other.occupancy_information(j)) ||
        geometry_information(j) != other.geometry_information(j) ||
        attribute_information(j) != other.attribute_information(j)) {
      return false;
    }
  }
  return true;
}

auto VpccParameterSet::operator!=(const VpccParameterSet &other) const noexcept -> bool {
  return !operator==(other);
}

auto VpccParameterSet::decodeFrom(istream &stream) -> VpccParameterSet {
  auto x = VpccParameterSet{};
  InputBitstream bitstream{stream};

  x.profile_tier_level(ProfileTierLevel::decodeFrom(bitstream));
  x.vps_vpcc_parameter_set_id(bitstream.readBits<uint8_t>(4));

  // TODO(TMC2#33): Add vps_reserved_zero_8bits
  if (MivDecoder::mode != MivDecoder::Mode::TMC2) {
    x.vps_miv_mode_flag(bitstream.getFlag());
    VERIFY_VPCCBITSTREAM(MivDecoder::mode == MivDecoder::Mode::MIV || !x.vps_miv_mode_flag());

    const auto vps_reserved_zero_7bits = bitstream.readBits<uint8_t>(7);
    VERIFY_VPCCBITSTREAM(vps_reserved_zero_7bits == 0);
  }

  x.vps_atlas_count_minus1(bitstream.readBits<uint8_t>(6));

  for (int j = 0; j <= x.vps_atlas_count_minus1(); ++j) {
    x.vps_frame_width(j, bitstream.getUint16());
    x.vps_frame_height(j, bitstream.getUint16());

    x.vps_map_count_minus1(j, bitstream.readBits<uint8_t>(4));

    if (x.vps_map_count_minus1(j) > 0) {
      const auto vps_multiple_map_streams_present_flag = bitstream.getFlag();
      VERIFY_MIVBITSTREAM(!vps_multiple_map_streams_present_flag);
    }

    x.vps_auxiliary_video_present_flag(j, bitstream.getFlag());
    VERIFY_MIVBITSTREAM(!x.vps_auxiliary_video_present_flag(j));

    if (!x.vps_miv_mode_flag()) {
      x.occupancy_information(j, OccupancyInformation::decodeFrom(bitstream));
    }
    x.geometry_information(j, GeometryInformation::decodeFrom(bitstream, x, j));
    x.attribute_information(j, AttributeInformation::decodeFrom(bitstream, x, j));
  }

  x.vps_extension_present_flag(bitstream.getFlag());

  if (MivDecoder::mode == MivDecoder::Mode::MIV) {
    VERIFY_MIVBITSTREAM(x.vps_extension_present_flag());

    const auto vps_extension_length_minus1 = bitstream.getUExpGolomb<size_t>();
    const auto extensionEnd = size_t(bitstream.tellg()) + 8 * (vps_extension_length_minus1 + 1);

    x.vps_miv_extension_flag(bitstream.getFlag());

    if (x.vps_miv_extension_flag()) {
      x.miv_sequence_params() = MivSequenceParams::decodeFrom(bitstream);
      x.vps_miv_sequence_vui_params_present_flag(bitstream.getFlag());

      if (x.vps_miv_sequence_vui_params_present_flag()) {
        x.miv_vui_params() = MivVuiParams::decodeFrom(bitstream);
      }
    }

    while (size_t(bitstream.tellg()) < extensionEnd) {
      bitstream.getFlag(); // vps_miv_extension_data_flag
    }
    VERIFY_MIVBITSTREAM(size_t(bitstream.tellg()) == extensionEnd);
  }

  bitstream.byteAlign();
  return x;
}

void VpccParameterSet::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};
  profile_tier_level().encodeTo(bitstream);

  VERIFY_VPCCBITSTREAM(vps_vpcc_parameter_set_id() <= 15);
  bitstream.writeBits(vps_vpcc_parameter_set_id(), 4);

  bitstream.putFlag(vps_miv_mode_flag());
  bitstream.writeBits(0, 7);

  VERIFY_VPCCBITSTREAM(vps_atlas_count_minus1() < 64);
  bitstream.writeBits(vps_atlas_count_minus1(), 6);

  for (int j = 0; j <= vps_atlas_count_minus1(); ++j) {
    VERIFY_VPCCBITSTREAM(1 <= vps_frame_width(j));
    bitstream.putUint16(vps_frame_width(j));

    VERIFY_VPCCBITSTREAM(1 <= vps_frame_height(j));
    bitstream.putUint16(vps_frame_height(j));

    VERIFY_VPCCBITSTREAM(vps_map_count_minus1(j) < 16);
    VERIFY_MIVBITSTREAM(vps_map_count_minus1(j) == 0);
    bitstream.writeBits(vps_map_count_minus1(j), 4);

    VERIFY_MIVBITSTREAM(!vps_auxiliary_video_present_flag(j));
    bitstream.putFlag(vps_auxiliary_video_present_flag(j));

    if (!vps_miv_mode_flag()) {
      occupancy_information(j).encodeTo(bitstream);
    }
    geometry_information(j).encodeTo(bitstream, *this, j);
    attribute_information(j).encodeTo(bitstream, *this, j);
  }

  VERIFY_MIVBITSTREAM(vps_extension_present_flag());

  constexpr auto vps_extension_bit_equal_to_one = true;
  bitstream.putFlag(vps_extension_bit_equal_to_one);

  const auto encodeExtension = [this](OutputBitstream &bitstream) {
    bitstream.putFlag(vps_miv_extension_flag());

    if (vps_miv_extension_flag()) {
      miv_sequence_params().encodeTo(bitstream);

      bitstream.putFlag(vps_miv_sequence_vui_params_present_flag());

      if (vps_miv_sequence_vui_params_present_flag()) {
        miv_vui_params().encodeTo(bitstream);
      }
    }
  };

  const auto vps_extension_length_minus1 = [encodeExtension]() {
    ostringstream stream;
    OutputBitstream bitstream{stream};
    encodeExtension(bitstream);
    return (unsigned(bitstream.tellp()) - 1U) / 8U;
  }();

  bitstream.putUExpGolomb(vps_extension_length_minus1);
  const auto extensionEnd = uint64_t(bitstream.tellp()) + 8 * (vps_extension_length_minus1 + 1);
  encodeExtension(bitstream);

  while (uint64_t(bitstream.tellp()) < extensionEnd) {
    bitstream.putFlag(false);
  }
  VERIFY_MIVBITSTREAM(uint64_t(bitstream.tellp()) == extensionEnd);

  bitstream.byteAlign();
}

auto merge(const vector<const VpccParameterSet *> &vps) -> VpccParameterSet {
  VERIFY_MIVBITSTREAM(!vps.empty());
  auto x = *vps.front();

  VERIFY_MIVBITSTREAM(x.miv_sequence_params().msp_num_groups_minus1() + 1 == vps.size());

  for (auto i = begin(vps) + 1; i != end(vps); ++i) {
    const auto &y = **i;
    VERIFY_MIVBITSTREAM(x.profile_tier_level() == y.profile_tier_level());
    VERIFY_MIVBITSTREAM(x.vps_vpcc_parameter_set_id() == y.vps_vpcc_parameter_set_id());
    VERIFY_MIVBITSTREAM(x.vps_miv_mode_flag() == y.vps_miv_mode_flag());

    x.m_vps_atlases.insert(x.m_vps_atlases.end(), begin(y.m_vps_atlases), end(y.m_vps_atlases));

    VERIFY_MIVBITSTREAM(x.vps_extension_present_flag() && y.vps_extension_present_flag());
    VERIFY_MIVBITSTREAM(x.vps_miv_extension_flag() == y.vps_miv_extension_flag());

    if (x.vps_miv_extension_flag()) {
      VERIFY_MIVBITSTREAM(x.miv_sequence_params() == y.miv_sequence_params());
      VERIFY_MIVBITSTREAM(x.vps_miv_sequence_vui_params_present_flag() ==
                          y.vps_miv_sequence_vui_params_present_flag());
      if (x.vps_miv_sequence_vui_params_present_flag()) {
        VERIFY_MIVBITSTREAM(x.miv_vui_params() == y.miv_vui_params());
      }
    }
  }
  return x;
}

} // namespace TMIV::MivBitstream
