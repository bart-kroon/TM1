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

#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <TMIV/MivBitstream/MivDecoderMode.h>
#include <TMIV/MivBitstream/verify.h>

#include <utility>

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

auto codeOf(AiAttributeTypeId typeId) -> char {
  switch (typeId) {
  case AiAttributeTypeId::ATTR_TEXTURE:
    return 'T';
  case AiAttributeTypeId::ATTR_MATERIAL_ID:
    return 'M';
  case AiAttributeTypeId::ATTR_TRANSPARENCY:
    return 'A';
  case AiAttributeTypeId::ATTR_REFLECTANCE:
    return 'R';
  case AiAttributeTypeId::ATTR_NORMAL:
    return 'N';
  default:
    V3CBITSTREAM_ERROR("Unknown attribute type ID");
  }
}

auto ProfileTierLevel::ptl_num_sub_profiles() const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(m_subProfileIdcs.size() <= UINT8_MAX);
  return uint8_t(m_subProfileIdcs.size());
}

auto ProfileTierLevel::ptl_sub_profile_idc(std::uint8_t i) const noexcept -> uint64_t {
  VERIFY_V3CBITSTREAM(i < m_subProfileIdcs.size());
  return m_subProfileIdcs[i];
}

auto ProfileTierLevel::ptl_num_sub_profiles(std::uint8_t value) noexcept -> ProfileTierLevel & {
  m_subProfileIdcs = vector<uint64_t>(value, 0);
  return *this;
}

auto ProfileTierLevel::ptl_extended_sub_profile_flag(bool value) noexcept -> ProfileTierLevel & {
  m_ptl_extended_sub_profile_flag = value;
  for (auto x : m_subProfileIdcs) {
    VERIFY_V3CBITSTREAM(ptl_extended_sub_profile_flag() || x <= UINT32_MAX);
  }
  return *this;
}

auto ProfileTierLevel::ptl_sub_profile_idc(std::uint8_t i, std::uint64_t value) noexcept
    -> ProfileTierLevel & {
  VERIFY_V3CBITSTREAM(i < ptl_num_sub_profiles());
  VERIFY_V3CBITSTREAM(ptl_extended_sub_profile_flag() || value <= UINT32_MAX);
  m_subProfileIdcs[i] = value;
  return *this;
}

auto operator<<(ostream &stream, const ProfileTierLevel &x) -> ostream & {
  stream << "ptl_tier_flag=" << boolalpha << x.ptl_tier_flag() << '\n';
  stream << "ptl_profile_codec_group_idc=" << x.ptl_profile_codec_group_idc() << '\n';
  stream << "ptl_profile_pcc_toolset_idc=" << x.ptl_profile_pcc_toolset_idc() << '\n';
  stream << "ptl_profile_reconstruction_idc=" << x.ptl_profile_reconstruction_idc() << '\n';
  stream << "ptl_level_idc=" << x.ptl_level_idc() << '\n';
  stream << "ptl_num_sub_profiles=" << int(x.ptl_num_sub_profiles()) << '\n';
  stream << "ptl_extended_sub_profile_flag=" << boolalpha << x.ptl_extended_sub_profile_flag()
         << '\n';
  for (uint8_t i = 0; i < x.ptl_num_sub_profiles(); ++i) {
    stream << "ptl_sub_profile_idc[ " << int(i) << " ]=" << x.ptl_sub_profile_idc(i) << '\n';
  }
  stream << "ptl_tool_constraints_present_flag=" << boolalpha
         << x.ptl_tool_constraints_present_flag() << '\n';
  return stream;
}

auto ProfileTierLevel::operator==(const ProfileTierLevel &other) const noexcept -> bool {
  return ptl_tier_flag() == other.ptl_tier_flag() &&
         ptl_profile_codec_group_idc() == other.ptl_profile_codec_group_idc() &&
         ptl_profile_pcc_toolset_idc() == other.ptl_profile_pcc_toolset_idc() &&
         ptl_profile_reconstruction_idc() == other.ptl_profile_reconstruction_idc() &&
         ptl_level_idc() == other.ptl_level_idc() &&
         ptl_extended_sub_profile_flag() == other.ptl_extended_sub_profile_flag() &&
         m_subProfileIdcs == other.m_subProfileIdcs &&
         ptl_tool_constraints_present_flag() == other.ptl_tool_constraints_present_flag();
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
  x.ptl_num_sub_profiles(bitstream.readBits<uint8_t>(6));
  x.ptl_extended_sub_profile_flag(bitstream.getFlag());
  for (uint8_t i = 0; i < x.ptl_num_sub_profiles(); ++i) {
    if (x.ptl_extended_sub_profile_flag()) {
      x.ptl_sub_profile_idc(i, bitstream.getUint64());
    } else {
      x.ptl_sub_profile_idc(i, bitstream.getUint32());
    }
  }
  x.ptl_tool_constraints_present_flag(bitstream.getFlag());
  LIMITATION(!x.ptl_tool_constraints_present_flag());
  return x;
}

void ProfileTierLevel::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFlag(ptl_tier_flag());
  bitstream.writeBits(ptl_profile_codec_group_idc(), 7);
  bitstream.writeBits(ptl_profile_pcc_toolset_idc(), 8);
  bitstream.writeBits(ptl_profile_reconstruction_idc(), 8);
  bitstream.putUint32(0);
  bitstream.writeBits(ptl_level_idc(), 8);
  bitstream.writeBits(ptl_num_sub_profiles(), 6);
  bitstream.putFlag(ptl_extended_sub_profile_flag());
  for (uint8_t i = 0; i < ptl_num_sub_profiles(); ++i) {
    if (ptl_extended_sub_profile_flag()) {
      bitstream.putUint64(ptl_sub_profile_idc(i));
    } else {
      VERIFY_V3CBITSTREAM(ptl_sub_profile_idc(i) <= UINT32_MAX);
      bitstream.putUint32(uint32_t(ptl_sub_profile_idc(i)));
    }
  }
  bitstream.putFlag(ptl_tool_constraints_present_flag());
  LIMITATION(!ptl_tool_constraints_present_flag());
}

auto OccupancyInformation::printTo(ostream &stream, uint8_t atlasIdx) const -> ostream & {
  stream << "oi_occupancy_codec_id( " << int(atlasIdx) << " )=" << int(oi_occupancy_codec_id())
         << '\n';
  stream << "oi_lossy_occupancy_map_compression_threshold( " << int(atlasIdx)
         << " )=" << int(oi_lossy_occupancy_map_compression_threshold()) << '\n';
  stream << "oi_occupancy_nominal_2d_bitdepth_minus1( " << int(atlasIdx)
         << " )=" << int(oi_occupancy_nominal_2d_bitdepth_minus1()) << '\n';
  stream << "oi_occupancy_MSB_align_flag( " << int(atlasIdx) << " )=" << boolalpha
         << oi_occupancy_MSB_align_flag() << '\n';
  return stream;
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

auto GeometryInformation::printTo(ostream &stream, uint8_t atlasIdx) const -> ostream & {
  stream << "gi_geometry_codec_id( " << int(atlasIdx) << " )=" << int(gi_geometry_codec_id())
         << '\n';
  stream << "gi_geometry_nominal_2d_bitdepth_minus1( " << int(atlasIdx)
         << " )=" << int(gi_geometry_nominal_2d_bitdepth_minus1()) << '\n';
  stream << "gi_geometry_MSB_align_flag( " << int(atlasIdx) << " )=" << boolalpha
         << gi_geometry_MSB_align_flag() << '\n';
  stream << "gi_geometry_3d_coordinates_bitdepth_minus1( " << int(atlasIdx)
         << " )=" << int(gi_geometry_3d_coordinates_bitdepth_minus1()) << '\n';
  return stream;
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

auto GeometryInformation::decodeFrom(InputBitstream &bitstream, const V3cParameterSet &vps,
                                     uint8_t atlasIdx) -> GeometryInformation {
  auto x = GeometryInformation{};
  x.gi_geometry_codec_id(bitstream.getUint8());
  x.gi_geometry_nominal_2d_bitdepth_minus1(bitstream.readBits<uint8_t>(5));
  x.gi_geometry_MSB_align_flag(bitstream.getFlag());
  x.gi_geometry_3d_coordinates_bitdepth_minus1(bitstream.readBits<uint8_t>(5));
  VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasIdx));
  return x;
}

void GeometryInformation::encodeTo(OutputBitstream &bitstream, const V3cParameterSet &vps,
                                   uint8_t atlasIdx) const {
  bitstream.putUint8(gi_geometry_codec_id());
  bitstream.writeBits(gi_geometry_nominal_2d_bitdepth_minus1(), 5);
  bitstream.putFlag(gi_geometry_MSB_align_flag());
  bitstream.writeBits(gi_geometry_3d_coordinates_bitdepth_minus1(), 5);
  VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasIdx));
}

auto AttributeInformation::ai_attribute_count() const noexcept -> uint8_t {
  return uint8_t(m_aiAttributes.size());
}

auto AttributeInformation::ai_attribute_type_id(uint8_t attributeId) const -> AiAttributeTypeId {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  return m_aiAttributes[attributeId].ai_attribute_type_id;
}

auto AttributeInformation::ai_attribute_codec_id(uint8_t attributeId) const -> uint8_t {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  return m_aiAttributes[attributeId].ai_attribute_codec_id;
}

auto AttributeInformation::ai_attribute_dimension_minus1(uint8_t attributeId) const -> uint8_t {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  return m_aiAttributes[attributeId].ai_attribute_dimension_minus1;
}

auto AttributeInformation::ai_attribute_nominal_2d_bitdepth_minus1(uint8_t attributeId) const
    -> uint8_t {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  return m_aiAttributes[attributeId].ai_attribute_nominal_2d_bitdepth_minus1;
}

auto AttributeInformation::ai_attribute_MSB_align_flag(uint8_t attributeId) const -> bool {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  return m_aiAttributes[attributeId].ai_attribute_MSB_align_flag;
}

auto AttributeInformation::ai_attribute_map_absolute_coding_persistence_flag(
    std::uint8_t attributeId) const -> bool {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  VERIFY_V3CBITSTREAM(
      m_aiAttributes[attributeId].ai_attribute_map_absolute_coding_persistence_flag.has_value());
  return *m_aiAttributes[attributeId].ai_attribute_map_absolute_coding_persistence_flag;
}

auto AttributeInformation::ai_attribute_count(uint8_t value) -> AttributeInformation & {
  m_aiAttributes.resize(value);
  return *this;
}

auto AttributeInformation::ai_attribute_type_id(uint8_t attributeId, AiAttributeTypeId value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  m_aiAttributes[attributeId].ai_attribute_type_id = value;
  return *this;
}

auto AttributeInformation::ai_attribute_codec_id(uint8_t attributeId, uint8_t value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  m_aiAttributes[attributeId].ai_attribute_codec_id = value;
  return *this;
}

auto AttributeInformation::ai_attribute_map_absolute_coding_persistence_flag(
    std::uint8_t attributeId, bool value) -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  m_aiAttributes[attributeId].ai_attribute_map_absolute_coding_persistence_flag = value;
  return *this;
}

auto AttributeInformation::ai_attribute_dimension_minus1(uint8_t attributeId, uint8_t value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  m_aiAttributes[attributeId].ai_attribute_dimension_minus1 = value;
  return *this;
}

auto AttributeInformation::ai_attribute_nominal_2d_bitdepth_minus1(uint8_t attributeId,
                                                                   uint8_t value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  m_aiAttributes[attributeId].ai_attribute_nominal_2d_bitdepth_minus1 = value;
  return *this;
}

auto AttributeInformation::ai_attribute_MSB_align_flag(uint8_t attributeId, bool value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attributeId < ai_attribute_count());
  m_aiAttributes[attributeId].ai_attribute_MSB_align_flag = value;
  return *this;
}

auto AttributeInformation::printTo(ostream &stream, uint8_t atlasIdx) const -> ostream & {
  stream << "ai_attribute_count( " << int(atlasIdx) << " )=" << int(ai_attribute_count()) << '\n';
  for (auto i = 0; i < ai_attribute_count(); ++i) {
    stream << "ai_attribute_type_id( " << int(atlasIdx) << ", " << i
           << " )=" << ai_attribute_type_id(i) << '\n';
    stream << "ai_attribute_codec_id( " << int(atlasIdx) << ", " << i
           << " )=" << int(ai_attribute_codec_id(i)) << '\n';
    if (m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag) {
      stream << "ai_attribute_map_absolute_coding_persistence_flag( " << int(atlasIdx) << ", " << i
             << " )=" << boolalpha
             << *m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag << '\n';
    }
    stream << "ai_attribute_dimension_minus1( " << int(atlasIdx) << ", " << i
           << " )=" << int(ai_attribute_dimension_minus1(i)) << '\n';
    stream << "ai_attribute_nominal_2d_bitdepth_minus1( " << int(atlasIdx) << ", " << i
           << " )=" << int(ai_attribute_nominal_2d_bitdepth_minus1(i));
    stream << '\n';
    stream << "ai_attribute_MSB_align_flag( " << int(atlasIdx) << ", " << i << " )=" << boolalpha
           << ai_attribute_MSB_align_flag(i) << '\n';
  }
  return stream;
}

auto AttributeInformation::operator==(const AttributeInformation &other) const noexcept -> bool {
  if (ai_attribute_count() != other.ai_attribute_count()) {
    return false;
  }
  for (auto i = 0; i < ai_attribute_count(); ++i) {
    if (ai_attribute_type_id(i) != other.ai_attribute_type_id(i) ||
        ai_attribute_codec_id(i) != other.ai_attribute_codec_id(i) ||
        m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag !=
            other.m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag ||
        ai_attribute_dimension_minus1(i) != other.ai_attribute_dimension_minus1(i) ||
        ai_attribute_nominal_2d_bitdepth_minus1(i) !=
            other.ai_attribute_nominal_2d_bitdepth_minus1(i) ||
        ai_attribute_MSB_align_flag(i) != other.ai_attribute_MSB_align_flag(i)) {
      return false;
    }
  }
  return true;
}

auto AttributeInformation::operator!=(const AttributeInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto AttributeInformation::decodeFrom(InputBitstream &bitstream, const V3cParameterSet &vps,
                                      uint8_t atlasIdx) -> AttributeInformation {
  auto x = AttributeInformation{};
  x.ai_attribute_count(bitstream.readBits<uint8_t>(7));
  for (auto i = 0; i < x.ai_attribute_count(); ++i) {
    x.ai_attribute_type_id(i, bitstream.readBits<AiAttributeTypeId>(4));
    x.ai_attribute_codec_id(i, bitstream.getUint8());

    VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasIdx));

    if (vps.vps_map_count_minus1(atlasIdx) > 0) {
      x.ai_attribute_map_absolute_coding_persistence_flag(i, bitstream.getFlag());
    }

    x.ai_attribute_dimension_minus1(i, bitstream.readBits<uint8_t>(6));

    const auto ai_attribute_dimension_partitions_minus1 = bitstream.readBits<uint8_t>(6);
    VERIFY_MIVBITSTREAM(ai_attribute_dimension_partitions_minus1 == 0);

    x.ai_attribute_nominal_2d_bitdepth_minus1(i, bitstream.readBits<uint8_t>(5));
    x.ai_attribute_MSB_align_flag(i, bitstream.getFlag());
  }
  return x;
}

void AttributeInformation::encodeTo(OutputBitstream &bitstream, const V3cParameterSet &vps,
                                    uint8_t atlasIdx) const {
  bitstream.writeBits(ai_attribute_count(), 7);
  for (auto i = 0; i < ai_attribute_count(); ++i) {
    bitstream.writeBits(ai_attribute_type_id(i), 4);
    bitstream.writeBits(ai_attribute_codec_id(i), 8);

    if (vps.vps_map_count_minus1(atlasIdx) > 0) {
      bitstream.putFlag(ai_attribute_map_absolute_coding_persistence_flag(i));
    }

    VERIFY_V3CBITSTREAM(ai_attribute_dimension_minus1(i) < 64);
    bitstream.writeBits(ai_attribute_dimension_minus1(i), 6);

    constexpr auto ai_attribute_dimension_partitions_minus1 = 0;
    bitstream.writeBits(ai_attribute_dimension_partitions_minus1, 6);

    VERIFY_V3CBITSTREAM(ai_attribute_nominal_2d_bitdepth_minus1(i) < 32);
    bitstream.writeBits(ai_attribute_nominal_2d_bitdepth_minus1(i), 5);
    bitstream.putFlag(ai_attribute_MSB_align_flag(i));
  }
}

auto VpsMivExtension::miv_vui_parameters() const noexcept -> const MivVuiParams & {
  VERIFY_V3CBITSTREAM(vme_vui_params_present_flag());
  VERIFY_V3CBITSTREAM(m_mvp.has_value());
  return *m_mvp;
}

auto VpsMivExtension::miv_vui_parameters(const MivVuiParams &value) noexcept -> VpsMivExtension & {
  m_mvp = value;
  return *this;
}

auto operator<<(ostream &stream, const VpsMivExtension &x) -> ostream & {
  stream << "vme_depth_low_quality_flag=" << boolalpha << x.vme_depth_low_quality_flag() << '\n';
  stream << "vme_geometry_scale_enabled_flag=" << boolalpha << x.vme_geometry_scale_enabled_flag()
         << '\n';
  stream << "vme_num_groups_minus1=" << x.vme_num_groups_minus1() << '\n';
  stream << "vme_max_entities_minus1=" << x.vme_max_entities_minus1() << '\n';
  stream << "vme_embedded_occupancy_flag=" << boolalpha << x.vme_embedded_occupancy_flag() << '\n';
  if (!x.vme_embedded_occupancy_flag())
    stream << "vme_occupancy_scale_enabled_flag=" << boolalpha
           << x.vme_occupancy_scale_enabled_flag() << '\n';
  if (x.vme_vui_params_present_flag()) {
    stream << x.miv_vui_parameters();
  }
  return stream;
}

auto VpsMivExtension::decodeFrom(InputBitstream &bitstream) -> VpsMivExtension {
  auto x = VpsMivExtension{};
  x.vme_depth_low_quality_flag(bitstream.getFlag());
  x.vme_geometry_scale_enabled_flag(bitstream.getFlag());
  x.vme_num_groups_minus1(bitstream.getUExpGolomb<unsigned>());
  x.vme_max_entities_minus1(bitstream.getUExpGolomb<unsigned>());
  x.vme_embedded_occupancy_flag(bitstream.getFlag());
  if (!x.vme_embedded_occupancy_flag())
    x.vme_occupancy_scale_enabled_flag(bitstream.getFlag());
  x.vme_vui_params_present_flag(bitstream.getFlag());
  if (x.vme_vui_params_present_flag()) {
    x.miv_vui_parameters(MivVuiParams::decodeFrom(bitstream));
  }
  return x;
}

void VpsMivExtension::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFlag(vme_depth_low_quality_flag());
  bitstream.putFlag(vme_geometry_scale_enabled_flag());
  bitstream.putUExpGolomb(vme_num_groups_minus1());
  bitstream.putUExpGolomb(vme_max_entities_minus1());
  bitstream.putFlag(vme_embedded_occupancy_flag());
  if (!vme_embedded_occupancy_flag())
    bitstream.putFlag(vme_occupancy_scale_enabled_flag());
  bitstream.putFlag(vme_vui_params_present_flag());
  if (vme_vui_params_present_flag()) {
    miv_vui_parameters().encodeTo(bitstream);
  }
}

auto V3cParameterSet::profile_tier_level() const noexcept -> const ProfileTierLevel & {
  return m_profile_tier_level;
}

auto V3cParameterSet::vps_atlas_count_minus1() const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(!m_vpsAtlases.empty());
  return uint8_t(m_vpsAtlases.size() - 1U);
}

auto V3cParameterSet::vps_atlas_id(uint8_t j) const -> uint8_t {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_atlas_id;
}

auto V3cParameterSet::vps_frame_width(uint8_t j) const -> uint16_t {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_frame_width;
}

auto V3cParameterSet::vps_frame_height(uint8_t j) const -> uint16_t {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_frame_height;
}

auto V3cParameterSet::vps_map_count_minus1(uint8_t j) const -> uint8_t {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_map_count_minus1;
}

auto V3cParameterSet::vps_auxiliary_video_present_flag(uint8_t j) const -> bool {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_auxiliary_video_present_flag;
}

auto V3cParameterSet::vps_occupancy_video_present_flag(uint8_t j) const -> bool {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_occupancy_video_present_flag;
}

auto V3cParameterSet::vps_geometry_video_present_flag(uint8_t j) const -> bool {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_geometry_video_present_flag;
}

auto V3cParameterSet::vps_attribute_video_present_flag(uint8_t j) const -> bool {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  return m_vpsAtlases[j].vps_attribute_video_present_flag;
}

auto V3cParameterSet::occupancy_information(uint8_t j) const -> const OccupancyInformation & {
  VERIFY_V3CBITSTREAM(vps_occupancy_video_present_flag(j));
  VERIFY_V3CBITSTREAM(m_vpsAtlases[j].occupancy_information.has_value());
  return *m_vpsAtlases[j].occupancy_information;
}

auto V3cParameterSet::geometry_information(uint8_t j) const -> const GeometryInformation & {
  VERIFY_V3CBITSTREAM(vps_geometry_video_present_flag(j));
  VERIFY_V3CBITSTREAM(m_vpsAtlases[j].geometry_information.has_value());
  return *m_vpsAtlases[j].geometry_information;
}

auto V3cParameterSet::attribute_information(uint8_t j) const -> const AttributeInformation & {
  VERIFY_V3CBITSTREAM(vps_attribute_video_present_flag(j));
  VERIFY_V3CBITSTREAM(m_vpsAtlases[j].attribute_information.has_value());
  return *m_vpsAtlases[j].attribute_information;
}

auto V3cParameterSet::vps_vpcc_extension() const noexcept -> const VpsVpccExtension & {
  VERIFY_V3CBITSTREAM(vps_vpcc_extension_flag());
  VERIFY_V3CBITSTREAM(m_vps_vpcc_extension.has_value());
  return *m_vps_vpcc_extension;
}

auto V3cParameterSet::vps_miv_extension() const noexcept -> const VpsMivExtension & {
  VERIFY_V3CBITSTREAM(vps_miv_extension_flag());
  VERIFY_V3CBITSTREAM(m_vps_miv_extension.has_value());
  return *m_vps_miv_extension;
}

auto V3cParameterSet::vps_extension_length_minus1() const noexcept -> size_t {
  VERIFY_V3CBITSTREAM(vps_extension_6bits());
  VERIFY_V3CBITSTREAM(m_vpsExtensionData.has_value());
  return m_vpsExtensionData->size() - 1;
}

auto V3cParameterSet::vpsExtensionData() const noexcept -> const vector<uint8_t> & {
  VERIFY_V3CBITSTREAM(vps_extension_6bits());
  return *m_vpsExtensionData;
}

auto V3cParameterSet::profile_tier_level(ProfileTierLevel value) noexcept -> V3cParameterSet & {
  m_profile_tier_level = std::move(value);
  return *this;
}

auto V3cParameterSet::vps_atlas_count_minus1(uint8_t value) -> V3cParameterSet & {
  m_vpsAtlases.resize(value + 1U);
  return *this;
}

auto V3cParameterSet::vps_atlas_id(uint8_t j, uint8_t value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_atlas_id = value;
  return *this;
}

auto V3cParameterSet::vps_frame_width(uint8_t j, uint16_t value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_frame_width = value;
  return *this;
}

auto V3cParameterSet::vps_frame_height(uint8_t j, uint16_t value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_frame_height = value;
  return *this;
}

auto V3cParameterSet::vps_map_count_minus1(uint8_t j, uint8_t value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_map_count_minus1 = value;
  return *this;
}

auto V3cParameterSet::vps_auxiliary_video_present_flag(uint8_t j, bool value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_auxiliary_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_occupancy_video_present_flag(uint8_t j, bool value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_occupancy_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_geometry_video_present_flag(uint8_t j, bool value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_geometry_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_attribute_video_present_flag(uint8_t j, bool value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  m_vpsAtlases[j].vps_attribute_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::occupancy_information(uint8_t j, OccupancyInformation value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_occupancy_video_present_flag(j));
  m_vpsAtlases[j].occupancy_information = value;
  return *this;
}

auto V3cParameterSet::geometry_information(uint8_t j, GeometryInformation value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_geometry_video_present_flag(j));
  m_vpsAtlases[j].geometry_information = value;
  return *this;
}

auto V3cParameterSet::attribute_information(uint8_t j, AttributeInformation value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_attribute_video_present_flag(j));
  m_vpsAtlases[j].attribute_information = move(value);
  return *this;
}

auto V3cParameterSet::vps_vpcc_extension_flag(bool value) noexcept -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_extension_present_flag());
  m_vps_vpcc_extension_flag = value;
  return *this;
}

auto V3cParameterSet::vps_miv_extension_flag(bool value) noexcept -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_extension_present_flag());
  m_vps_miv_extension_flag = value;
  return *this;
}

auto V3cParameterSet::vps_extension_6bits(uint8_t value) noexcept -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_extension_present_flag());
  VERIFY_V3CBITSTREAM(value < 0x40);
  m_vps_extension_6bits = value;
  return *this;
}

auto V3cParameterSet::vps_vpcc_extension(VpsVpccExtension value) noexcept -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_vpcc_extension_flag());
  m_vps_vpcc_extension = value;
  return *this;
}

auto V3cParameterSet::vps_miv_extension(VpsMivExtension value) noexcept -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_miv_extension_flag());
  m_vps_miv_extension = value;
  return *this;
}

auto V3cParameterSet::vpsExtensionData(std::vector<std::uint8_t> value) noexcept
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_extension_6bits() != 0);
  VERIFY_V3CBITSTREAM(!value.empty());
  m_vpsExtensionData = move(value);
  return *this;
}

auto V3cParameterSet::occupancy_information(uint8_t j) -> OccupancyInformation & {
  VERIFY_V3CBITSTREAM(vps_occupancy_video_present_flag(j));
  auto &oi = m_vpsAtlases[j].occupancy_information;
  if (!oi) {
    oi = OccupancyInformation{};
  }
  return *oi;
}

auto V3cParameterSet::geometry_information(uint8_t j) -> GeometryInformation & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  auto &gi = m_vpsAtlases[j].geometry_information;
  if (!gi) {
    gi = GeometryInformation{};
  }
  return *gi;
}

auto V3cParameterSet::attribute_information(uint8_t j) -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(j <= vps_atlas_count_minus1());
  auto &ai = m_vpsAtlases[j].attribute_information;
  if (!ai) {
    ai = AttributeInformation{};
  }
  return *ai;
}

auto V3cParameterSet::vps_miv_extension() noexcept -> VpsMivExtension & {
  VERIFY_V3CBITSTREAM(vps_miv_extension_flag());
  if (!m_vps_miv_extension) {
    m_vps_miv_extension = VpsMivExtension{};
  }
  return *m_vps_miv_extension;
}

auto V3cParameterSet::atlasIdxOf(uint8_t atlasId) const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(atlasId != specialAtlasId);

  for (uint8_t j = 0; j <= vps_atlas_count_minus1(); ++j) {
    if (vps_atlas_id(j) == atlasId) {
      return j;
    }
  }
  V3CBITSTREAM_ERROR("The atlasId is not in the VPS");
}

auto operator<<(ostream &stream, const V3cParameterSet &x) -> ostream & {
  stream << x.profile_tier_level();
  stream << "vps_v3c_parameter_set_id=" << int(x.vps_v3c_parameter_set_id()) << '\n';
  stream << "vps_atlas_count_minus1=" << int(x.vps_atlas_count_minus1()) << '\n';
  for (int j = 0; j <= x.vps_atlas_count_minus1(); ++j) {
    stream << "vps_atlas_id( " << j << " )=" << int(x.vps_atlas_id(j)) << '\n';
    stream << "vps_frame_width( " << j << " )=" << x.vps_frame_width(j) << '\n';
    stream << "vps_frame_height( " << j << " )=" << x.vps_frame_height(j) << '\n';
    stream << "vps_map_count_minus1( " << j << " )=" << int(x.vps_map_count_minus1(j)) << '\n';
    stream << "vps_auxiliary_video_present_flag( " << j << " )=" << boolalpha
           << x.vps_auxiliary_video_present_flag(j) << '\n';
    stream << "vps_occupancy_video_present_flag( " << j << " )=" << boolalpha
           << x.vps_occupancy_video_present_flag(j) << '\n';
    stream << "vps_geometry_video_present_flag( " << j << " )=" << boolalpha
           << x.vps_geometry_video_present_flag(j) << '\n';
    stream << "vps_attribute_video_present_flag( " << j << " )=" << boolalpha
           << x.vps_attribute_video_present_flag(j) << '\n';
    if (x.vps_occupancy_video_present_flag(j)) {
      x.occupancy_information(j).printTo(stream, j);
    }
    if (x.vps_geometry_video_present_flag(j)) {
      x.geometry_information(j).printTo(stream, j);
    }
    if (x.vps_attribute_video_present_flag(j)) {
      x.attribute_information(j).printTo(stream, j);
    }
  }
  stream << "vps_extension_present_flag=" << boolalpha << x.vps_extension_present_flag() << '\n';
  if (x.vps_extension_present_flag()) {
    stream << "vps_vpcc_extension_flag=" << boolalpha << x.vps_vpcc_extension_flag() << '\n';
    stream << "vps_miv_extension_flag=" << boolalpha << x.vps_miv_extension_flag() << '\n';
    stream << "vps_extension_6bits=" << int(x.vps_extension_6bits()) << '\n';
  }
  if (x.vps_vpcc_extension_flag()) {
    stream << x.vps_vpcc_extension();
  }
  if (x.vps_miv_extension_flag()) {
    stream << x.vps_miv_extension();
  }
  if (x.vps_extension_6bits() != 0) {
    stream << "vps_extension_length_minus1=" << x.vps_extension_length_minus1() << '\n';
    for (uint8_t byte : x.vpsExtensionData()) {
      stream << "vps_extension_data_byte=" << int(byte) << '\n';
    }
  }
  return stream;
}

auto V3cParameterSet::operator==(const V3cParameterSet &other) const noexcept -> bool {
  if (profile_tier_level() != other.profile_tier_level() ||
      vps_v3c_parameter_set_id() != other.vps_v3c_parameter_set_id() ||
      vps_atlas_count_minus1() != other.vps_atlas_count_minus1() ||
      vps_extension_present_flag() != other.vps_extension_present_flag() ||
      vps_vpcc_extension_flag() != other.vps_vpcc_extension_flag() ||
      vps_miv_extension_flag() != other.vps_miv_extension_flag() ||
      vps_extension_6bits() != other.vps_extension_6bits()) {
    return false;
  }
  for (int j = 0; j <= vps_atlas_count_minus1(); ++j) {
    if (vps_atlas_id(j) != other.vps_atlas_id(j) ||
        vps_frame_width(j) != other.vps_frame_width(j) ||
        vps_frame_height(j) != other.vps_frame_height(j) ||
        vps_map_count_minus1(j) != other.vps_map_count_minus1(j) ||
        vps_auxiliary_video_present_flag(j) != other.vps_auxiliary_video_present_flag(j) ||
        vps_occupancy_video_present_flag(j) != other.vps_occupancy_video_present_flag(j) ||
        vps_geometry_video_present_flag(j) != other.vps_geometry_video_present_flag(j) ||
        vps_attribute_video_present_flag(j) != other.vps_attribute_video_present_flag(j)) {
      return false;
    }
    if (vps_occupancy_video_present_flag(j) &&
        occupancy_information(j) != other.occupancy_information(j)) {
      return false;
    }
    if (vps_geometry_video_present_flag(j) &&
        geometry_information(j) != other.geometry_information(j)) {
      return false;
    }
    if (vps_attribute_video_present_flag(j) &&
        attribute_information(j) != other.attribute_information(j)) {
      return false;
    }
  }
  if (vps_vpcc_extension_flag() && vps_vpcc_extension() != other.vps_vpcc_extension()) {
    return false;
  }
  if (vps_miv_extension_flag() && vps_miv_extension() != other.vps_miv_extension()) {
    return false;
  }
  if (vps_extension_6bits() && vpsExtensionData() != other.vpsExtensionData()) {
    return false;
  }
  return true;
}

auto V3cParameterSet::operator!=(const V3cParameterSet &other) const noexcept -> bool {
  return !operator==(other);
}

auto V3cParameterSet::decodeFrom(istream &stream) -> V3cParameterSet {
  auto x = V3cParameterSet{};
  InputBitstream bitstream{stream};

  x.profile_tier_level(ProfileTierLevel::decodeFrom(bitstream));
  x.vps_v3c_parameter_set_id(bitstream.readBits<uint8_t>(4));
  bitstream.getUint8(); // vps_reserved_zero_8bits
  x.vps_atlas_count_minus1(bitstream.readBits<uint8_t>(6));

  for (int j = 0; j <= x.vps_atlas_count_minus1(); ++j) {
    x.vps_atlas_id(j, bitstream.readBits<uint8_t>(6));
    x.vps_frame_width(j, bitstream.getUint16());
    x.vps_frame_height(j, bitstream.getUint16());
    x.vps_map_count_minus1(j, bitstream.readBits<uint8_t>(4));

    if (x.vps_map_count_minus1(j) > 0) {
      const auto vps_multiple_map_streams_present_flag = bitstream.getFlag();
      VERIFY_MIVBITSTREAM(!vps_multiple_map_streams_present_flag);
    }

    x.vps_auxiliary_video_present_flag(j, bitstream.getFlag());
    x.vps_occupancy_video_present_flag(j, bitstream.getFlag());
    x.vps_geometry_video_present_flag(j, bitstream.getFlag());
    x.vps_attribute_video_present_flag(j, bitstream.getFlag());

    if (x.vps_occupancy_video_present_flag(j)) {
      x.occupancy_information(j, OccupancyInformation::decodeFrom(bitstream));
    }
    if (x.vps_geometry_video_present_flag(j)) {
      x.geometry_information(j, GeometryInformation::decodeFrom(bitstream, x, j));
    }
    if (x.vps_attribute_video_present_flag(j)) {
      x.attribute_information(j, AttributeInformation::decodeFrom(bitstream, x, j));
    }
  }

  x.vps_extension_present_flag(bitstream.getFlag());

  if (x.vps_extension_present_flag()) {
    x.vps_vpcc_extension_flag(bitstream.getFlag());
    x.vps_miv_extension_flag(bitstream.getFlag());
    x.vps_extension_6bits(bitstream.readBits<uint8_t>(6));
  }
  if (x.vps_vpcc_extension_flag()) {
    x.vps_vpcc_extension(VpsVpccExtension::decodeFrom(bitstream));
  }
  if (x.vps_miv_extension_flag()) {
    x.vps_miv_extension(VpsMivExtension::decodeFrom(bitstream));
  }
  if (x.vps_extension_6bits()) {
    const auto vps_extension_length_minus1 = bitstream.getUExpGolomb<size_t>();
    auto vpsExtensionData = vector<uint8_t>();
    vpsExtensionData.reserve(vps_extension_length_minus1 + 1);
    for (size_t j = 0; j <= vps_extension_length_minus1; ++j) {
      vpsExtensionData.push_back(bitstream.getUint8());
    }
    x.vpsExtensionData(vpsExtensionData);
  }
  bitstream.byteAlignment();
  return x;
}

void V3cParameterSet::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};
  profile_tier_level().encodeTo(bitstream);
  bitstream.writeBits(vps_v3c_parameter_set_id(), 4);
  bitstream.putUint8(0); // vps_reserved_zero_8bits
  bitstream.writeBits(vps_atlas_count_minus1(), 6);

  for (int j = 0; j <= vps_atlas_count_minus1(); ++j) {
    bitstream.writeBits(vps_atlas_id(j), 6);
    bitstream.putUint16(vps_frame_width(j));
    bitstream.putUint16(vps_frame_height(j));
    bitstream.writeBits(vps_map_count_minus1(j), 4);

    if (vps_map_count_minus1(j) > 0) {
      const auto vps_multiple_map_streams_present_flag = false;
      bitstream.putFlag(vps_multiple_map_streams_present_flag);
    }

    bitstream.putFlag(vps_auxiliary_video_present_flag(j));
    bitstream.putFlag(vps_occupancy_video_present_flag(j));
    bitstream.putFlag(vps_geometry_video_present_flag(j));
    bitstream.putFlag(vps_attribute_video_present_flag(j));

    if (vps_occupancy_video_present_flag(j)) {
      occupancy_information(j).encodeTo(bitstream);
    }
    if (vps_geometry_video_present_flag(j)) {
      geometry_information(j).encodeTo(bitstream, *this, j);
    }
    if (vps_attribute_video_present_flag(j)) {
      attribute_information(j).encodeTo(bitstream, *this, j);
    }
  }

  bitstream.putFlag(vps_extension_present_flag());

  if (vps_extension_present_flag()) {
    bitstream.putFlag(vps_vpcc_extension_flag());
    bitstream.putFlag(vps_miv_extension_flag());
    bitstream.writeBits(vps_extension_6bits(), 6);
  }
  if (vps_vpcc_extension_flag()) {
    vps_vpcc_extension().encodeTo(bitstream);
  }
  if (vps_miv_extension_flag()) {
    vps_miv_extension().encodeTo(bitstream);
  }
  if (vps_extension_6bits()) {
    bitstream.putUExpGolomb(vps_extension_length_minus1());
    for (uint8_t byte : vpsExtensionData()) {
      bitstream.putUint8(byte);
    }
  }
  bitstream.byteAlignment();
}

auto merge(const vector<const V3cParameterSet *> &vps) -> V3cParameterSet {
  VERIFY_MIVBITSTREAM(!vps.empty());
  auto x = *vps.front();

  VERIFY_MIVBITSTREAM(x.vps_miv_extension().vme_num_groups_minus1() + 1 == vps.size());

  for (auto i = begin(vps) + 1; i != end(vps); ++i) {
    const auto &y = **i;
    VERIFY_MIVBITSTREAM(x.profile_tier_level() == y.profile_tier_level());
    VERIFY_MIVBITSTREAM(x.vps_v3c_parameter_set_id() == y.vps_v3c_parameter_set_id());

    x.m_vpsAtlases.insert(x.m_vpsAtlases.end(), begin(y.m_vpsAtlases), end(y.m_vpsAtlases));

    VERIFY_MIVBITSTREAM(x.vps_extension_present_flag() && y.vps_extension_present_flag());
    VERIFY_MIVBITSTREAM(x.vps_miv_extension_flag() == y.vps_miv_extension_flag());

    if (x.vps_miv_extension_flag()) {
      VERIFY_MIVBITSTREAM(x.vps_miv_extension() == y.vps_miv_extension());
    }
  }
  for (uint8_t i = 0; i <= x.vps_atlas_count_minus1(); ++i) {
    x.vps_atlas_id(i, i);
  }
  return x;
}

} // namespace TMIV::MivBitstream
