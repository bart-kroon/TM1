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

#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <numeric>
#include <sstream>
#include <type_traits>
#include <utility>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, PtlProfileCodecGroupIdc x) -> std::ostream & {
  switch (x) {
  case PtlProfileCodecGroupIdc::AVC_Progressive_High:
    return stream << "AVC Progressive High";
  case PtlProfileCodecGroupIdc::HEVC_Main10:
    return stream << "HEVC Main10";
  case PtlProfileCodecGroupIdc::HEVC444:
    return stream << "HEVC444";
  case PtlProfileCodecGroupIdc::VVC_Main10:
    return stream << "VVC Main10";
  case PtlProfileCodecGroupIdc::MP4RA:
    return stream << "MP4RA";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, PtlProfileToolsetIdc x) -> std::ostream & {
  switch (x) {
  case PtlProfileToolsetIdc::VPCC_Basic:
    return stream << "V-PCC Basic";
  case PtlProfileToolsetIdc::VPCC_Extended:
    return stream << "V-PCC Extended";
  case PtlProfileToolsetIdc::MIV_Main:
    return stream << "MIV Main";
  case PtlProfileToolsetIdc::MIV_Extended:
    return stream << "MIV Extended";
  case PtlProfileToolsetIdc::MIV_Geometry_Absent:
    return stream << "MIV Geometry Absent";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, PtlProfileReconstructionIdc x) -> std::ostream & {
  switch (x) {
  case PtlProfileReconstructionIdc::VPCC_Rec0:
    return stream << "Rec0";
  case PtlProfileReconstructionIdc::VPCC_Rec1:
    return stream << "Rec1";
  case PtlProfileReconstructionIdc::VPCC_Rec2:
    return stream << "Rec2";
  case PtlProfileReconstructionIdc::Rec_Unconstrained:
    return stream << "Rec Unconstrained";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, PtlMaxDecodesIdc x) -> std::ostream & {
  switch (x) {
  case PtlMaxDecodesIdc::max_1:
    return stream << "max_1";
  case PtlMaxDecodesIdc::max_2:
    return stream << "max_2";
  case PtlMaxDecodesIdc::max_3:
    return stream << "max_3";
  case PtlMaxDecodesIdc::max_4:
    return stream << "max_4";
  case PtlMaxDecodesIdc::max_6:
    return stream << "max_6";
  case PtlMaxDecodesIdc::max_12:
    return stream << "max_12";
  case PtlMaxDecodesIdc::max_16:
    return stream << "max_16";
  case PtlMaxDecodesIdc::max_24:
    return stream << "max_24";
  case PtlMaxDecodesIdc::max_32:
    return stream << "max_32";
  case PtlMaxDecodesIdc::unconstrained:
    return stream << "unconstrained";
  default:
    return stream << "[reserved:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, PtlLevelIdc x) -> std::ostream & {
  switch (x) {
  case PtlLevelIdc::Level_1_0:
    return stream << "1.0";
  case PtlLevelIdc::Level_1_5:
    return stream << "1.5";
  case PtlLevelIdc::Level_2_0:
    return stream << "2.0";
  case PtlLevelIdc::Level_2_5:
    return stream << "2.5";
  case PtlLevelIdc::Level_3_0:
    return stream << "3.0";
  case PtlLevelIdc::Level_3_5:
    return stream << "3.5";
  case PtlLevelIdc::Level_4_0:
    return stream << "4.0";
  case PtlLevelIdc::Level_4_5:
    return stream << "4.5";
  case PtlLevelIdc::Level_8_5:
    return stream << "8.5";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, VuhUnitType x) -> std::ostream & {
  switch (x) {
  case VuhUnitType::V3C_VPS:
    return stream << "V3C_VPS";
  case VuhUnitType::V3C_AD:
    return stream << "V3C_AD";
  case VuhUnitType::V3C_OVD:
    return stream << "V3C_OVD";
  case VuhUnitType::V3C_GVD:
    return stream << "V3C_GVD";
  case VuhUnitType::V3C_AVD:
    return stream << "V3C_AVD";
  case VuhUnitType::V3C_PVD:
    return stream << "V3C_PVD";
  case VuhUnitType::V3C_CAD:
    return stream << "V3C_CAD";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, AiAttributeTypeId x) -> std::ostream & {
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
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
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

auto operator<<(std::ostream &stream, VpsExtensionType x) -> std::ostream & {
  switch (x) {
  case VpsExtensionType::VPS_EXT_UNSPECIFIED:
    return stream << "VPS_EXT_UNSPECIFIED";
  case VpsExtensionType::VPS_EXT_MIV:
    return stream << "VPS_EXT_MIV";
  case VpsExtensionType::VPS_EXT_PACKED:
    return stream << "VPS_EXT_PACKED";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto ProfileTierLevel::ptl_num_sub_profiles() const -> uint8_t {
  VERIFY_V3CBITSTREAM(m_subProfileIdcs.size() <= UINT8_MAX);
  return static_cast<uint8_t>(m_subProfileIdcs.size());
}

auto ProfileTierLevel::ptl_sub_profile_idc(uint8_t i) const -> uint64_t {
  VERIFY_V3CBITSTREAM(i < m_subProfileIdcs.size());
  return m_subProfileIdcs[i];
}

auto ProfileTierLevel::ptl_profile_toolset_constraints_information() const
    -> const ProfileToolsetConstraintsInformation & {
  VERIFY_V3CBITSTREAM(ptl_toolset_constraints_present_flag());
  VERIFY_V3CBITSTREAM(m_ptl_profile_toolset_constraints_information.has_value());
  return *m_ptl_profile_toolset_constraints_information;
}

auto ProfileTierLevel::ptl_num_sub_profiles(uint8_t value) noexcept -> ProfileTierLevel & {
  m_subProfileIdcs = std::vector<uint64_t>(value, 0);
  return *this;
}

auto ProfileTierLevel::ptl_extended_sub_profile_flag(bool value) -> ProfileTierLevel & {
  m_ptl_extended_sub_profile_flag = value;
  for (auto x : m_subProfileIdcs) {
    PRECONDITION(ptl_extended_sub_profile_flag() || x <= UINT32_MAX);
  }
  return *this;
}

auto ProfileTierLevel::ptl_sub_profile_idc(uint8_t i, uint64_t value) -> ProfileTierLevel & {
  PRECONDITION(i < ptl_num_sub_profiles());
  PRECONDITION(ptl_extended_sub_profile_flag() || value <= UINT32_MAX);
  m_subProfileIdcs[i] = value;
  return *this;
}

auto ProfileTierLevel::ptl_profile_toolset_constraints_information(
    ProfileToolsetConstraintsInformation value) noexcept -> ProfileTierLevel & {
  ptl_toolset_constraints_present_flag(true);
  m_ptl_profile_toolset_constraints_information.emplace(value);
  return *this;
}

auto operator<<(std::ostream &stream, const ProfileTierLevel &x) -> std::ostream & {
  fmt::print(stream, "ptl_tier_flag={}\n", x.ptl_tier_flag());
  fmt::print(stream, "ptl_profile_codec_group_idc={}\n", x.ptl_profile_codec_group_idc());
  fmt::print(stream, "ptl_profile_toolset_idc={}\n", x.ptl_profile_toolset_idc());
  fmt::print(stream, "ptl_profile_reconstruction_idc={}\n", x.ptl_profile_reconstruction_idc());
  fmt::print(stream, "ptl_max_decodes_idc={}\n", x.ptl_max_decodes_idc());
  fmt::print(stream, "ptl_level_idc={}\n", x.ptl_level_idc());
  fmt::print(stream, "ptl_num_sub_profiles={}\n", x.ptl_num_sub_profiles());
  fmt::print(stream, "ptl_extended_sub_profile_flag={}\n", x.ptl_extended_sub_profile_flag());

  for (uint8_t i = 0; i < x.ptl_num_sub_profiles(); ++i) {
    fmt::print(stream, "ptl_sub_profile_idc[ {} ]={}\n", i, x.ptl_sub_profile_idc(i));
  }
  fmt::print(stream, "ptl_toolset_constraints_present_flag={}\n",
             x.ptl_toolset_constraints_present_flag());
  if (x.ptl_toolset_constraints_present_flag()) {
    stream << x.ptl_profile_toolset_constraints_information();
  }
  return stream;
}

auto ProfileTierLevel::operator==(const ProfileTierLevel &other) const noexcept -> bool {
  return ptl_tier_flag() == other.ptl_tier_flag() &&
         ptl_profile_codec_group_idc() == other.ptl_profile_codec_group_idc() &&
         ptl_profile_toolset_idc() == other.ptl_profile_toolset_idc() &&
         ptl_profile_reconstruction_idc() == other.ptl_profile_reconstruction_idc() &&
         ptl_max_decodes_idc() == other.ptl_max_decodes_idc() &&
         ptl_level_idc() == other.ptl_level_idc() &&
         ptl_extended_sub_profile_flag() == other.ptl_extended_sub_profile_flag() &&
         m_subProfileIdcs == other.m_subProfileIdcs &&
         ptl_toolset_constraints_present_flag() == other.ptl_toolset_constraints_present_flag() &&
         (ptl_toolset_constraints_present_flag()
              ? (ptl_profile_toolset_constraints_information() ==
                 other.ptl_profile_toolset_constraints_information())
              : true);
}

auto ProfileTierLevel::operator!=(const ProfileTierLevel &other) const noexcept -> bool {
  return !operator==(other);
}

auto ProfileTierLevel::decodeFrom(Common::InputBitstream &bitstream) -> ProfileTierLevel {
  auto x = ProfileTierLevel{};
  x.ptl_tier_flag(bitstream.getFlag());
  x.ptl_profile_codec_group_idc(bitstream.readBits<PtlProfileCodecGroupIdc>(7));
  x.ptl_profile_toolset_idc(bitstream.readBits<PtlProfileToolsetIdc>(8));
  x.ptl_profile_reconstruction_idc(bitstream.readBits<PtlProfileReconstructionIdc>(8));
  bitstream.getUint16();
  x.ptl_max_decodes_idc(bitstream.readBits<PtlMaxDecodesIdc>(4));
  bitstream.readBits<uint16_t>(12);
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
  x.ptl_toolset_constraints_present_flag(bitstream.getFlag());
  if (x.ptl_toolset_constraints_present_flag()) {
    x.ptl_profile_toolset_constraints_information(
        ProfileToolsetConstraintsInformation::decodeFrom(bitstream));
  }
  return x;
}

void ProfileTierLevel::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(ptl_tier_flag());
  bitstream.writeBits(ptl_profile_codec_group_idc(), 7);
  bitstream.writeBits(ptl_profile_toolset_idc(), 8);
  bitstream.writeBits(ptl_profile_reconstruction_idc(), 8);
  constexpr auto ptl_reserved_zero_16bits = 0;
  bitstream.putUint16(ptl_reserved_zero_16bits);
  bitstream.writeBits(ptl_max_decodes_idc(), 4);
  constexpr auto ptl_reserved_0xfff_12bits = 0xFFF;
  bitstream.writeBits(ptl_reserved_0xfff_12bits, 12);
  bitstream.writeBits(ptl_level_idc(), 8);
  bitstream.writeBits(ptl_num_sub_profiles(), 6);
  bitstream.putFlag(ptl_extended_sub_profile_flag());
  for (uint8_t i = 0; i < ptl_num_sub_profiles(); ++i) {
    if (ptl_extended_sub_profile_flag()) {
      bitstream.putUint64(ptl_sub_profile_idc(i));
    } else {
      PRECONDITION(ptl_sub_profile_idc(i) <= UINT32_MAX);
      bitstream.putUint32(static_cast<uint32_t>(ptl_sub_profile_idc(i)));
    }
  }
  bitstream.putFlag(ptl_toolset_constraints_present_flag());
  if (ptl_toolset_constraints_present_flag()) {
    ptl_profile_toolset_constraints_information().encodeTo(bitstream);
  }
}

auto ProfileTierLevel::profile() const -> std::string {
  std::ostringstream stream;

  fmt::print(stream, "{} {}", ptl_profile_codec_group_idc(), ptl_profile_toolset_idc());

  if (ptl_toolset_constraints_present_flag()) {
    const auto &ptci = ptl_profile_toolset_constraints_information();

    if (ptci.ptc_restricted_geometry_flag()) {
      fmt::print(stream, " Restricted Geometry");
    }
    if (ptci.ptc_one_v3c_frame_only_flag()) {
      fmt::print(stream, " Still");
    }
  }

  if (ptl_profile_reconstruction_idc() != PtlProfileReconstructionIdc::Rec_Unconstrained) {
    fmt::print(stream, " {}", ptl_profile_reconstruction_idc());
  }
  return stream.str();
}

auto OccupancyInformation::printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream & {
  fmt::print(stream, "oi_occupancy_codec_id[ {} ]={}\n", atlasId, oi_occupancy_codec_id());
  fmt::print(stream, "oi_lossy_occupancy_compression_threshold[ {} ]={}\n", atlasId,
             oi_lossy_occupancy_compression_threshold());
  fmt::print(stream, "oi_occupancy_2d_bit_depth_minus1[ {} ]={}\n", atlasId,
             oi_occupancy_2d_bit_depth_minus1());
  fmt::print(stream, "oi_occupancy_MSB_align_flag[ {} ]={}\n", atlasId,
             oi_occupancy_MSB_align_flag());
  return stream;
}

auto OccupancyInformation::operator==(const OccupancyInformation &other) const noexcept -> bool {
  return oi_occupancy_codec_id() == other.oi_occupancy_codec_id() &&
         oi_lossy_occupancy_compression_threshold() ==
             other.oi_lossy_occupancy_compression_threshold() &&
         oi_occupancy_2d_bit_depth_minus1() == other.oi_occupancy_2d_bit_depth_minus1() &&
         oi_occupancy_MSB_align_flag() == other.oi_occupancy_MSB_align_flag();
}

auto OccupancyInformation::operator!=(const OccupancyInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto OccupancyInformation::decodeFrom(Common::InputBitstream &bitstream) -> OccupancyInformation {
  auto x = OccupancyInformation{};
  x.oi_occupancy_codec_id(bitstream.getUint8());
  x.oi_lossy_occupancy_compression_threshold(bitstream.getUint8());
  x.oi_occupancy_2d_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
  x.oi_occupancy_MSB_align_flag(bitstream.getFlag());
  return x;
}

void OccupancyInformation::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint8(oi_occupancy_codec_id());
  bitstream.putUint8(oi_lossy_occupancy_compression_threshold());
  bitstream.writeBits(oi_occupancy_2d_bit_depth_minus1(), 5);
  bitstream.putFlag(oi_occupancy_MSB_align_flag());
}

auto GeometryInformation::printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream & {
  fmt::print(stream, "gi_geometry_codec_id[ {} ]={}\n", atlasId, gi_geometry_codec_id());
  fmt::print(stream, "gi_geometry_2d_bit_depth_minus1[ {} ]={}\n", atlasId,
             gi_geometry_2d_bit_depth_minus1());
  fmt::print(stream, "gi_geometry_MSB_align_flag[ {} ]={}\n", atlasId,
             gi_geometry_MSB_align_flag());
  fmt::print(stream, "gi_geometry_3d_coordinates_bit_depth_minus1[ {} ]={}\n", atlasId,
             gi_geometry_3d_coordinates_bit_depth_minus1());
  return stream;
}

auto GeometryInformation::operator==(const GeometryInformation &other) const noexcept -> bool {
  return gi_geometry_codec_id() == other.gi_geometry_codec_id() &&
         gi_geometry_2d_bit_depth_minus1() == other.gi_geometry_2d_bit_depth_minus1() &&
         gi_geometry_MSB_align_flag() == other.gi_geometry_MSB_align_flag() &&
         gi_geometry_3d_coordinates_bit_depth_minus1() ==
             other.gi_geometry_3d_coordinates_bit_depth_minus1();
}

auto GeometryInformation::operator!=(const GeometryInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto GeometryInformation::decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                                     AtlasId atlasId) -> GeometryInformation {
  auto x = GeometryInformation{};
  x.gi_geometry_codec_id(bitstream.getUint8());
  x.gi_geometry_2d_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
  x.gi_geometry_MSB_align_flag(bitstream.getFlag());
  x.gi_geometry_3d_coordinates_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
  VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasId));
  return x;
}

void GeometryInformation::encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                                   AtlasId atlasId) const {
  bitstream.putUint8(gi_geometry_codec_id());
  bitstream.writeBits(gi_geometry_2d_bit_depth_minus1(), 5);
  bitstream.putFlag(gi_geometry_MSB_align_flag());
  bitstream.writeBits(gi_geometry_3d_coordinates_bit_depth_minus1(), 5);
  PRECONDITION(!vps.vps_auxiliary_video_present_flag(atlasId));
}

auto AttributeInformation::ai_attribute_count() const noexcept -> uint8_t {
  return static_cast<uint8_t>(m_aiAttributes.size());
}

auto AttributeInformation::ai_attribute_type_id(uint8_t attrIdx) const -> AiAttributeTypeId {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  return m_aiAttributes[attrIdx].ai_attribute_type_id;
}

auto AttributeInformation::ai_attribute_codec_id(uint8_t attrIdx) const -> uint8_t {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  return m_aiAttributes[attrIdx].ai_attribute_codec_id;
}

auto AttributeInformation::ai_attribute_dimension_minus1(uint8_t attrIdx) const -> uint8_t {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  return m_aiAttributes[attrIdx].ai_attribute_dimension_minus1;
}

auto AttributeInformation::ai_attribute_2d_bit_depth_minus1(uint8_t attrIdx) const -> uint8_t {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  return m_aiAttributes[attrIdx].ai_attribute_2d_bit_depth_minus1;
}

auto AttributeInformation::ai_attribute_MSB_align_flag(uint8_t attrIdx) const -> bool {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  return m_aiAttributes[attrIdx].ai_attribute_MSB_align_flag;
}

auto AttributeInformation::ai_attribute_map_absolute_coding_persistence_flag(uint8_t attrIdx) const
    -> bool {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  VERIFY_V3CBITSTREAM(
      m_aiAttributes[attrIdx].ai_attribute_map_absolute_coding_persistence_flag.has_value());
  return *m_aiAttributes[attrIdx].ai_attribute_map_absolute_coding_persistence_flag;
}

auto AttributeInformation::ai_attribute_count(uint8_t value) -> AttributeInformation & {
  m_aiAttributes.resize(value);
  return *this;
}

auto AttributeInformation::ai_attribute_type_id(uint8_t attrIdx, AiAttributeTypeId value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  m_aiAttributes[attrIdx].ai_attribute_type_id = value;
  return *this;
}

auto AttributeInformation::ai_attribute_codec_id(uint8_t attrIdx, uint8_t value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  m_aiAttributes[attrIdx].ai_attribute_codec_id = value;
  return *this;
}

auto AttributeInformation::ai_attribute_map_absolute_coding_persistence_flag(uint8_t attrIdx,
                                                                             bool value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  m_aiAttributes[attrIdx].ai_attribute_map_absolute_coding_persistence_flag = value;
  return *this;
}

auto AttributeInformation::ai_attribute_dimension_minus1(uint8_t attrIdx, uint8_t value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  m_aiAttributes[attrIdx].ai_attribute_dimension_minus1 = value;
  return *this;
}

auto AttributeInformation::ai_attribute_2d_bit_depth_minus1(uint8_t attrIdx, uint8_t value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  m_aiAttributes[attrIdx].ai_attribute_2d_bit_depth_minus1 = value;
  return *this;
}

auto AttributeInformation::ai_attribute_MSB_align_flag(uint8_t attrIdx, bool value)
    -> AttributeInformation & {
  VERIFY_V3CBITSTREAM(attrIdx < ai_attribute_count());
  m_aiAttributes[attrIdx].ai_attribute_MSB_align_flag = value;
  return *this;
}

auto AttributeInformation::printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream & {
  fmt::print(stream, "ai_attribute_count[ {} ]={}\n", atlasId, ai_attribute_count());
  for (uint8_t i = 0; i < ai_attribute_count(); ++i) {
    fmt::print(stream, "ai_attribute_type_id[ {} ][ {} ]={}\n", atlasId, i,
               ai_attribute_type_id(i));
    fmt::print(stream, "ai_attribute_codec_id[ {} ][ {} ]={}\n", atlasId, i,
               ai_attribute_codec_id(i));
    if (m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag) {
      fmt::print(stream, "ai_attribute_map_absolute_coding_persistence_flag[ {} ][ {} ]={}\n",
                 atlasId, i, *m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag);
    }
    fmt::print(stream, "ai_attribute_dimension_minus1[ {} ][ {} ]={}\n", atlasId, i,
               ai_attribute_dimension_minus1(i));
    fmt::print(stream, "ai_attribute_2d_bit_depth_minus1[ {} ][ {} ]={}\n", atlasId, i,
               ai_attribute_2d_bit_depth_minus1(i));
    fmt::print(stream, "ai_attribute_MSB_align_flag[ {} ][ {} ]={}\n", atlasId, i,
               ai_attribute_MSB_align_flag(i));
  }
  return stream;
}

auto AttributeInformation::operator==(const AttributeInformation &other) const -> bool {
  if (ai_attribute_count() != other.ai_attribute_count()) {
    return false;
  }
  for (uint8_t i = 0; i < ai_attribute_count(); ++i) {
    if (ai_attribute_type_id(i) != other.ai_attribute_type_id(i) ||
        ai_attribute_codec_id(i) != other.ai_attribute_codec_id(i) ||
        m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag !=
            other.m_aiAttributes[i].ai_attribute_map_absolute_coding_persistence_flag ||
        ai_attribute_dimension_minus1(i) != other.ai_attribute_dimension_minus1(i) ||
        ai_attribute_2d_bit_depth_minus1(i) != other.ai_attribute_2d_bit_depth_minus1(i) ||
        ai_attribute_MSB_align_flag(i) != other.ai_attribute_MSB_align_flag(i)) {
      return false;
    }
  }
  return true;
}

auto AttributeInformation::operator!=(const AttributeInformation &other) const -> bool {
  return !operator==(other);
}

auto AttributeInformation::decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                                      AtlasId atlasId) -> AttributeInformation {
  auto x = AttributeInformation{};
  x.ai_attribute_count(bitstream.readBits<uint8_t>(7));
  for (uint8_t i = 0; i < x.ai_attribute_count(); ++i) {
    x.ai_attribute_type_id(i, bitstream.readBits<AiAttributeTypeId>(4));
    x.ai_attribute_codec_id(i, bitstream.getUint8());

    VERIFY_MIVBITSTREAM(!vps.vps_auxiliary_video_present_flag(atlasId));

    if (vps.vps_map_count_minus1(atlasId) > 0) {
      x.ai_attribute_map_absolute_coding_persistence_flag(i, bitstream.getFlag());
    }

    x.ai_attribute_dimension_minus1(i, bitstream.readBits<uint8_t>(6));

    const auto ai_attribute_dimension_partitions_minus1 = bitstream.readBits<uint8_t>(6);
    VERIFY_MIVBITSTREAM(ai_attribute_dimension_partitions_minus1 == 0);

    x.ai_attribute_2d_bit_depth_minus1(i, bitstream.readBits<uint8_t>(5));
    x.ai_attribute_MSB_align_flag(i, bitstream.getFlag());
  }
  return x;
}

void AttributeInformation::encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                                    AtlasId atlasId) const {
  bitstream.writeBits(ai_attribute_count(), 7);
  for (uint8_t i = 0; i < ai_attribute_count(); ++i) {
    bitstream.writeBits(ai_attribute_type_id(i), 4);
    bitstream.writeBits(ai_attribute_codec_id(i), 8);

    if (vps.vps_map_count_minus1(atlasId) > 0) {
      bitstream.putFlag(ai_attribute_map_absolute_coding_persistence_flag(i));
    }

    VERIFY_V3CBITSTREAM(ai_attribute_dimension_minus1(i) < 64);
    bitstream.writeBits(ai_attribute_dimension_minus1(i), 6);

    constexpr auto ai_attribute_dimension_partitions_minus1 = 0;
    bitstream.writeBits(ai_attribute_dimension_partitions_minus1, 6);

    VERIFY_V3CBITSTREAM(ai_attribute_2d_bit_depth_minus1(i) < 32);
    bitstream.writeBits(ai_attribute_2d_bit_depth_minus1(i), 5);
    bitstream.putFlag(ai_attribute_MSB_align_flag(i));
  }
}
auto operator<<(std::ostream &stream, const ProfileToolsetConstraintsInformation &x)
    -> std::ostream & {
  fmt::print(stream, "ptc_one_v3c_frame_only_flag={}\n", x.ptc_one_v3c_frame_only_flag());
  fmt::print(stream, "ptc_eom_constraint_flag={}\n", x.ptc_eom_constraint_flag());
  fmt::print(stream, "ptc_max_map_count_minus1={}\n", x.ptc_max_map_count_minus1());
  fmt::print(stream, "ptc_max_atlas_count_minus1={}\n", x.ptc_max_atlas_count_minus1());
  fmt::print(stream, "ptc_multiple_map_streams_constraint_flag={}\n",
             x.ptc_multiple_map_streams_constraint_flag());
  fmt::print(stream, "ptc_plr_constraint_flag={}\n", x.ptc_plr_constraint_flag());
  fmt::print(stream, "ptc_attribute_max_dimension_minus1={}\n",
             x.ptc_attribute_max_dimension_minus1());
  fmt::print(stream, "ptc_attribute_max_dimension_partitions_minus1={}\n",
             x.ptc_attribute_max_dimension_partitions_minus1());
  fmt::print(stream, "ptc_no_eight_orientations_constraint_flag={}\n",
             x.ptc_no_eight_orientations_constraint_flag());
  fmt::print(stream, "ptc_no_45degree_projection_patch_constraint_flag={}\n",
             x.ptc_no_45degree_projection_patch_constraint_flag());
  fmt::print(stream, "ptc_restricted_geometry_flag={}\n", x.ptc_restricted_geometry_flag());
  fmt::print(stream, "ptc_num_reserved_constraint_bytes={}\n",
             x.ptc_num_reserved_constraint_bytes());
  return stream;
}

auto ProfileToolsetConstraintsInformation::operator==(
    const ProfileToolsetConstraintsInformation &other) const noexcept -> bool {
  return ptc_one_v3c_frame_only_flag() == other.ptc_one_v3c_frame_only_flag() &&
         ptc_eom_constraint_flag() == other.ptc_eom_constraint_flag() &&
         ptc_max_map_count_minus1() == other.ptc_max_map_count_minus1() &&
         ptc_max_atlas_count_minus1() == other.ptc_max_atlas_count_minus1() &&
         ptc_multiple_map_streams_constraint_flag() ==
             other.ptc_multiple_map_streams_constraint_flag() &&
         ptc_plr_constraint_flag() == other.ptc_plr_constraint_flag() &&
         ptc_attribute_max_dimension_minus1() == other.ptc_attribute_max_dimension_minus1() &&
         ptc_attribute_max_dimension_partitions_minus1() ==
             other.ptc_attribute_max_dimension_partitions_minus1() &&
         ptc_no_eight_orientations_constraint_flag() ==
             other.ptc_no_eight_orientations_constraint_flag() &&
         ptc_no_45degree_projection_patch_constraint_flag() ==
             other.ptc_no_45degree_projection_patch_constraint_flag() &&
         ptc_restricted_geometry_flag() == other.ptc_restricted_geometry_flag() &&
         ptc_num_reserved_constraint_bytes() == other.ptc_num_reserved_constraint_bytes();
}

auto ProfileToolsetConstraintsInformation::operator!=(
    const ProfileToolsetConstraintsInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto ProfileToolsetConstraintsInformation::decodeFrom(Common::InputBitstream &bitstream)
    -> ProfileToolsetConstraintsInformation {
  auto x = ProfileToolsetConstraintsInformation{};

  x.ptc_one_v3c_frame_only_flag(bitstream.getFlag());
  x.ptc_eom_constraint_flag(bitstream.getFlag());
  x.ptc_max_map_count_minus1(bitstream.readBits<uint8_t>(4));
  x.ptc_max_atlas_count_minus1(bitstream.readBits<uint8_t>(4));
  x.ptc_multiple_map_streams_constraint_flag(bitstream.getFlag());
  x.ptc_plr_constraint_flag(bitstream.getFlag());
  x.ptc_attribute_max_dimension_minus1(bitstream.readBits<uint8_t>(6));
  x.ptc_attribute_max_dimension_partitions_minus1(bitstream.readBits<uint8_t>(6));
  x.ptc_no_eight_orientations_constraint_flag(bitstream.getFlag());
  x.ptc_no_45degree_projection_patch_constraint_flag(bitstream.getFlag());
  x.ptc_restricted_geometry_flag(bitstream.getFlag());
  bitstream.readBits<uint8_t>(5);
  x.ptc_num_reserved_constraint_bytes(bitstream.getUint8());
  // 23090-12 restrictions:
  //   * ptc_num_reserved_constraint_bytes[ ] == 0
  LIMITATION(x.ptc_num_reserved_constraint_bytes() == 0);
  return x;
}

void ProfileToolsetConstraintsInformation::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(ptc_one_v3c_frame_only_flag());
  bitstream.putFlag(ptc_eom_constraint_flag());
  bitstream.writeBits(ptc_max_map_count_minus1(), 4);
  bitstream.writeBits(ptc_max_atlas_count_minus1(), 4);
  bitstream.putFlag(ptc_multiple_map_streams_constraint_flag());
  bitstream.putFlag(ptc_plr_constraint_flag());
  bitstream.writeBits(ptc_attribute_max_dimension_minus1(), 6);
  bitstream.writeBits(ptc_attribute_max_dimension_partitions_minus1(), 6);
  bitstream.putFlag(ptc_no_eight_orientations_constraint_flag());
  bitstream.putFlag(ptc_no_45degree_projection_patch_constraint_flag());
  bitstream.putFlag(ptc_restricted_geometry_flag());
  constexpr auto ptl_reserved_zero_5bits = 0;
  bitstream.writeBits(ptl_reserved_zero_5bits, 5);
  // 23090-12 restrictions:
  //   * ptc_num_reserved_constraint_bytes[ ] == 0
  bitstream.putUint8(ptc_num_reserved_constraint_bytes());
}

auto PackingInformation::pin_codec_id() const noexcept -> uint8_t { return m_pin_codec_id; }

auto PackingInformation::pin_regions_count_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pin_attribute_count || m_pin_geometry_present_flag ||
                      m_pin_occupancy_present_flag);
  return m_pin_regions_count_minus1;
}

auto PackingInformation::pin_region_tile_id(size_t i) const -> uint8_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_tile_id;
}

auto PackingInformation::pin_region_type_id_minus2(size_t i) const -> uint8_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_type_id_minus2;
}

auto PackingInformation::pinRegionTypeId(size_t i) const -> VuhUnitType {
  return static_cast<VuhUnitType>(pin_region_type_id_minus2(i) + 2);
}

auto PackingInformation::pin_region_top_left_x(size_t i) const -> uint16_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_top_left_x;
}

auto PackingInformation::pin_region_top_left_y(size_t i) const -> uint16_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_top_left_y;
}

auto PackingInformation::pin_region_width_minus1(size_t i) const -> uint16_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_width_minus1;
}

auto PackingInformation::pin_region_height_minus1(size_t i) const -> uint16_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_height_minus1;
}

auto PackingInformation::pin_region_unpack_top_left_x(size_t i) const -> uint16_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_unpack_top_left_x;
}

auto PackingInformation::pin_region_unpack_top_left_y(size_t i) const -> uint16_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_unpack_top_left_y;
}

auto PackingInformation::pin_region_rotation_flag(size_t i) const -> bool {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_rotation_flag;
}

auto PackingInformation::pin_region_map_index(size_t i) const -> uint8_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  VERIFY_V3CBITSTREAM(m_pinRegions[i].pin_region_map_index.has_value());
  return m_pinRegions[i].pin_region_map_index.value();
}

auto PackingInformation::pin_region_auxiliary_data_flag(size_t i) const -> bool {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1() &&
                      m_pinRegions[i].pin_region_auxiliary_data_flag);
  return m_pinRegions[i].pin_region_auxiliary_data_flag.value();
}

auto PackingInformation::pin_region_attr_index(size_t i) const -> uint8_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1() && m_pinRegions[i].pin_region_attr_index);
  return m_pinRegions[i].pin_region_attr_index.value();
}

auto PackingInformation::pin_region_attr_partition_index(size_t i) const -> uint8_t {
  VERIFY_V3CBITSTREAM(i <= pin_regions_count_minus1());
  return m_pinRegions[i].pin_region_attr_partition_index.value_or(0);
}

auto PackingInformation::pin_occupancy_present_flag() const -> bool {
  return m_pin_occupancy_present_flag;
}

auto PackingInformation::pin_geometry_present_flag() const -> bool {
  return m_pin_geometry_present_flag;
}

auto PackingInformation::pin_attribute_present_flag() const -> bool {
  return m_pin_attribute_present_flag;
}

auto PackingInformation::pin_occupancy_2d_bit_depth_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pin_occupancy_2d_bit_depth_minus1.has_value());
  return *m_pin_occupancy_2d_bit_depth_minus1;
}

auto PackingInformation::pin_occupancy_MSB_align_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_pin_occupancy_MSB_align_flag.has_value());
  return *m_pin_occupancy_MSB_align_flag;
}

auto PackingInformation::pin_lossy_occupancy_compression_threshold() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pin_lossy_occupancy_compression_threshold.has_value());
  return *m_pin_lossy_occupancy_compression_threshold;
}

auto PackingInformation::pin_geometry_2d_bit_depth_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pin_geometry_2d_bit_depth_minus1.has_value());
  return *m_pin_geometry_2d_bit_depth_minus1;
}

auto PackingInformation::pin_geometry_MSB_align_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_pin_geometry_MSB_align_flag.has_value());
  return *m_pin_geometry_MSB_align_flag;
}

auto PackingInformation::pin_geometry_3d_coordinates_bit_depth_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pin_geometry_3d_coordinates_bit_depth_minus1.has_value());
  return *m_pin_geometry_3d_coordinates_bit_depth_minus1;
}

auto PackingInformation::pin_attribute_count() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pin_attribute_count.has_value());
  return *m_pin_attribute_count;
}

auto PackingInformation::pin_attribute_type_id(size_t i) const -> AiAttributeTypeId {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return m_pinAttributeInformation->at(i).pin_attribute_type_id;
}

auto PackingInformation::pin_attribute_2d_bit_depth_minus1(size_t i) const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return m_pinAttributeInformation->at(i).pin_attribute_2d_bit_depth_minus1;
}

auto PackingInformation::pin_attribute_MSB_align_flag(size_t i) const -> bool {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return m_pinAttributeInformation->at(i).pin_attribute_MSB_align_flag;
}

auto PackingInformation::pin_attribute_map_absolute_coding_persistence_flag(size_t i) const
    -> bool {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return m_pinAttributeInformation->at(i).pin_attribute_map_absolute_coding_persistence_flag;
}

auto PackingInformation::pin_attribute_dimension_minus1(size_t i) const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return m_pinAttributeInformation->at(i).pin_attribute_dimension_minus1;
}

auto PackingInformation::pin_attribute_dimension_partitions_minus1(size_t i) const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return *m_pinAttributeInformation->at(i).pin_attribute_dimension_partitions_minus1;
}

auto PackingInformation::pin_attribute_partition_channels_minus1(size_t i, uint8_t l) const
    -> uint8_t {
  VERIFY_MIVBITSTREAM(m_pinAttributeInformation.has_value());
  PRECONDITION(i < m_pinAttributeInformation.value().size());
  return m_pinAttributeInformation->at(i).pin_attribute_partition_channels_minus1->at(l);
}

auto PackingInformation::printTo(std::ostream &stream, AtlasId j) const -> std::ostream & {
  fmt::print(stream, "pin_codec_id[ {} ]={}\n", j, pin_codec_id());
  fmt::print(stream, "pin_occupancy_present_flag[ {} ]={}\n", j, pin_occupancy_present_flag());
  fmt::print(stream, "pin_geometry_present_flag[ {} ]={}\n", j, pin_geometry_present_flag());
  fmt::print(stream, "pin_attribute_present_flag[ {} ]={}\n", j, pin_attribute_present_flag());
  if (pin_occupancy_present_flag()) {
    fmt::print(stream, "pin_occupancy_2d_bit_depth_minus1[ {} ]={}\n", j,
               pin_occupancy_2d_bit_depth_minus1());
    fmt::print(stream, "pin_occupancy_MSB_align_flag[ {} ]={}\n", j,
               pin_occupancy_MSB_align_flag());
    fmt::print(stream, "pin_lossy_occupancy_compression_threshold[ {} ]={}\n", j,
               pin_lossy_occupancy_compression_threshold());
  }
  if (pin_geometry_present_flag()) {
    fmt::print(stream, "pin_geometry_2d_bit_depth_minus1[ {} ]={}\n", j,
               pin_geometry_2d_bit_depth_minus1());
    fmt::print(stream, "pin_geometry_MSB_align_flag[ {} ]={}\n", j, pin_geometry_MSB_align_flag());
    fmt::print(stream, "pin_geometry_3d_coordinates_bit_depth_minus1[ {} ]={}\n", j,
               pin_geometry_3d_coordinates_bit_depth_minus1());
  }
  if (pin_attribute_present_flag()) {
    fmt::print(stream, "pin_attribute_count[ {} ]={}\n", j, pin_attribute_count());
    for (size_t i = 0; i < pin_attribute_count(); i++) {
      fmt::print(stream, "pin_attribute_type_id[ {} ][ {} ]={}\n", j, i, pin_attribute_type_id(i));
      fmt::print(stream, "pin_attribute_2d_bit_depth_minus1[ {} ][ {} ]={}\n", j, i,
                 pin_attribute_2d_bit_depth_minus1(i));
      fmt::print(stream, "pin_attribute_MSB_align_flag[ {} ][ {} ]={}\n", j, i,
                 pin_attribute_MSB_align_flag(i));
      fmt::print(stream, "pin_attribute_map_absolute_coding_persistence_flag[ {} ][ {} ]={}\n", j,
                 i, pin_attribute_map_absolute_coding_persistence_flag(i));
      fmt::print(stream, "pin_attribute_dimension_minus1[ {} ][ {} ]={}\n", j, i,
                 pin_attribute_dimension_minus1(i));
      auto d = pin_attribute_dimension_minus1(i);
      uint8_t m = 0;
      if (d != 0) {
        fmt::print(stream, "pin_attribute_dimension_partitions_minus1[ {} ][ {} ]={}\n", j, i,
                   pin_attribute_dimension_partitions_minus1(i));
        m = pin_attribute_dimension_partitions_minus1(i);
      }
      for (uint8_t k = 0; k < m; k++) {
        uint8_t n = 0;
        if (k + d != m) {
          fmt::print(stream, "pin_attribute_partition_channels_minus1[ {} ][ {} ] [ {} ]={}\n", j,
                     i, k, pin_attribute_partition_channels_minus1(i, k));
          n = pin_attribute_partition_channels_minus1(i, k);
        }
        d -= n + 1;
      }
    }
  }

  fmt::print(stream, "pin_regions_count_minus1[ {} ]={}\n", j, pin_regions_count_minus1());
  for (size_t i = 0; i <= pin_regions_count_minus1(); ++i) {
    fmt::print(stream, "pin_region_tile_id[ {} ][ {} ]={}\n", j, i, pin_region_tile_id(i));
    fmt::print(stream, "pin_region_type_id_minus2[ {} ][ {} ]={} ({})\n", j, i,
               pin_region_type_id_minus2(i), pinRegionTypeId(i));
    fmt::print(stream, "pin_region_top_left_x[ {} ][ {} ]={}\n", j, i, pin_region_top_left_x(i));
    fmt::print(stream, "pin_region_top_left_y[ {} ][ {} ]={}\n", j, i, pin_region_top_left_y(i));
    fmt::print(stream, "pin_region_width_minus1[ {} ][ {} ]={}\n", j, i,
               pin_region_width_minus1(i));
    fmt::print(stream, "pin_region_height_minus1[ {} ][ {} ]={}\n", j, i,
               pin_region_height_minus1(i));
    fmt::print(stream, "pin_region_unpack_top_left_x[ {} ][ {} ]={}\n", j, i,
               pin_region_unpack_top_left_x(i));
    fmt::print(stream, "pin_region_unpack_top_left_y[ {} ][ {} ]={}\n", j, i,
               pin_region_unpack_top_left_y(i));
    fmt::print(stream, "pin_region_rotation_flag[ {} ][ {} ]={}\n", j, i,
               pin_region_rotation_flag(i));
    if (pinRegionTypeId(i) == VuhUnitType::V3C_AVD || pinRegionTypeId(i) == VuhUnitType::V3C_GVD) {
      fmt::print(stream, "pin_region_map_index[ {} ][ {} ]={}\n", j, i, pin_region_map_index(i));
      fmt::print(stream, "pin_region_auxiliary_data_flag[ {} ][ {} ]={}\n", j, i,
                 pin_region_auxiliary_data_flag(i));
    }
    if (pinRegionTypeId(i) == VuhUnitType::V3C_AVD) {
      fmt::print(stream, "pin_region_attr_index[ {} ][ {} ]={}\n", j, i, pin_region_attr_index(i));
      const auto k = pin_region_attr_index(i);

      if (pin_attribute_dimension_minus1(k) > 0) {
        fmt::print(stream, "pin_region_attr_partition_index[ {} ][ {} ]={}\n", j, i,
                   pin_region_attr_partition_index(i));
      }
    }
  }
  return stream;
}

auto PackingInformation::operator==(const PackingInformation &other) const noexcept -> bool {
  return (m_pin_codec_id == other.m_pin_codec_id) &&
         (m_pin_occupancy_present_flag == other.m_pin_occupancy_present_flag) &&
         (m_pin_geometry_present_flag == other.m_pin_geometry_present_flag) &&
         (m_pin_attribute_present_flag == other.m_pin_attribute_present_flag) &&
         (m_pin_occupancy_2d_bit_depth_minus1 == other.m_pin_occupancy_2d_bit_depth_minus1) &&
         (m_pin_occupancy_MSB_align_flag == other.m_pin_occupancy_MSB_align_flag) &&
         (m_pin_lossy_occupancy_compression_threshold ==
          other.m_pin_lossy_occupancy_compression_threshold) &&
         (m_pin_geometry_2d_bit_depth_minus1 == other.m_pin_geometry_2d_bit_depth_minus1) &&
         (m_pin_geometry_MSB_align_flag == other.m_pin_geometry_MSB_align_flag) &&
         (m_pin_geometry_3d_coordinates_bit_depth_minus1 ==
          other.m_pin_geometry_3d_coordinates_bit_depth_minus1) &&
         (m_pin_attribute_count == other.m_pin_attribute_count) &&
         (m_pinAttributeInformation == other.m_pinAttributeInformation) &&
         (m_pinRegions == other.m_pinRegions);
}

auto PackingInformation::operator!=(const PackingInformation &other) const noexcept -> bool {
  return !operator==(other);
}

auto PackingInformation::decodeFrom(Common::InputBitstream &bitstream) -> PackingInformation {
  PackingInformation result{};
  result.pin_codec_id(bitstream.getUint8());
  result.pin_occupancy_present_flag(bitstream.getFlag());
  result.pin_geometry_present_flag(bitstream.getFlag());
  result.pin_attribute_present_flag(bitstream.getFlag());

  if (result.pin_occupancy_present_flag()) {
    result.pin_occupancy_2d_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
    result.pin_occupancy_MSB_align_flag(bitstream.getFlag());
    result.pin_lossy_occupancy_compression_threshold(bitstream.getUint8());
  }
  if (result.pin_geometry_present_flag()) {
    result.pin_geometry_2d_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
    result.pin_geometry_MSB_align_flag(bitstream.getFlag());
    result.pin_geometry_3d_coordinates_bit_depth_minus1(bitstream.readBits<uint8_t>(5));
  }

  if (result.pin_attribute_present_flag()) {
    result.pin_attribute_count(bitstream.readBits<uint8_t>(7));
    for (size_t i = 0; i < result.pin_attribute_count(); i++) {
      result.pin_attribute_type_id(i, bitstream.readBits<AiAttributeTypeId>(4));
      result.pin_attribute_2d_bit_depth_minus1(i, bitstream.readBits<uint8_t>(5));
      result.pin_attribute_MSB_align_flag(i, bitstream.getFlag());
      result.pin_attribute_map_absolute_coding_persistence_flag(i, bitstream.getFlag());
      result.pin_attribute_dimension_minus1(i, bitstream.readBits<uint8_t>(6));
      auto d = result.pin_attribute_dimension_minus1(i);
      uint8_t m = 0;
      if (d == 0) {
        result.pin_attribute_dimension_partitions_minus1(i, 0);
      } else {
        result.pin_attribute_dimension_partitions_minus1(i, bitstream.readBits<uint8_t>(6));
        m = result.pin_attribute_dimension_partitions_minus1(i);
      }
      for (uint8_t k = 0; k < m; k++) {
        uint8_t n = 0;
        if (k + d == m) {
          result.pin_attribute_partition_channels_minus1(i, k, 0);
        } else {
          result.pin_attribute_partition_channels_minus1(i, k, bitstream.getUExpGolomb<uint8_t>());
          n = result.pin_attribute_partition_channels_minus1(i, k);
        }
        d -= n + 1;
      }
      result.pin_attribute_partition_channels_minus1(i, m, d);
    }
  }

  result.pin_regions_count_minus1(bitstream.getUExpGolomb<uint8_t>());
  for (size_t i = 0; i <= result.pin_regions_count_minus1(); ++i) {
    result.pin_region_tile_id(i, bitstream.getUint8());
    result.pin_region_type_id_minus2(i, bitstream.readBits<uint8_t>(2));
    result.pin_region_top_left_x(i, bitstream.getUint16());
    result.pin_region_top_left_y(i, bitstream.getUint16());
    result.pin_region_width_minus1(i, bitstream.getUint16());
    result.pin_region_height_minus1(i, bitstream.getUint16());
    result.pin_region_unpack_top_left_x(i, bitstream.getUint16());
    result.pin_region_unpack_top_left_y(i, bitstream.getUint16());
    result.pin_region_rotation_flag(i, bitstream.getFlag());
    if (result.pinRegionTypeId(i) == VuhUnitType::V3C_AVD ||
        result.pinRegionTypeId(i) == VuhUnitType::V3C_GVD) {
      result.pin_region_map_index(i, bitstream.readBits<uint8_t>(4));
      result.pin_region_auxiliary_data_flag(i, bitstream.getFlag());
    }
    if (result.pinRegionTypeId(i) == VuhUnitType::V3C_AVD) {
      result.pin_region_attr_index(i, bitstream.readBits<uint8_t>(7));
      const auto k = result.pin_region_attr_index(i);
      if (result.pin_attribute_dimension_minus1(k) > 0) {
        result.pin_region_attr_partition_index(i, bitstream.readBits<uint8_t>(5));
      }
    }
  }
  return result;
}

void PackingInformation::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint8(pin_codec_id());
  bitstream.putFlag(pin_occupancy_present_flag());
  bitstream.putFlag(pin_geometry_present_flag());
  bitstream.putFlag(pin_attribute_present_flag());
  if (pin_occupancy_present_flag()) {
    bitstream.writeBits(pin_occupancy_2d_bit_depth_minus1(), 5);
    bitstream.putFlag(pin_occupancy_MSB_align_flag());
    bitstream.putUint8(pin_lossy_occupancy_compression_threshold());
  }
  if (pin_geometry_present_flag()) {
    bitstream.writeBits(pin_geometry_2d_bit_depth_minus1(), 5);
    bitstream.putFlag(pin_geometry_MSB_align_flag());
    bitstream.writeBits(pin_geometry_3d_coordinates_bit_depth_minus1(), 5);
  }
  if (pin_attribute_present_flag()) {
    bitstream.writeBits(pin_attribute_count(), 7);
    for (size_t i = 0; i < pin_attribute_count(); i++) {
      bitstream.writeBits(pin_attribute_type_id(i), 4);
      bitstream.writeBits(pin_attribute_2d_bit_depth_minus1(i), 5);
      bitstream.putFlag(pin_attribute_MSB_align_flag(i));
      bitstream.putFlag(pin_attribute_map_absolute_coding_persistence_flag(i));
      bitstream.writeBits(pin_attribute_dimension_minus1(i), 6);
      auto d = pin_attribute_dimension_minus1(i);
      uint8_t m = 0;
      if (d != 0) {
        bitstream.writeBits(pin_attribute_dimension_partitions_minus1(i), 6);
        m = pin_attribute_dimension_partitions_minus1(i);
      }
      for (uint8_t k = 0; k < m; k++) {
        uint8_t n = 0;
        if (k + d != m) {
          bitstream.putUExpGolomb(pin_attribute_partition_channels_minus1(i, k));
          n = pin_attribute_partition_channels_minus1(i, k);
        }
        d -= n + 1;
      }
    }
  }

  bitstream.putUExpGolomb(pin_regions_count_minus1());
  for (size_t i = 0; i <= pin_regions_count_minus1(); ++i) {
    bitstream.putUint8(pin_region_tile_id(i));
    bitstream.writeBits(pin_region_type_id_minus2(i), 2);
    bitstream.putUint16(pin_region_top_left_x(i));
    bitstream.putUint16(pin_region_top_left_y(i));
    bitstream.putUint16(pin_region_width_minus1(i));
    bitstream.putUint16(pin_region_height_minus1(i));
    bitstream.putUint16(pin_region_unpack_top_left_x(i));
    bitstream.putUint16(pin_region_unpack_top_left_y(i));
    bitstream.putFlag(pin_region_rotation_flag(i));

    if ((pinRegionTypeId(i) == VuhUnitType::V3C_AVD) ||
        pinRegionTypeId(i) == VuhUnitType::V3C_GVD) {
      bitstream.writeBits(pin_region_map_index(i), 4);
      bitstream.putFlag(pin_region_auxiliary_data_flag(i));
    }
    if (pinRegionTypeId(i) == VuhUnitType::V3C_AVD) {
      bitstream.writeBits(pin_region_attr_index(i), 7);
      const auto k = pin_region_attr_index(i);
      if (pin_attribute_dimension_minus1(k) > 0) {
        bitstream.writeBits(pin_region_attr_partition_index(i), 5);
      }
    }
  }
}

auto GroupMapping::gm_group_id(size_t i) const -> uint8_t {
  VERIFY_MIVBITSTREAM(0 < gm_group_count());
  VERIFY_MIVBITSTREAM(i < m_gm_group_id.size());
  return m_gm_group_id[i];
}

auto GroupMapping::gm_group_id(size_t i, uint8_t value) -> GroupMapping & {
  VERIFY_MIVBITSTREAM(value < gm_group_count());
  if (m_gm_group_id.size() <= i) {
    m_gm_group_id.resize(i + 1, UINT8_MAX);
  }
  m_gm_group_id[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const GroupMapping &x) -> std::ostream & {
  fmt::print(stream, "gm_group_count={}\n", x.gm_group_count());
  if (x.gm_group_count() > 0) {
    for (size_t i = 0; i < x.m_gm_group_id.size(); ++i) {
      fmt::print(stream, "gm_group_id[ {} ]={}\n", i, x.gm_group_id(i));
    }
  }
  return stream;
}

auto GroupMapping::operator==(const GroupMapping &other) const noexcept -> bool {
  return m_gm_group_count == other.m_gm_group_count && m_gm_group_id == other.m_gm_group_id;
}

auto GroupMapping::operator!=(const GroupMapping &other) const noexcept -> bool {
  return !operator==(other);
}

auto GroupMapping::decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
    -> GroupMapping {
  auto result = GroupMapping{};
  result.gm_group_count(bitstream.readBits<uint8_t>(4));
  if (result.gm_group_count() > 0) {
    for (uint8_t i = 0; i <= vps.vps_atlas_count_minus1(); ++i) {
      result.gm_group_id(i, bitstream.getUVar<uint8_t>(result.gm_group_count()));
    }
  }
  return result;
}

void GroupMapping::encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps) const {
  bitstream.writeBits(gm_group_count(), 4);
  if (gm_group_count() > 0) {
    for (uint8_t i = 0; i <= vps.vps_atlas_count_minus1(); ++i) {
      bitstream.putUVar(gm_group_id(i), gm_group_count());
    }
  }
}

auto VpsMivExtension::vme_occupancy_scale_enabled_flag(bool value) noexcept -> VpsMivExtension & {
  PRECONDITION(!vme_embedded_occupancy_enabled_flag());
  m_vme_occupancy_scale_enabled_flag = value;
  return *this;
}

auto operator<<(std::ostream &stream, const VpsMivExtension &x) -> std::ostream & {
  fmt::print(stream, "vme_geometry_scale_enabled_flag={}\n", x.vme_geometry_scale_enabled_flag());
  fmt::print(stream, "vme_embedded_occupancy_enabled_flag={}\n",
             x.vme_embedded_occupancy_enabled_flag());
  if (!x.vme_embedded_occupancy_enabled_flag()) {
    fmt::print(stream, "vme_occupancy_scale_enabled_flag={}\n",
               x.vme_occupancy_scale_enabled_flag());
  }
  stream << x.group_mapping();
  return stream;
}

auto VpsMivExtension::decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
    -> VpsMivExtension {
  auto x = VpsMivExtension{};
  x.vme_geometry_scale_enabled_flag(bitstream.getFlag());
  x.vme_embedded_occupancy_enabled_flag(bitstream.getFlag());
  if (!x.vme_embedded_occupancy_enabled_flag()) {
    x.vme_occupancy_scale_enabled_flag(bitstream.getFlag());
  }
  x.group_mapping() = GroupMapping::decodeFrom(bitstream, vps);
  return x;
}

void VpsMivExtension::encodeTo(Common::OutputBitstream &bitstream,
                               const V3cParameterSet &vps) const {
  bitstream.putFlag(vme_geometry_scale_enabled_flag());
  bitstream.putFlag(vme_embedded_occupancy_enabled_flag());
  if (!vme_embedded_occupancy_enabled_flag()) {
    bitstream.putFlag(vme_occupancy_scale_enabled_flag());
  }
  group_mapping().encodeTo(bitstream, vps);
}

auto VpsPackedVideoExtension::vps_packed_video_present_flag(AtlasId atlasId) const -> bool {
  if (m_packing_information.contains(atlasId)) {
    return m_packing_information[atlasId].has_value();
  }
  return false;
}

[[nodiscard]] auto VpsPackedVideoExtension::packing_information(AtlasId atlasId) const
    -> const PackingInformation & {
  const auto &pin = m_packing_information[atlasId];
  VERIFY_V3CBITSTREAM(pin.has_value());
  return *pin;
}

auto VpsPackedVideoExtension::vps_packed_video_present_flag(AtlasId atlasId, bool value)
    -> VpsPackedVideoExtension & {
  auto &pin = m_packing_information[atlasId];

  if (!value) {
    pin.reset();
  } else if (!pin) {
    pin = PackingInformation{};
  }
  return *this;
}

auto VpsPackedVideoExtension::packing_information(AtlasId atlasId) -> PackingInformation & {
  auto &pin = m_packing_information[atlasId];

  if (!pin) {
    m_packing_information[atlasId] = PackingInformation{};
  }
  return *pin;
}

auto operator<<(std::ostream &stream, const VpsPackedVideoExtension &x) -> std::ostream & {
  for (const auto &[j, pi] : x.m_packing_information) {
    fmt::print(stream, "vps_packed_video_present_flag[ {} ]={}\n", j, pi.has_value());
    if (pi) {
      pi->printTo(stream, j);
    }
  }
  return stream;
}

auto VpsPackedVideoExtension::operator==(const VpsPackedVideoExtension &other) const -> bool {
  return m_packing_information == other.m_packing_information;
}

auto VpsPackedVideoExtension::operator!=(const VpsPackedVideoExtension &other) const -> bool {
  return !operator==(other);
}

auto VpsPackedVideoExtension::decodeFrom(Common::InputBitstream &stream, const V3cParameterSet &vps)
    -> VpsPackedVideoExtension {
  auto x = VpsPackedVideoExtension{};

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = vps.vps_atlas_id(k);
    auto &pin = x.m_packing_information[j];

    if (stream.getFlag()) {
      pin = PackingInformation::decodeFrom(stream);
    }
  }

  return x;
}

void VpsPackedVideoExtension::encodeTo(Common::OutputBitstream &stream,
                                       const V3cParameterSet &vps) const {
  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = vps.vps_atlas_id(k);
    const auto &pin = m_packing_information[j];

    stream.putFlag(pin.has_value());

    if (pin) {
      pin->encodeTo(stream);
    }
  }
}

auto VpsExtension::vps_miv_extension() const -> const VpsMivExtension & {
  VERIFY_V3CBITSTREAM(m_vps_miv_extension);
  return *m_vps_miv_extension;
}

auto VpsExtension::vps_packed_video_extension() const -> const VpsPackedVideoExtension & {
  VERIFY_V3CBITSTREAM(m_vps_packed_video_extension);
  return *m_vps_packed_video_extension;
}

auto VpsExtension::vps_extension_data_byte() const -> const std::vector<uint8_t> & {
  VERIFY_V3CBITSTREAM(!m_vps_miv_extension);
  VERIFY_V3CBITSTREAM(!m_vps_packed_video_extension);
  return m_vps_extension_data_byte;
}

auto VpsExtension::vps_miv_extension() -> VpsMivExtension & {
  VERIFY_V3CBITSTREAM(!m_vps_packed_video_extension);
  VERIFY_V3CBITSTREAM(m_vps_extension_data_byte.empty());

  if (!m_vps_miv_extension) {
    m_vps_miv_extension = VpsMivExtension{};
  }
  return *m_vps_miv_extension;
}

auto VpsExtension::vps_packed_video_extension() -> VpsPackedVideoExtension & {
  VERIFY_V3CBITSTREAM(!m_vps_miv_extension);
  VERIFY_V3CBITSTREAM(m_vps_extension_data_byte.empty());

  if (!m_vps_packed_video_extension) {
    m_vps_packed_video_extension = VpsPackedVideoExtension{};
  }
  return *m_vps_packed_video_extension;
}

auto VpsExtension::vps_extension_data_byte() -> std::vector<uint8_t> & {
  VERIFY_V3CBITSTREAM(!m_vps_miv_extension);
  VERIFY_V3CBITSTREAM(!m_vps_packed_video_extension);
  return m_vps_extension_data_byte;
}

auto operator<<(std::ostream &stream, const VpsExtension &x) -> std::ostream & {
  if (x.m_vps_packed_video_extension) {
    return stream << *x.m_vps_packed_video_extension;
  }
  if (x.m_vps_miv_extension) {
    return stream << *x.m_vps_miv_extension;
  }
  for (int32_t byte : x.m_vps_extension_data_byte) {
    fmt::print(stream, "vps_extension_data_byte={}\n", byte);
  }
  return stream;
}

auto VpsExtension::operator==(const VpsExtension &other) const -> bool {
  return m_vps_packed_video_extension == other.m_vps_packed_video_extension &&
         m_vps_miv_extension == other.m_vps_miv_extension &&
         m_vps_extension_data_byte == other.m_vps_extension_data_byte;
}

auto VpsExtension::operator!=(const VpsExtension &other) const -> bool {
  return !operator==(other);
}

auto VpsExtension::decodeFrom(Common::InputBitstream &stream, VpsExtensionType extensionType,
                              uint16_t extensionLength, const V3cParameterSet &vps)
    -> VpsExtension {
  auto x = VpsExtension{};

  const auto endPosition = stream.tellg() + extensionLength * std::streampos{8};

  if (extensionType == VpsExtensionType::VPS_EXT_PACKED) {
    x.m_vps_packed_video_extension = VpsPackedVideoExtension::decodeFrom(stream, vps);
  } else if (extensionType == VpsExtensionType::VPS_EXT_MIV) {
    x.m_vps_miv_extension = VpsMivExtension::decodeFrom(stream, vps);
  } else {
    x.m_vps_extension_data_byte.resize(extensionLength);

    for (uint16_t j = 0; j < extensionLength; ++j) {
      x.m_vps_extension_data_byte[j] = stream.getUint8();
    }
  }

  VERIFY_V3CBITSTREAM(stream.tellg() <= endPosition);

  while (stream.tellg() < endPosition) {
    stream.getFlag();
  }

  return x;
}

auto VpsExtension::extensionLength(const V3cParameterSet &vps) const -> uint16_t {
  if (m_vps_packed_video_extension) {
    std::ostringstream stream;
    Common::OutputBitstream bitstream{stream};
    m_vps_packed_video_extension->encodeTo(bitstream, vps);
    bitstream.zeroAlign();
    return Common::downCast<uint16_t>(std::streamoff{stream.tellp()});
  }

  if (m_vps_miv_extension) {
    std::ostringstream stream;
    Common::OutputBitstream bitstream{stream};
    m_vps_miv_extension->encodeTo(bitstream, vps);
    bitstream.zeroAlign();
    return Common::downCast<uint16_t>(std::streamoff{stream.tellp()});
  }

  return Common::downCast<uint16_t>(m_vps_extension_data_byte.size());
}

void VpsExtension::encodeTo(Common::OutputBitstream &stream, VpsExtensionType extensionType,
                            uint16_t extensionLength, const V3cParameterSet &vps) const {
  const auto endPosition = stream.tellp() + extensionLength * std::streampos{8};

  if (extensionType == VpsExtensionType::VPS_EXT_PACKED) {
    VERIFY_V3CBITSTREAM(m_vps_packed_video_extension.has_value());
    m_vps_packed_video_extension->encodeTo(stream, vps);
  } else if (extensionType == VpsExtensionType::VPS_EXT_MIV) {
    VERIFY_V3CBITSTREAM(m_vps_miv_extension.has_value());
    m_vps_miv_extension->encodeTo(stream, vps);
  } else {
    for (uint8_t byte : m_vps_extension_data_byte) {
      stream.putUint8(byte);
    }
  }

  VERIFY_V3CBITSTREAM(stream.tellp() <= endPosition);

  while (stream.tellp() < endPosition) {
    stream.putFlag(false);
  }
}

auto V3cParameterSet::profile_tier_level() const noexcept -> const ProfileTierLevel & {
  return m_profile_tier_level;
}

auto V3cParameterSet::vps_atlas_count_minus1() const noexcept -> uint8_t {
  PRECONDITION(!m_vpsAtlases.empty());
  return static_cast<uint8_t>(m_vpsAtlases.size() - 1U);
}

auto V3cParameterSet::vps_atlas_id(size_t k) const -> AtlasId {
  VERIFY_V3CBITSTREAM(k <= vps_atlas_count_minus1());
  return m_vpsAtlases[k].vps_atlas_id;
}

auto V3cParameterSet::vps_frame_width(AtlasId j) const -> int32_t {
  return atlas(j).vps_frame_width;
}

auto V3cParameterSet::vps_frame_height(AtlasId j) const -> int32_t {
  return atlas(j).vps_frame_height;
}

auto V3cParameterSet::vps_map_count_minus1(AtlasId j) const -> uint8_t {
  return atlas(j).vps_map_count_minus1;
}

auto V3cParameterSet::vps_multiple_map_streams_present_flag(AtlasId j) const -> bool {
  return atlas(j).vps_multiple_map_streams_present_flag.value_or(false);
}

auto V3cParameterSet::vps_map_absolute_coding_enabled_flag(AtlasId j, uint8_t i) const -> bool {
  return map(j, i).vps_map_absolute_coding_enabled_flag;
}

auto V3cParameterSet::vps_map_predictor_index_diff(AtlasId j, uint8_t i) const -> uint8_t {
  return map(j, i).vps_map_predictor_index_diff;
}

auto V3cParameterSet::vps_auxiliary_video_present_flag(AtlasId j) const -> bool {
  return atlas(j).vps_auxiliary_video_present_flag;
}

auto V3cParameterSet::vps_occupancy_video_present_flag(AtlasId j) const -> bool {
  return atlas(j).vps_occupancy_video_present_flag;
}

auto V3cParameterSet::vps_geometry_video_present_flag(AtlasId j) const -> bool {
  return atlas(j).vps_geometry_video_present_flag;
}

auto V3cParameterSet::vps_attribute_video_present_flag(AtlasId j) const -> bool {
  return atlas(j).vps_attribute_video_present_flag;
}

auto V3cParameterSet::occupancy_information(AtlasId j) const -> const OccupancyInformation & {
  VERIFY_V3CBITSTREAM(vps_occupancy_video_present_flag(j));
  VERIFY_V3CBITSTREAM(atlas(j).occupancy_information.has_value());
  return *atlas(j).occupancy_information;
}

auto V3cParameterSet::geometry_information(AtlasId j) const -> const GeometryInformation & {
  VERIFY_V3CBITSTREAM(vps_geometry_video_present_flag(j));
  VERIFY_V3CBITSTREAM(atlas(j).geometry_information.has_value());
  return *atlas(j).geometry_information;
}

auto V3cParameterSet::attribute_information(AtlasId j) const -> const AttributeInformation & {
  VERIFY_V3CBITSTREAM(vps_attribute_video_present_flag(j));
  VERIFY_V3CBITSTREAM(atlas(j).attribute_information.has_value());
  return *atlas(j).attribute_information;
}

auto V3cParameterSet::vps_extension_present_flag() const noexcept -> bool {
  return 0 < m_vps_extension_count;
}

auto V3cParameterSet::vps_extension_count() const noexcept -> uint8_t {
  return m_vps_extension_count;
}

auto V3cParameterSet::vps_extensions_length_minus1() const -> uint32_t {
  VERIFY_V3CBITSTREAM(0 < vps_extension_count());
  PRECONDITION(m_vps_extensions_length_minus1);
  return *m_vps_extensions_length_minus1;
}

auto V3cParameterSet::vps_extension_type(uint8_t n) const -> VpsExtensionType {
  VERIFY_V3CBITSTREAM(n < vps_extension_count());
  PRECONDITION(m_vps_extension_type.size() == vps_extension_count());
  return m_vps_extension_type[n];
}

auto V3cParameterSet::vps_extension_length(uint8_t n) const -> uint16_t {
  VERIFY_V3CBITSTREAM(n < vps_extension_count());
  PRECONDITION(m_vps_extension_length.size() == vps_extension_count());
  return m_vps_extension_length[n];
}

auto V3cParameterSet::vps_extension(VpsExtensionType vet) const -> const VpsExtension & {
  PRECONDITION(m_vps_extension_type.size() == vps_extension_count());
  PRECONDITION(m_vps_extension.size() == vps_extension_count());
  const auto i = std::find(m_vps_extension_type.cbegin(), m_vps_extension_type.cend(), vet);
  PRECONDITION(i != m_vps_extension_type.cend());
  return m_vps_extension[i - m_vps_extension_type.cbegin()];
}

auto V3cParameterSet::vpsPackingInformationPresentFlag() const -> bool {
  return Common::contains(m_vps_extension_type, VpsExtensionType::VPS_EXT_PACKED);
}

auto V3cParameterSet::vpsMivExtensionPresentFlag() const -> bool {
  return Common::contains(m_vps_extension_type, VpsExtensionType::VPS_EXT_MIV);
}

auto V3cParameterSet::vps_packed_video_present_flag(AtlasId j) const -> bool {
  return vpsPackingInformationPresentFlag() ? vps_extension(VpsExtensionType::VPS_EXT_PACKED)
                                                  .vps_packed_video_extension()
                                                  .vps_packed_video_present_flag(j)
                                            : false;
}

auto V3cParameterSet::packing_information(AtlasId j) const -> const PackingInformation & {
  VERIFY_V3CBITSTREAM(vps_packed_video_present_flag(j));
  return vps_extension(VpsExtensionType::VPS_EXT_PACKED)
      .vps_packed_video_extension()
      .packing_information(j);
}

auto V3cParameterSet::vps_miv_extension() const -> const VpsMivExtension & {
  VERIFY_V3CBITSTREAM(vpsMivExtensionPresentFlag());
  return vps_extension(VpsExtensionType::VPS_EXT_MIV).vps_miv_extension();
}

auto V3cParameterSet::profile_tier_level(ProfileTierLevel value) noexcept -> V3cParameterSet & {
  m_profile_tier_level = std::move(value);
  return *this;
}

auto V3cParameterSet::vps_atlas_count_minus1(uint8_t value) -> V3cParameterSet & {
  m_vpsAtlases.resize(value + size_t{1});
  return *this;
}

auto V3cParameterSet::vps_atlas_id(size_t k, AtlasId value) -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(k <= vps_atlas_count_minus1());
  m_vpsAtlases[k].vps_atlas_id = value;
  return *this;
}

auto V3cParameterSet::vps_frame_width(AtlasId j, int32_t value) -> V3cParameterSet & {
  atlas(j).vps_frame_width = value;
  return *this;
}

auto V3cParameterSet::vps_frame_height(AtlasId j, int32_t value) -> V3cParameterSet & {
  atlas(j).vps_frame_height = value;
  return *this;
}

auto V3cParameterSet::vps_map_count_minus1(AtlasId j, uint8_t value) -> V3cParameterSet & {
  atlas(j).vps_map_count_minus1 = value;
  atlas(j).m_vpsMaps.resize(value + size_t{1});
  return *this;
}

auto V3cParameterSet::vps_multiple_map_streams_present_flag(AtlasId j, bool value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_map_count_minus1(j) > 0);
  atlas(j).vps_multiple_map_streams_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_map_absolute_coding_enabled_flag(AtlasId j, uint8_t i, bool value)
    -> V3cParameterSet & {
  map(j, i).vps_map_absolute_coding_enabled_flag = value;
  return *this;
}

auto V3cParameterSet::vps_map_predictor_index_diff(AtlasId j, uint8_t i, uint8_t value)
    -> V3cParameterSet & {
  map(j, i).vps_map_predictor_index_diff = value;
  return *this;
}

auto V3cParameterSet::vps_auxiliary_video_present_flag(AtlasId j, bool value) -> V3cParameterSet & {
  atlas(j).vps_auxiliary_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_occupancy_video_present_flag(AtlasId j, bool value) -> V3cParameterSet & {
  atlas(j).vps_occupancy_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_geometry_video_present_flag(AtlasId j, bool value) -> V3cParameterSet & {
  atlas(j).vps_geometry_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::vps_attribute_video_present_flag(AtlasId j, bool value) -> V3cParameterSet & {
  atlas(j).vps_attribute_video_present_flag = value;
  return *this;
}

auto V3cParameterSet::occupancy_information(AtlasId j, OccupancyInformation value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_occupancy_video_present_flag(j));
  atlas(j).occupancy_information = value;
  return *this;
}

auto V3cParameterSet::geometry_information(AtlasId j, GeometryInformation value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_geometry_video_present_flag(j));
  atlas(j).geometry_information = value;
  return *this;
}

auto V3cParameterSet::attribute_information(AtlasId j, AttributeInformation value)
    -> V3cParameterSet & {
  VERIFY_V3CBITSTREAM(vps_attribute_video_present_flag(j));
  atlas(j).attribute_information = std::move(value);
  return *this;
}

auto V3cParameterSet::vps_extension(VpsExtensionType vet) -> VpsExtension & {
  PRECONDITION(m_vps_extension_count == m_vps_extension_type.size());
  PRECONDITION(m_vps_extension_count == m_vps_extension_length.size());
  PRECONDITION(m_vps_extension_count == m_vps_extension.size());

  const auto i = std::find(m_vps_extension_type.cbegin(), m_vps_extension_type.cend(), vet);

  if (i == m_vps_extension_type.cend()) {
    m_vps_extension_count = Common::downCast<uint8_t>(vps_extension_count() + 1);
    m_vps_extension_type.push_back(vet);
    m_vps_extension_length.emplace_back();
    return m_vps_extension.emplace_back();
  }

  return m_vps_extension[i - m_vps_extension_type.cbegin()];
}

auto V3cParameterSet::vps_packed_video_present_flag(AtlasId j, bool value) -> V3cParameterSet & {
  if (!vpsPackingInformationPresentFlag()) {
    auto &pve = vps_extension(VpsExtensionType::VPS_EXT_PACKED).vps_packed_video_extension();

    for (uint8_t k = 0; k <= vps_atlas_count_minus1(); ++k) {
      pve.vps_packed_video_present_flag(vps_atlas_id(k), false);
    }
  }

  vps_extension(VpsExtensionType::VPS_EXT_PACKED)
      .vps_packed_video_extension()
      .vps_packed_video_present_flag(j, value);
  return *this;
}

auto V3cParameterSet::packing_information(AtlasId j, PackingInformation value)
    -> V3cParameterSet & {
  vps_packed_video_present_flag(j, true);
  vps_extension(VpsExtensionType::VPS_EXT_PACKED)
      .vps_packed_video_extension()
      .packing_information(j) = std::move(value);
  return *this;
}

auto V3cParameterSet::vps_miv_extension(const VpsMivExtension &value) -> V3cParameterSet & {
  vps_miv_extension() = value;
  return *this;
}

void V3cParameterSet::removeVpsExtension(VpsExtensionType vet) {
  for (uint8_t n = 0; n < m_vps_extension_count; ++n) {
    if (m_vps_extension_type[n] == vet) {
      --m_vps_extension_count;
      m_vps_extension_type.erase(m_vps_extension_type.begin() + n);
      m_vps_extension_length.erase(m_vps_extension_length.begin() + n);
      m_vps_extension.erase(m_vps_extension.begin() + n);
      return;
    }
  }
}

auto V3cParameterSet::occupancy_information(AtlasId j) -> OccupancyInformation & {
  auto &oi = atlas(j).occupancy_information;
  if (!oi) {
    oi = OccupancyInformation{};
  }
  return *oi;
}

auto V3cParameterSet::geometry_information(AtlasId j) -> GeometryInformation & {
  auto &gi = atlas(j).geometry_information;
  if (!gi) {
    gi = GeometryInformation{};
  }
  return *gi;
}

auto V3cParameterSet::attribute_information(AtlasId j) -> AttributeInformation & {
  auto &ai = atlas(j).attribute_information;
  if (!ai) {
    ai = AttributeInformation{};
  }
  return *ai;
}

auto V3cParameterSet::vps_miv_extension() -> VpsMivExtension & {
  return vps_extension(VpsExtensionType::VPS_EXT_MIV).vps_miv_extension();
}

auto V3cParameterSet::indexOf(AtlasId atlasId) const -> size_t {
  for (size_t k = 0; k <= vps_atlas_count_minus1(); ++k) {
    if (vps_atlas_id(k) == atlasId) {
      return k;
    }
  }
  V3CBITSTREAM_ERROR("Invalid atlas ID");
}

auto V3cParameterSet::attrIdxOf(AtlasId atlasId, AiAttributeTypeId attrTypeId) const
    -> std::optional<uint8_t> {
  if (vps_attribute_video_present_flag(atlasId)) {
    const auto &ai = attribute_information(atlasId);

    for (uint8_t attrIdx = 0; attrIdx < ai.ai_attribute_count(); ++attrIdx) {
      if (ai.ai_attribute_type_id(attrIdx) == attrTypeId) {
        return attrIdx;
      }
    }
  } else if (vps_packed_video_present_flag(atlasId)) {
    const auto &pin = packing_information(atlasId);

    for (uint8_t attrIdx = 0; attrIdx < pin.pin_attribute_count(); ++attrIdx) {
      if (pin.pin_attribute_type_id(attrIdx) == attrTypeId) {
        return attrIdx;
      }
    }
  }
  return std::nullopt;
}

auto operator<<(std::ostream &stream, const V3cParameterSet &x) -> std::ostream & {
  stream << x.profile_tier_level();
  fmt::print(stream, "vps_v3c_parameter_set_id={}\n", x.vps_v3c_parameter_set_id());
  fmt::print(stream, "vps_atlas_count_minus1={}\n", x.vps_atlas_count_minus1());

  for (size_t k = 0; k <= x.vps_atlas_count_minus1(); ++k) {
    const auto j = x.vps_atlas_id(k);
    fmt::print(stream, "vps_atlas_id[ {} ]={}\n", k, j);
    fmt::print(stream, "vps_frame_width[ {} ]={}\n", j, x.vps_frame_width(j));
    fmt::print(stream, "vps_frame_height[ {} ]={}\n", j, x.vps_frame_height(j));
    fmt::print(stream, "vps_map_count_minus1[ {} ]={}\n", j, x.vps_map_count_minus1(j));

    if (x.vps_map_count_minus1(j) > 0) {
      fmt::print(stream, "vps_multiple_map_streams_present_flag[ {} ]={}\n", j,
                 x.vps_multiple_map_streams_present_flag(j));
    }
    fmt::print(stream, "vps_map_absolute_coding_enabled_flag[ {} ][ 0 ]={}\n", j,
               x.vps_map_absolute_coding_enabled_flag(j, 0));
    fmt::print(stream, "vps_map_predictor_index_diff[ {} ][ 0 ]={}\n", j,
               x.vps_map_predictor_index_diff(j, 0));

    for (uint8_t i = 1; i <= x.vps_map_count_minus1(j); ++i) {
      fmt::print(stream, "vps_map_absolute_coding_enabled_flag[ {} ][ {} ]={}\n", j, i,
                 x.vps_map_absolute_coding_enabled_flag(j, i));

      if (!x.vps_map_absolute_coding_enabled_flag(j, i)) {
        fmt::print(stream, "vps_map_predictor_index_diff[ {} ][ {} ]={}\n", j, i,
                   x.vps_map_predictor_index_diff(j, i));
      }
    }
    fmt::print(stream, "vps_auxiliary_video_present_flag[ {} ]={}\n", j,
               x.vps_auxiliary_video_present_flag(j));
    fmt::print(stream, "vps_occupancy_video_present_flag[ {} ]={}\n", j,
               x.vps_occupancy_video_present_flag(j));
    fmt::print(stream, "vps_geometry_video_present_flag[ {} ]={}\n", j,
               x.vps_geometry_video_present_flag(j));
    fmt::print(stream, "vps_attribute_video_present_flag[ {} ]={}\n", j,
               x.vps_attribute_video_present_flag(j));
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

  fmt::print(stream, "vps_extension_present_flag={}\n", x.vps_extension_present_flag());

  if (x.vps_extension_present_flag()) {
    fmt::print(stream, "vps_extension_count={}\n", x.vps_extension_count());
  }

  if (0 < x.vps_extension_count()) {
    fmt::print(stream, "vps_extensions_length_minus1={}\n", x.vps_extensions_length_minus1());

    for (uint8_t n = 0; n < x.vps_extension_count(); ++n) {
      fmt::print(stream, "vps_extension_type[ {} ]={}\n", n, x.vps_extension_type(n));
      fmt::print(stream, "vps_extension_length[ {} ]={}\n", n, x.vps_extension_length(n));
      stream << x.m_vps_extension[n];
    }
  }
  return stream;
}

auto V3cParameterSet::summary() const -> std::string {
  std::ostringstream stream;

  fmt::print(stream, "V3C parameter set {}:", vps_v3c_parameter_set_id());

  const auto &ptl = profile_tier_level();
  fmt::print(stream, "\n  {}, tier {}, level {}, decodes {}", ptl.profile(), ptl.ptl_tier_flag(),
             ptl.ptl_level_idc(), ptl.ptl_max_decodes_idc());

  for (size_t k = 0; k <= vps_atlas_count_minus1(); ++k) {
    const auto j = vps_atlas_id(k);
    fmt::print(stream, "\n  Atlas {}: {} x {}", j, vps_frame_width(j), vps_frame_height(j));
    if (vps_occupancy_video_present_flag(j)) {
      const auto &oi = occupancy_information(j);
      fmt::print(stream, "; [OI: codec {}, {}, 2D {}, align {}]", oi.oi_occupancy_codec_id(),
                 oi.oi_lossy_occupancy_compression_threshold(),
                 oi.oi_occupancy_2d_bit_depth_minus1() + 1, oi.oi_occupancy_MSB_align_flag());
    }

    if (vps_geometry_video_present_flag(j)) {
      const auto &gi = geometry_information(j);
      fmt::print(stream, "; [GI: codec {}, 2D {}, algin {}, 3D {}]", gi.gi_geometry_codec_id(),
                 gi.gi_geometry_2d_bit_depth_minus1() + 1, gi.gi_geometry_MSB_align_flag(),
                 gi.gi_geometry_3d_coordinates_bit_depth_minus1() + 1);
    }

    if (vps_attribute_video_present_flag(j)) {
      const auto &ai = attribute_information(j);
      fmt::print(stream, "; [AI: {}", ai.ai_attribute_count());
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        fmt::print(stream, ", {}, codec {}, dims {}, 2D {}, align {}", ai.ai_attribute_type_id(i),
                   ai.ai_attribute_codec_id(i), ai.ai_attribute_dimension_minus1(i) + 1,
                   ai.ai_attribute_2d_bit_depth_minus1(i) + 1, ai.ai_attribute_MSB_align_flag(i));
        if (i + 1 == ai.ai_attribute_count()) {
          fmt::print(stream, "]");
        }
      }
    }

    if (vps_packed_video_present_flag(j)) {
      const auto &pin = packing_information(j);
      fmt::print(stream, "; [PIN: regions {}]", pin.pin_regions_count_minus1() + 1);
    }
  }

  if (vpsMivExtensionPresentFlag()) {
    const auto &vme = vps_miv_extension();
    fmt::print(stream,
               "\n, geometry scaling {}, groups {}, embedded occupancy {}, occupancy scaling {}",
               vme.vme_geometry_scale_enabled_flag(), vme.group_mapping().gm_group_count(),
               vme.vme_embedded_occupancy_enabled_flag(), vme.vme_occupancy_scale_enabled_flag());
  }

  return stream.str();
}

auto V3cParameterSet::operator==(const V3cParameterSet &other) const -> bool {
  if (profile_tier_level() != other.profile_tier_level() ||
      vps_v3c_parameter_set_id() != other.vps_v3c_parameter_set_id() ||
      vps_atlas_count_minus1() != other.vps_atlas_count_minus1() ||
      vps_extension_present_flag() != other.vps_extension_present_flag()) {
    return false;
  }
  for (uint8_t k = 0; k <= vps_atlas_count_minus1(); ++k) {
    if (vps_atlas_id(k) != other.vps_atlas_id(k)) {
      return false;
    }
    const auto j = vps_atlas_id(k);

    if (!isAtlasEqual(other, j)) {
      return false;
    }
  }
  return areExtensionsEqual(other);
}

auto V3cParameterSet::isAtlasEqual(const V3cParameterSet &other, AtlasId j) const -> bool {
  if (vps_frame_width(j) != other.vps_frame_width(j) ||
      vps_frame_height(j) != other.vps_frame_height(j) ||
      vps_map_count_minus1(j) != other.vps_map_count_minus1(j) ||
      vps_auxiliary_video_present_flag(j) != other.vps_auxiliary_video_present_flag(j) ||
      vps_occupancy_video_present_flag(j) != other.vps_occupancy_video_present_flag(j) ||
      vps_geometry_video_present_flag(j) != other.vps_geometry_video_present_flag(j) ||
      vps_attribute_video_present_flag(j) != other.vps_attribute_video_present_flag(j)) {
    return false;
  }
  if (vps_map_count_minus1(j) > 0) {
    if (vps_multiple_map_streams_present_flag(j) !=
        other.vps_multiple_map_streams_present_flag(j)) {
      return false;
    }
  }
  for (uint8_t i = 1; i <= vps_map_count_minus1(j); ++i) {
    if (vps_map_absolute_coding_enabled_flag(j, i) !=
        other.vps_map_absolute_coding_enabled_flag(j, i)) {
      return false;
    }
    if (!vps_map_absolute_coding_enabled_flag(j, i)) {
      if (vps_map_predictor_index_diff(j, i) != other.vps_map_predictor_index_diff(j, i)) {
        return false;
      }
    }
  }
  if (vps_occupancy_video_present_flag(j)) {
    if (occupancy_information(j) != other.occupancy_information(j)) {
      return false;
    }
  }
  if (vps_geometry_video_present_flag(j)) {
    if (geometry_information(j) != other.geometry_information(j)) {
      return false;
    }
  }
  if (vps_attribute_video_present_flag(j)) {
    if (attribute_information(j) != other.attribute_information(j)) {
      return false;
    }
  }
  return true;
}

auto V3cParameterSet::areExtensionsEqual(const V3cParameterSet &other) const -> bool {
  if (vps_extension_present_flag()) {
    if (vps_extension_count() != other.vps_extension_count()) {
      return false;
    }
  }
  if (vps_extension_count() != 0) {
    if (vps_extensions_length_minus1() != other.vps_extensions_length_minus1()) {
      return false;
    }
    for (uint8_t i = 0; i < vps_extension_count(); ++i) {
      if (vps_extension_type(i) != other.vps_extension_type(i) ||
          vps_extension_length(i) != other.vps_extension_length(i) ||
          vps_extension(vps_extension_type(i)) != other.vps_extension(vps_extension_type(i))) {
        return false;
      }
    }
  }
  return true;
}

auto V3cParameterSet::operator!=(const V3cParameterSet &other) const -> bool {
  return !operator==(other);
}

auto V3cParameterSet::decodeFrom(std::istream &stream) -> V3cParameterSet {
  auto x = V3cParameterSet{};
  Common::InputBitstream bitstream{stream};

  x.profile_tier_level(ProfileTierLevel::decodeFrom(bitstream));
  x.vps_v3c_parameter_set_id(bitstream.readBits<uint8_t>(4));
  bitstream.getUint8(); // vps_reserved_zero_8bits
  x.vps_atlas_count_minus1(bitstream.readBits<uint8_t>(6));

  for (size_t k = 0; k <= x.vps_atlas_count_minus1(); ++k) {
    x.vps_atlas_id(k, AtlasId::decodeFrom(bitstream));
    const auto j = x.vps_atlas_id(k);
    x.vps_frame_width(j, bitstream.getUExpGolomb<int32_t>());
    x.vps_frame_height(j, bitstream.getUExpGolomb<int32_t>());
    x.vps_map_count_minus1(j, bitstream.readBits<uint8_t>(4));

    if (x.vps_map_count_minus1(j) > 0) {
      x.vps_multiple_map_streams_present_flag(j, bitstream.getFlag());
    }
    x.vps_map_absolute_coding_enabled_flag(j, 0, true);
    x.vps_map_predictor_index_diff(j, 0, 0);

    for (uint8_t i = 1; i <= x.vps_map_count_minus1(j); ++i) {
      if (x.vps_multiple_map_streams_present_flag(j)) {
        x.vps_map_absolute_coding_enabled_flag(j, i, bitstream.getFlag());
      } else {
        x.vps_map_absolute_coding_enabled_flag(j, i, true);
      }
      if (!x.vps_map_absolute_coding_enabled_flag(j, i)) {
        x.vps_map_predictor_index_diff(j, i, bitstream.getUExpGolomb<uint8_t>());
      }
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

  const auto vps_extension_present_flag = bitstream.getFlag();

  if (vps_extension_present_flag) {
    x.m_vps_extension_count = bitstream.getUint8();
    VERIFY_V3CBITSTREAM(0 < x.m_vps_extension_count);
  }

  if (0 < x.m_vps_extension_count) {
    x.m_vps_extensions_length_minus1 = bitstream.getUExpGolomb<uint32_t>();

    x.m_vps_extension_type.resize(x.m_vps_extension_count);
    x.m_vps_extension_length.resize(x.m_vps_extension_count);
    x.m_vps_extension.resize(x.m_vps_extension_count);

    for (uint8_t n = 0; n < x.m_vps_extension_count; ++n) {
      x.m_vps_extension_type[n] = static_cast<VpsExtensionType>(bitstream.getUint8());
      x.m_vps_extension_length[n] = bitstream.getUint16();
      x.m_vps_extension[n] = VpsExtension::decodeFrom(bitstream, x.m_vps_extension_type[n],
                                                      x.m_vps_extension_length[n], x);
    }
  }
  bitstream.byteAlignment();
  return x;
}

void V3cParameterSet::calculateExtensionLengths() {
  m_vps_extensions_length_minus1.reset();

  if (0 < m_vps_extension_count) {
    m_vps_extensions_length_minus1 = 0;

    for (uint8_t n = 0; n < vps_extension_count(); ++n) {
      m_vps_extension_length[n] = m_vps_extension[n].extensionLength(*this);
      *m_vps_extensions_length_minus1 += 3U + m_vps_extension_length[n];
    }

    --*m_vps_extensions_length_minus1;
  }
}

void V3cParameterSet::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};

  profile_tier_level().encodeTo(bitstream);
  bitstream.writeBits(vps_v3c_parameter_set_id(), 4);
  bitstream.putUint8(0); // vps_reserved_zero_8bits
  bitstream.writeBits(vps_atlas_count_minus1(), 6);

  for (size_t k = 0; k <= vps_atlas_count_minus1(); ++k) {
    const auto j = vps_atlas_id(k);
    j.encodeTo(bitstream);
    bitstream.putUExpGolomb(vps_frame_width(j));
    bitstream.putUExpGolomb(vps_frame_height(j));
    bitstream.writeBits(vps_map_count_minus1(j), 4);

    if (vps_map_count_minus1(j) > 0) {
      bitstream.putFlag(vps_multiple_map_streams_present_flag(j));
    }
    for (uint8_t i = 1; i <= vps_map_count_minus1(j); ++i) {
      if (vps_multiple_map_streams_present_flag(j)) {
        bitstream.putFlag(vps_map_absolute_coding_enabled_flag(j, i));
      }
      if (!vps_map_absolute_coding_enabled_flag(j, i)) {
        bitstream.putUExpGolomb(vps_map_predictor_index_diff(j, i));
      }
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
    bitstream.putUint8(vps_extension_count());
  }
  if (0 < vps_extension_count()) {
    VERIFY_V3CBITSTREAM(vps_extensions_length_minus1() + size_t{1} ==
                        std::accumulate(m_vps_extension_length.cbegin(),
                                        m_vps_extension_length.cend(),
                                        3 * m_vps_extension_length.size()));
    bitstream.putUExpGolomb(vps_extensions_length_minus1());

    for (uint8_t n = 0; n < vps_extension_count(); ++n) {
      bitstream.putUint8(static_cast<uint8_t>(vps_extension_type(n)));
      bitstream.putUint16(vps_extension_length(n));
      m_vps_extension[n].encodeTo(bitstream, vps_extension_type(n), vps_extension_length(n), *this);
    }
  }
  bitstream.byteAlignment();
}

auto V3cParameterSet::atlas(AtlasId atlasId) const -> const VpsAtlas & {
  return m_vpsAtlases[indexOf(atlasId)];
}

auto V3cParameterSet::atlas(AtlasId atlasId) -> VpsAtlas & {
  return m_vpsAtlases[indexOf(atlasId)];
}

auto V3cParameterSet::map(AtlasId atlasId, uint8_t mapIdx) const -> const VpsMap & {
  const auto &atlas_ = atlas(atlasId);
  VERIFY_V3CBITSTREAM(mapIdx < atlas_.m_vpsMaps.size());
  return atlas_.m_vpsMaps[mapIdx];
}

auto V3cParameterSet::map(AtlasId atlasId, uint8_t mapIdx) -> VpsMap & {
  auto &atlas_ = atlas(atlasId);
  VERIFY_V3CBITSTREAM(mapIdx < atlas_.m_vpsMaps.size());
  return atlas_.m_vpsMaps[mapIdx];
}
} // namespace TMIV::MivBitstream
