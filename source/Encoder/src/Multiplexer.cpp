/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/verify.h>
#include <TMIV/Encoder/AnnexB.h>
#include <TMIV/Encoder/Multiplexer.h>

namespace TMIV::Encoder {
namespace {
void setRegionInformation(MivBitstream::PackingInformation &packingInformation, int regionIdx,
                          const Common::Json &region) {
  packingInformation
      .pin_region_tile_id(regionIdx, region.require("pin_region_tile_id").as<uint8_t>())
      .pin_region_type_id_minus2(regionIdx,
                                 region.require("pin_region_type_id_minus2").as<uint8_t>())
      .pin_region_top_left_x(regionIdx, region.require("pin_region_top_left_x").as<uint16_t>())
      .pin_region_top_left_y(regionIdx, region.require("pin_region_top_left_y").as<uint16_t>())
      .pin_region_width_minus1(regionIdx, region.require("pin_region_width_minus1").as<uint16_t>())
      .pin_region_height_minus1(regionIdx,
                                region.require("pin_region_height_minus1").as<uint16_t>())
      .pin_region_unpack_top_left_x(regionIdx,
                                    region.require("pin_region_unpack_top_left_x").as<uint16_t>())
      .pin_region_unpack_top_left_y(regionIdx,
                                    region.require("pin_region_unpack_top_left_y").as<uint16_t>())
      .pin_region_rotation_flag(regionIdx, region.require("pin_region_rotation_flag").as<bool>());
  if (packingInformation.pin_region_type_id_minus2(regionIdx) + 2 ==
          MivBitstream::VuhUnitType::V3C_AVD ||
      packingInformation.pin_region_type_id_minus2(regionIdx) + 2 ==
          MivBitstream::VuhUnitType::V3C_GVD) {
    packingInformation
        .pin_region_map_index(regionIdx, region.require("pin_region_map_index").as<uint8_t>())
        .pin_region_auxiliary_data_flag(
            regionIdx, region.require("pin_region_auxiliary_data_flag").as<bool>());
  }

  if (packingInformation.pin_region_type_id_minus2(regionIdx) + 2 ==
      MivBitstream::VuhUnitType::V3C_AVD) {
    auto k = region.require("pin_region_attr_index").as<uint8_t>();
    packingInformation.pin_region_attr_index(regionIdx, k);
    if (packingInformation.pin_attribute_dimension_minus1(k) > 0U) {
      packingInformation.pin_region_attr_partition_index(
          regionIdx, region.require("pin_region_attr_partition_index").as<uint8_t>());
    }
  }
}
} // namespace

Multiplexer::Multiplexer(Common::Json packingInformationNode)
    : m_packingInformationNode{std::move(packingInformationNode)} {
  if (packingInformationNode) {
    const auto &atlases_packing_info = m_packingInformationNode.as<Common::Json::Array>();
    PRECONDITION(!atlases_packing_info.empty());
    for (const auto &atlas_packing_info : atlases_packing_info) {
      PRECONDITION(!atlas_packing_info.require("pin_regions").as<Common::Json::Array>().empty());
      PRECONDITION(atlas_packing_info.require("pin_regions").as<Common::Json::Array>().size() <
                   256U);
    }
  }
}

void Multiplexer::setAttributeVideoBitstreamServer(AttributeVideoBitstreamServer server) {
  m_openAttributeVideoBitstream = std::move(server);
}

void Multiplexer::setGeometryVideoBitstreamServer(GeometryVideoBitstreamServer server) {
  m_openGeometryVideoBitstream = std::move(server);
}

void Multiplexer::setOccupancyVideoBitstreamServer(OccupancyVideoBitstreamServer server) {
  m_openOccupancyVideoBitstream = std::move(server);
}

void Multiplexer::setPackedVideoBitstreamServer(PackedVideoBitstreamServer server) {
  m_openPackedVideoBitstream = std::move(server);
}

void Multiplexer::readInputBitstream(std::istream &stream) {
  // Decode SSVH
  const auto ssvh = MivBitstream::SampleStreamV3cHeader::decodeFrom(stream);

  // Decode first V3C unit, which has to contain the VPS
  const auto ssvu0 = MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);

  // Decode the VPS
  std::istringstream substream{ssvu0.ssvu_v3c_unit()};
  const auto vuh = MivBitstream::V3cUnitHeader::decodeFrom(substream);
  if (vuh.vuh_unit_type() != MivBitstream::VuhUnitType::V3C_VPS) {
    throw std::runtime_error("the first V3C unit has to be the VPS");
  }
  m_vps = MivBitstream::V3cParameterSet::decodeFrom(substream);

  // Append the remaining V3C units
  while (!stream.eof()) {
    const auto ssvu = MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);
    m_units.push_back(ssvu.ssvu_v3c_unit());
    stream.peek();
  }
}

void Multiplexer::appendVideoSubBitstreams() {
  for (size_t k = 0; k <= m_vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_vps.vps_atlas_id(k);
    checkRestrictions(j);

    if (m_vps.vps_packing_information_present_flag()) {
      if (m_vps.vps_packed_video_present_flag(j)) {
        appendPvd(j);
        continue;
      }
    }

    if (m_vps.vps_geometry_video_present_flag(j)) {
      appendGvd(j);
    }
    if (m_vps.vps_occupancy_video_present_flag(j)) {
      appendOvd(j);
    }
    if (m_vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = m_vps.attribute_information(j);
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        const auto type = ai.ai_attribute_type_id(i);
        appendAvd(j, i, type);
      }
    }
  }
}

void Multiplexer::writeOutputBitstream(std::ostream &stream) const {
  std::ostringstream outVPSStream;
  const auto vuh = TMIV::MivBitstream::V3cUnitHeader{TMIV::MivBitstream::VuhUnitType::V3C_VPS};
  const auto vpsUnit = TMIV::MivBitstream::V3cUnit{vuh, m_vps};
  vpsUnit.encodeTo(outVPSStream);
  const auto vpsUnitSize = outVPSStream.str().size();

  // Find size of largest unit
  auto maxSize =
      max_element(std::cbegin(m_units), std::cend(m_units),
                  [](const std::string &a, const std::string &b) { return a.size() < b.size(); })
          ->size();

  maxSize = std::max(maxSize, vpsUnitSize);

  // Calculate how many bytes are needed to store that size
  auto precisionBytesMinus1 = uint8_t{};
  while (maxSize >= uint64_t{1} << 8 * (precisionBytesMinus1 + 1)) {
    ++precisionBytesMinus1;
  }

  // Write the sample stream header
  const auto ssvh = MivBitstream::SampleStreamV3cHeader{precisionBytesMinus1};
  ssvh.encodeTo(stream);

  const auto ssvu0 = MivBitstream::SampleStreamV3cUnit{outVPSStream.str()};
  ssvu0.encodeTo(stream, ssvh);

  // Write the units
  for (const auto &unit : m_units) {
    const auto ssvu = MivBitstream::SampleStreamV3cUnit{unit};
    ssvu.encodeTo(stream, ssvh);
  }
}

void Multiplexer::checkRestrictions(MivBitstream::AtlasId atlasId) const {
  if (m_vps.vps_map_count_minus1(atlasId) > 0) {
    throw std::runtime_error("Having multiple maps is not supported.");
  }
  if (m_vps.vps_auxiliary_video_present_flag(atlasId)) {
    throw std::runtime_error("Auxiliary video is not supported.");
  }
}

void Multiplexer::appendGvd(MivBitstream::AtlasId atlasId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_GVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  appendVideoSubBitstream(vuh, m_openGeometryVideoBitstream(atlasId));
}

void Multiplexer::appendOvd(MivBitstream::AtlasId atlasId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_OVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  appendVideoSubBitstream(vuh, m_openOccupancyVideoBitstream(atlasId));
}

void Multiplexer::appendAvd(MivBitstream::AtlasId atlasId, uint8_t attributeIdx,
                            MivBitstream::AiAttributeTypeId typeId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  vuh.vuh_attribute_index(attributeIdx);

  appendVideoSubBitstream(vuh, m_openAttributeVideoBitstream(typeId, atlasId, attributeIdx));
}

void Multiplexer::appendPvd(MivBitstream::AtlasId atlasId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_PVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  appendVideoSubBitstream(vuh, m_openPackedVideoBitstream(atlasId));
}

void Multiplexer::addPackingInformation() {
  PRECONDITION(m_packingInformationNode);

  m_vps.vps_packing_information_present_flag(true);

  for (const auto &info : m_packingInformationNode.as<Common::Json::Array>()) {
    MivBitstream::PackingInformation packingInformation;
    const auto atlasId = MivBitstream::AtlasId{info.require("pin_atlas_id").as<uint8_t>()};

    packingInformation.pin_codec_id(info.require("pin_codec_id").as<uint8_t>());

    if (info.require("pin_occupancy_present_flag").as<bool>()) {
      updateOccupancyInformation(packingInformation, atlasId);
    }

    if (info.require("pin_geometry_present_flag").as<bool>()) {
      updateGeometryInformation(packingInformation, atlasId);
    }

    if (info.require("pin_attributes_present_flag").as<bool>()) {
      updateAttributeInformation(packingInformation, atlasId);
    }

    packingInformation.pin_regions_count_minus1(
        static_cast<uint8_t>(info.require("pin_regions").as<Common::Json::Array>().size() - 1U));

    auto regionIdx = 0;
    for (const auto &region : info.require("pin_regions").as<Common::Json::Array>()) {
      setRegionInformation(packingInformation, regionIdx, region);
      ++regionIdx;
    }

    m_vps.vps_packed_video_present_flag(atlasId, true);
    m_vps.packing_information(atlasId, packingInformation);
  }
}

void Multiplexer::updateOccupancyInformation(MivBitstream::PackingInformation &packingInformation,
                                             const MivBitstream::AtlasId &atlasId) {
  m_vps.vps_occupancy_video_present_flag(atlasId, false);
  packingInformation.pin_occupancy_present_flag(true)
      .pin_occupancy_2d_bit_depth_minus1(
          m_vps.occupancy_information(atlasId).oi_occupancy_2d_bit_depth_minus1())
      .pin_occupancy_MSB_align_flag(
          m_vps.occupancy_information(atlasId).oi_occupancy_MSB_align_flag())
      .pin_lossy_occupancy_compression_threshold(
          m_vps.occupancy_information(atlasId).oi_lossy_occupancy_compression_threshold());
}

void Multiplexer::updateGeometryInformation(MivBitstream::PackingInformation &packingInformation,
                                            const MivBitstream::AtlasId &atlasId) {
  m_vps.vps_geometry_video_present_flag(atlasId, false);
  packingInformation.pin_geometry_present_flag(true)
      .pin_geometry_2d_bit_depth_minus1(
          m_vps.geometry_information(atlasId).gi_geometry_2d_bit_depth_minus1())
      .pin_geometry_MSB_align_flag(m_vps.geometry_information(atlasId).gi_geometry_MSB_align_flag())
      .pin_geometry_3d_coordinates_bit_depth_minus1(
          m_vps.geometry_information(atlasId).gi_geometry_3d_coordinates_bit_depth_minus1());
}

void Multiplexer::updateAttributeInformation(MivBitstream::PackingInformation &packingInformation,
                                             const MivBitstream::AtlasId &atlasId) {
  m_vps.vps_attribute_video_present_flag(atlasId, false);
  packingInformation.pin_attribute_present_flag(true).pin_attribute_count(
      m_vps.attribute_information(atlasId).ai_attribute_count());
  for (uint8_t i = 0U; i < packingInformation.pin_attribute_count(); i++) {
    packingInformation
        .pin_attribute_type_id(i, m_vps.attribute_information(atlasId).ai_attribute_type_id(i))
        .pin_attribute_2d_bit_depth_minus1(
            i, m_vps.attribute_information(atlasId).ai_attribute_2d_bit_depth_minus1(i))
        .pin_attribute_MSB_align_flag(
            i, m_vps.attribute_information(atlasId).ai_attribute_MSB_align_flag(i));

    LIMITATION(m_vps.vps_map_count_minus1(atlasId) == 0);
    packingInformation.pin_attribute_map_absolute_coding_persistence_flag(i, true);

    packingInformation
        .pin_attribute_dimension_minus1(
            i, m_vps.attribute_information(atlasId).ai_attribute_dimension_minus1(i))
        .pin_attribute_dimension_partitions_minus1(i, 0);
  }
}

void Multiplexer::appendVideoSubBitstream(const MivBitstream::V3cUnitHeader &vuh,
                                          std::unique_ptr<std::istream> stream) {
  std::ostringstream substream;
  vuh.encodeTo(substream);

  std::vector<char> buffer;
  readNalUnitFromAnnexBStreamIntoBuffer(*stream, buffer);
  VERIFY_BITSTREAM(!buffer.empty());
  do {
    fmt::print("[{}] NAL unit: {} bytes\n", vuh.summary(), buffer.size());
    Common::putUint32(substream, Common::downCast<uint32_t>(buffer.size()));
    substream.write(buffer.data(), Common::downCast<std::streamsize>(buffer.size()));
    readNalUnitFromAnnexBStreamIntoBuffer(*stream, buffer);
  } while (!buffer.empty());

  m_units.push_back(substream.str());
}
} // namespace TMIV::Encoder
