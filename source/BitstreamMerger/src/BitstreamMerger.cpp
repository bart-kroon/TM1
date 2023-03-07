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

#include <TMIV/BitstreamMerger/BitstreamMerger.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <ios>
#include <sstream>
#include <utility>
#include <vector>

using namespace std::string_literals;

namespace TMIV::BitstreamMerger {
using VU = MivBitstream::V3cUnit;
using VUT = MivBitstream::VuhUnitType;
using VUH = MivBitstream::V3cUnitHeader;
using NU = MivBitstream::NalUnit;
using NUT = MivBitstream::NalUnitType;
using Json = Common::Json;

void BitstreamMerger::readInputBitstream(std::istream &stream) {
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

void BitstreamMerger::readOutofBandMetadata(const Common::Json &node) {
  m_bitDepth = node.require("bit_depth").as<uint32_t>();
  auto irapFrameIndices = node.require("irap_frame_indices").asVector<int32_t>();
  for (auto ifi : irapFrameIndices) {
    m_irapFrameIndices.emplace_back(ifi);
  }
}

void BitstreamMerger::updatePackingInformation() {
  for (size_t atlasIdx = 0; atlasIdx <= m_vps.vps_atlas_count_minus1(); atlasIdx++) {
    const auto atlasId = m_vps.vps_atlas_id(atlasIdx);
    const auto &pin = m_vps.packing_information(atlasId);
    MivBitstream::PackingInformation packingInformation;

    packingInformation.pin_codec_id(pin.pin_codec_id());
    m_vps.vps_occupancy_video_present_flag(atlasId, false);
    m_vps.vps_geometry_video_present_flag(atlasId, false);
    updateOccupancyInformation(packingInformation, pin);
    updateGeometryInformation(packingInformation, pin);
    updateAttributeInformation(packingInformation, pin, m_vps, atlasId);

    if (m_vps.vpsPackingInformationPresentFlag()) {
      auto pin_attr_count = packingInformation.pin_attribute_count();
      packingInformation.pin_regions_count_minus1(pin.pin_regions_count_minus1() + pin_attr_count);
      setAttributePinRegion(packingInformation, m_vps, atlasId);
      m_vps.vps_attribute_video_present_flag(atlasId, false);
      for (size_t i = 0; i <= pin.pin_regions_count_minus1(); i++) {
        packingInformation.pin_region_tile_id(i + pin_attr_count, pin.pin_region_tile_id(i));
        packingInformation.pin_region_type_id_minus2(i + pin_attr_count,
                                                     pin.pin_region_type_id_minus2(i));
        packingInformation.pin_region_top_left_x(i + pin_attr_count, pin.pin_region_top_left_x(i));
        packingInformation.pin_region_top_left_y(i + pin_attr_count, pin.pin_region_top_left_y(i));
        packingInformation.pin_region_width_minus1(i + pin_attr_count,
                                                   pin.pin_region_width_minus1(i));
        packingInformation.pin_region_height_minus1(i + pin_attr_count,
                                                    pin.pin_region_height_minus1(i));
        packingInformation.pin_region_unpack_top_left_x(i + pin_attr_count,
                                                        pin.pin_region_unpack_top_left_x(i));
        packingInformation.pin_region_unpack_top_left_y(i + pin_attr_count,
                                                        pin.pin_region_unpack_top_left_y(i));
        packingInformation.pin_region_rotation_flag(i + pin_attr_count,
                                                    pin.pin_region_rotation_flag(i));

        if (pin.pinRegionTypeId(i) == MivBitstream::VuhUnitType::V3C_GVD) {
          packingInformation.pin_region_map_index(i + pin_attr_count, pin.pin_region_map_index(i));
          packingInformation.pin_region_auxiliary_data_flag(i + pin_attr_count,
                                                            pin.pin_region_auxiliary_data_flag(i));
          packingInformation.pin_region_top_left_x(
              i + pin_attr_count,
              static_cast<uint16_t>(m_vps.vps_frame_width(atlasId)) + pin.pin_region_top_left_x(i));
        }
      }
      m_vps.vps_packed_video_present_flag(atlasId, true);
      m_vps.packing_information(atlasId, packingInformation);
      m_vps.calculateExtensionLengths();
    }
  }
}

void BitstreamMerger::writeOutputBitstream(std::ostream &stream) {
  std::ostringstream outVPSStream;
  const auto vuh = TMIV::MivBitstream::V3cUnitHeader{TMIV::MivBitstream::V3cUnitHeader::vps()};
  const auto vpsUnit = TMIV::MivBitstream::V3cUnit{vuh, m_vps};
  vpsUnit.encodeTo(outVPSStream);
  const auto vpsUnitSize = outVPSStream.str().size();

  // Find size of largest unit
  auto maxSize = std::max_element(
                     std::cbegin(m_units), std::cend(m_units),
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

void BitstreamMerger::writeOutOfBandMetadata(std::ostream &stream) {
  auto metadata = Common::Json::Array{};

  for (size_t atlasIdx = 0; atlasIdx <= m_vps.vps_atlas_count_minus1(); atlasIdx++) {
    const auto atlasId = m_vps.vps_atlas_id(atlasIdx);
    VUH vuh = VUH::pvd(m_vpsId, atlasId);
    auto obj = Json::Object{};

    obj["vuh_unit_type"] = Json{vuh.vuh_unit_type()};
    obj["vuh_v3c_parameter_set_id"] = Json{vuh.vuh_v3c_parameter_set_id()};
    obj["vuh_atlas_id"] = Json{vuh.vuh_atlas_id()};

    calculatePackedVIdeoComponentResolution(atlasId);
    obj["frame_size"s] =
        Json{Json::Array{Json{m_packedVideoResolution[0]}, Json{m_packedVideoResolution[1]}}};
    obj["bit_depth"s] = Json{m_bitDepth};
    obj["irap_frame_indices"s] = Json{m_irapFrameIndices};

    metadata.emplace_back(obj);
  }

  Common::Json{std::move(metadata)}.saveTo(stream);
  stream << '\n';
}

// Note(JB): TODO: getting frame rate to report bitrate needs to be implemented
void BitstreamMerger::reportSummary(std::streampos bytesWritten) {
  Common::logInfo("Total size is {} B ({} Kb)", bytesWritten,
                  8e-3 * static_cast<double>(bytesWritten));
  // Common::logInfo("Total bitrate is {} kbps\n", 8e-3 * static_cast<double>(bytesWritten) *
  //                                              m_frameRate / m_numberOfFrames);
}

void BitstreamMerger::updateOccupancyInformation(
    MivBitstream::PackingInformation &packinginformation,
    const MivBitstream::PackingInformation &pin) {
  if (pin.pin_occupancy_present_flag()) {
    packinginformation.pin_occupancy_present_flag(true)
        .pin_occupancy_2d_bit_depth_minus1(pin.pin_occupancy_2d_bit_depth_minus1())
        .pin_occupancy_MSB_align_flag(pin.pin_occupancy_MSB_align_flag())
        .pin_lossy_occupancy_compression_threshold(pin.pin_lossy_occupancy_compression_threshold());
  }
}

void BitstreamMerger::updateGeometryInformation(
    MivBitstream::PackingInformation &packinginformation,
    const MivBitstream::PackingInformation &pin) {
  if (pin.pin_geometry_present_flag()) {
    packinginformation.pin_geometry_present_flag(true)
        .pin_geometry_2d_bit_depth_minus1(pin.pin_geometry_2d_bit_depth_minus1())
        .pin_geometry_MSB_align_flag(pin.pin_geometry_MSB_align_flag())
        .pin_geometry_3d_coordinates_bit_depth_minus1(
            pin.pin_geometry_3d_coordinates_bit_depth_minus1());
  }
}

void BitstreamMerger::updateAttributeInformation(
    MivBitstream::PackingInformation &packinginformation,
    const MivBitstream::PackingInformation &pin, const MivBitstream::V3cParameterSet &vps,
    MivBitstream::AtlasId j) {
  if (pin.pin_attribute_present_flag()) {
    packinginformation.pin_attribute_present_flag(true).pin_attribute_count(
        pin.pin_attribute_count());
    for (uint8_t i = 0; i < packinginformation.pin_attribute_count(); i++) {
      packinginformation.pin_attribute_type_id(i, pin.pin_attribute_type_id(i))
          .pin_attribute_2d_bit_depth_minus1(i, pin.pin_attribute_2d_bit_depth_minus1(i))
          .pin_attribute_MSB_align_flag(i, pin.pin_attribute_MSB_align_flag(i))
          .pin_attribute_map_absolute_coding_persistence_flag(i, false);

      packinginformation.pin_attribute_dimension_minus1(i, pin.pin_attribute_dimension_minus1(i))
          .pin_attribute_dimension_partitions_minus1(i, 0);
    }
  } else {
    if (vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = vps.attribute_information(j);
      packinginformation.pin_attribute_present_flag(true).pin_attribute_count(
          ai.ai_attribute_count());
      for (uint8_t i = 0; i < packinginformation.pin_attribute_count(); i++) {
        packinginformation.pin_attribute_type_id(i, ai.ai_attribute_type_id(i))
            .pin_attribute_2d_bit_depth_minus1(i, ai.ai_attribute_2d_bit_depth_minus1(i))
            .pin_attribute_MSB_align_flag(i, ai.ai_attribute_MSB_align_flag(i))
            .pin_attribute_map_absolute_coding_persistence_flag(i, false)
            .pin_attribute_dimension_minus1(i, ai.ai_attribute_dimension_minus1(i))
            .pin_attribute_dimension_partitions_minus1(i, 0);
      }
    }
  }
}

void BitstreamMerger::setAttributePinRegion(MivBitstream::PackingInformation &packingInformation,
                                            const MivBitstream::V3cParameterSet &vps,
                                            MivBitstream::AtlasId j) {
  if (vps.vps_attribute_video_present_flag(j)) {
    const auto &ai = vps.attribute_information(j);
    for (uint8_t i = 0; i < ai.ai_attribute_count(); i++) {
      packingInformation.pin_region_tile_id(i, 0)
          .pin_region_type_id_minus2(i, static_cast<uint16_t>(2))
          .pin_region_top_left_x(i, static_cast<uint16_t>(vps.vps_frame_width(j) * i))
          .pin_region_top_left_y(i, 0)
          .pin_region_width_minus1(i, static_cast<uint16_t>(vps.vps_frame_width(j) - 1))
          .pin_region_height_minus1(i, static_cast<uint16_t>(vps.vps_frame_height(j) - 1))
          .pin_region_unpack_top_left_x(i, 0)
          .pin_region_unpack_top_left_y(i, 0)
          .pin_region_rotation_flag(i, false)
          .pin_region_map_index(i, 0)
          .pin_region_auxiliary_data_flag(i, false)
          .pin_region_attr_index(i, 0);
    }
  }
}

void BitstreamMerger::calculatePackedVIdeoComponentResolution(MivBitstream::AtlasId atlasId) {
  const auto &pin = m_vps.packing_information(atlasId);

  for (uint8_t regionIdx = 0; regionIdx <= pin.pin_regions_count_minus1(); ++regionIdx) {
    const auto regionPackedOffsetX = pin.pin_region_top_left_x(regionIdx);
    const auto regionPackedOffsetY = pin.pin_region_top_left_y(regionIdx);
    auto regionWidth = pin.pin_region_width_minus1(regionIdx) + 1;
    auto regionHeight = pin.pin_region_height_minus1(regionIdx) + 1;

    if (pin.pin_region_rotation_flag(regionIdx)) {
      std::swap(regionWidth, regionHeight);
    }

    m_packedVideoResolution[0] =
        std::max(m_packedVideoResolution[0], regionPackedOffsetX + regionWidth);
    m_packedVideoResolution[1] =
        std::max(m_packedVideoResolution[1], regionPackedOffsetY + regionHeight);
  }
}
} // namespace TMIV::BitstreamMerger
