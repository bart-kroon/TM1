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

#include <TMIV/Encoder/Multiplexer.h>

namespace TMIV::Encoder {
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
  std::cout << m_vps;

  // Append the first V3C unit
  m_units.push_back(ssvu0.ssvu_v3c_unit());

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
  // Find size of largest unit
  const auto maxSize =
      max_element(std::cbegin(m_units), std::cend(m_units),
                  [](const std::string &a, const std::string &b) { return a.size() < b.size(); })
          ->size();

  // Calculate how many bytes are needed to store that size
  auto precisionBytesMinus1 = uint8_t{};
  while (maxSize >= uint64_t{1} << 8 * (precisionBytesMinus1 + 1)) {
    ++precisionBytesMinus1;
  }

  // Write the sample stream header
  const auto ssvh = MivBitstream::SampleStreamV3cHeader{precisionBytesMinus1};
  ssvh.encodeTo(stream);
  std::cout << '\n' << ssvh;

  // Write the units
  for (const auto &unit : m_units) {
    const auto ssvu = MivBitstream::SampleStreamV3cUnit{unit};
    ssvu.encodeTo(stream, ssvh);
    std::cout << '\n' << ssvu;

    // Print the V3C unit header (for fun, why not)
    std::istringstream stream2{unit};
    const auto vuh = MivBitstream::V3cUnitHeader::decodeFrom(stream2);
    std::cout << vuh;
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
  appendSubBitstream(vuh, m_openGeometryVideoBitstream(atlasId));
}

void Multiplexer::appendOvd(MivBitstream::AtlasId atlasId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_OVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  appendSubBitstream(vuh, m_openOccupancyVideoBitstream(atlasId));
}

void Multiplexer::appendAvd(MivBitstream::AtlasId atlasId, uint8_t attributeIdx,
                            MivBitstream::AiAttributeTypeId typeId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  vuh.vuh_attribute_index(attributeIdx);

  appendSubBitstream(vuh, m_openAttributeVideoBitstream(typeId, atlasId, attributeIdx));
}

void Multiplexer::appendPvd(MivBitstream::AtlasId atlasId) {
  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_PVD};
  vuh.vuh_v3c_parameter_set_id(m_vps.vps_v3c_parameter_set_id());
  vuh.vuh_atlas_id(atlasId);
  appendSubBitstream(vuh, m_openPackedVideoBitstream(atlasId));
}

void Multiplexer::addPackingInformation() {
  m_vps.vps_packing_information_present_flag(true);

  for (size_t k = 0; k <= m_vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_vps.vps_atlas_id(k);

    // TODO (LK) Get the packing information from JSON
    if (m_vps.vps_packed_video_present_flag(j)) {
      m_vps.vps_occupancy_video_present_flag(j, false);
      m_vps.vps_geometry_video_present_flag(j, false);
      m_vps.vps_attribute_video_present_flag(j, false);
    }
  }
}

void Multiplexer::appendSubBitstream(const MivBitstream::V3cUnitHeader &vuh,
                                     std::unique_ptr<std::istream> stream) {
  std::ostringstream substream;
  vuh.encodeTo(substream);
  substream << stream->rdbuf();

  m_units.push_back(substream.str());
}

} // namespace TMIV::Encoder
