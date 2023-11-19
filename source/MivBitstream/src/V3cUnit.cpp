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

#include <TMIV/MivBitstream/V3cUnit.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

using TMIV::Common::overload;

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const V3cUnitHeader &x) -> std::ostream & {
  stream << "vuh_unit_type=" << x.vuh_unit_type();
  if (x.vuh_unit_type() == VuhUnitType::V3C_AVD || x.vuh_unit_type() == VuhUnitType::V3C_GVD ||
      x.vuh_unit_type() == VuhUnitType::V3C_OVD || x.vuh_unit_type() == VuhUnitType::V3C_AD ||
      x.vuh_unit_type() == VuhUnitType::V3C_CAD || x.vuh_unit_type() == VuhUnitType::V3C_PVD) {
    stream << "\nvuh_v3c_parameter_set_id=" << int32_t{x.vuh_v3c_parameter_set_id()};
  }
  if (x.vuh_unit_type() == VuhUnitType::V3C_AVD || x.vuh_unit_type() == VuhUnitType::V3C_GVD ||
      x.vuh_unit_type() == VuhUnitType::V3C_OVD || x.vuh_unit_type() == VuhUnitType::V3C_AD ||
      x.vuh_unit_type() == VuhUnitType::V3C_PVD) {
    stream << "\nvuh_atlas_id=" << x.vuh_atlas_id();
  }
  if (x.vuh_unit_type() == VuhUnitType::V3C_AVD) {
    stream << "\nvuh_attribute_index=" << int32_t{x.vuh_attribute_index()}
           << "\nvuh_attribute_partition_index=" << int32_t{x.vuh_attribute_partition_index()}
           << "\nvuh_map_index=" << int32_t{x.vuh_map_index()}
           << "\nvuh_auxiliary_video_flag=" << std::boolalpha << x.vuh_auxiliary_video_flag();
  } else if (x.vuh_unit_type() == VuhUnitType::V3C_GVD) {
    stream << "\nvuh_map_index=" << int32_t{x.vuh_map_index()}
           << "\nvuh_auxiliary_video_flag=" << std::boolalpha << x.vuh_auxiliary_video_flag();
  }
  return stream << '\n';
}

auto V3cUnitHeader::summary() const -> std::string {
  std::ostringstream stream;

  stream << vuh_unit_type();

  if (vuh_unit_type() != VuhUnitType::V3C_VPS) {
    TMIV_FMT::print(stream, " vps:{}", vuh_v3c_parameter_set_id());
  }
  if (vuh_unit_type() != VuhUnitType::V3C_VPS && vuh_unit_type() != VuhUnitType::V3C_CAD) {
    TMIV_FMT::print(stream, " atlas:{}", vuh_atlas_id());
  }
  if (vuh_unit_type() == VuhUnitType::V3C_AVD) {
    TMIV_FMT::print(stream, " attr:{} part:{} map:{} aux:{}", vuh_attribute_index(),
                    vuh_attribute_partition_index(), vuh_map_index(), vuh_auxiliary_video_flag());
  } else if (vuh_unit_type() == VuhUnitType::V3C_GVD) {
    TMIV_FMT::print(stream, " map:{} aux:{}", vuh_map_index(), vuh_auxiliary_video_flag());
  }
  return stream.str();
}

auto V3cUnitHeader::decodeFrom(std::istream &stream) -> V3cUnitHeader {
  Common::InputBitstream bitstream{stream};
  auto x = V3cUnitHeader{};
  x.m_vuh_unit_type = bitstream.readBits<VuhUnitType>(5);

  if (x.vuh_unit_type() == VuhUnitType::V3C_AVD || x.vuh_unit_type() == VuhUnitType::V3C_GVD ||
      x.vuh_unit_type() == VuhUnitType::V3C_OVD || x.vuh_unit_type() == VuhUnitType::V3C_AD ||
      x.vuh_unit_type() == VuhUnitType::V3C_CAD || x.vuh_unit_type() == VuhUnitType::V3C_PVD) {
    x.m_vuh_v3c_parameter_set_id = bitstream.readBits<uint8_t>(4);
  }
  if (x.vuh_unit_type() == VuhUnitType::V3C_AVD || x.vuh_unit_type() == VuhUnitType::V3C_GVD ||
      x.vuh_unit_type() == VuhUnitType::V3C_OVD || x.vuh_unit_type() == VuhUnitType::V3C_AD ||
      x.vuh_unit_type() == VuhUnitType::V3C_PVD) {
    x.m_vuh_atlas_id = AtlasId::decodeFrom(bitstream);
  }
  if (x.vuh_unit_type() == VuhUnitType::V3C_AVD) {
    x.m_vuh_attribute_index = bitstream.readBits<uint8_t>(7);
    x.m_vuh_attribute_partition_index = bitstream.readBits<uint8_t>(5);
    x.m_vuh_map_index = bitstream.readBits<uint8_t>(4);
    x.m_vuh_auxiliary_video_flag = bitstream.getFlag();
  } else if (x.vuh_unit_type() == VuhUnitType::V3C_GVD) {
    x.m_vuh_map_index = bitstream.readBits<uint8_t>(4);
    x.m_vuh_auxiliary_video_flag = bitstream.getFlag();
    bitstream.readBits<uint16_t>(12);
  } else if (x.vuh_unit_type() == VuhUnitType::V3C_OVD ||
             x.vuh_unit_type() == VuhUnitType::V3C_AD ||
             x.vuh_unit_type() == VuhUnitType::V3C_PVD) {
    bitstream.readBits<uint32_t>(17);
  } else if (x.vuh_unit_type() == VuhUnitType::V3C_CAD) {
    bitstream.readBits<uint32_t>(23);
  } else {
    bitstream.readBits<uint32_t>(27);
  }

  return x;
}

void V3cUnitHeader::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};
  bitstream.writeBits(vuh_unit_type(), 5);

  if (vuh_unit_type() == VuhUnitType::V3C_AVD || vuh_unit_type() == VuhUnitType::V3C_GVD ||
      vuh_unit_type() == VuhUnitType::V3C_OVD || vuh_unit_type() == VuhUnitType::V3C_AD ||
      vuh_unit_type() == VuhUnitType::V3C_CAD || vuh_unit_type() == VuhUnitType::V3C_PVD) {
    bitstream.writeBits(vuh_v3c_parameter_set_id(), 4);
  }
  if (vuh_unit_type() == VuhUnitType::V3C_AVD || vuh_unit_type() == VuhUnitType::V3C_GVD ||
      vuh_unit_type() == VuhUnitType::V3C_OVD || vuh_unit_type() == VuhUnitType::V3C_AD ||
      vuh_unit_type() == VuhUnitType::V3C_PVD) {
    vuh_atlas_id().encodeTo(bitstream);
  }
  if (vuh_unit_type() == VuhUnitType::V3C_AVD) {
    bitstream.writeBits(vuh_attribute_index(), 7);
    bitstream.writeBits(vuh_attribute_partition_index(), 5);
    bitstream.writeBits(vuh_map_index(), 4);
    bitstream.putFlag(vuh_auxiliary_video_flag());
  } else if (vuh_unit_type() == VuhUnitType::V3C_GVD) {
    bitstream.writeBits(vuh_map_index(), 4);
    bitstream.putFlag(vuh_auxiliary_video_flag());
    bitstream.writeBits(0, 12);
  } else if (vuh_unit_type() == VuhUnitType::V3C_OVD || vuh_unit_type() == VuhUnitType::V3C_AD ||
             vuh_unit_type() == VuhUnitType::V3C_PVD) {
    bitstream.writeBits(0, 17);
  } else if (vuh_unit_type() == VuhUnitType::V3C_CAD) {
    bitstream.writeBits(0, 23);
  } else {
    bitstream.writeBits(0, 27);
  }
}

auto V3cUnitPayload::v3c_parameter_set() const noexcept -> const V3cParameterSet & {
  PRECONDITION(std::holds_alternative<V3cParameterSet>(m_payload));
  return *std::get_if<V3cParameterSet>(&m_payload);
}

auto V3cUnitPayload::atlas_sub_bitstream() const noexcept -> const AtlasSubBitstream & {
  PRECONDITION(std::holds_alternative<AtlasSubBitstream>(m_payload));
  return *std::get_if<AtlasSubBitstream>(&m_payload);
}

auto V3cUnitPayload::video_sub_bitstream() const noexcept -> const VideoSubBitstream & {
  PRECONDITION(std::holds_alternative<VideoSubBitstream>(m_payload));
  return *std::get_if<VideoSubBitstream>(&m_payload);
}

auto operator<<(std::ostream &stream, const V3cUnitPayload &x) -> std::ostream & {
  visit(overload([&](const std::monostate & /* unused */) { stream << "[unknown]\n"; },
                 [&](const auto &payload) { stream << payload; }),
        x.payload());
  return stream;
}

auto V3cUnitPayload::operator==(const V3cUnitPayload &other) const noexcept -> bool {
  return m_payload == other.m_payload;
}

auto V3cUnitPayload::operator!=(const V3cUnitPayload &other) const noexcept -> bool {
  return !operator==(other);
}

auto V3cUnitPayload::decodeFrom(std::istream &stream, const V3cUnitHeader &vuh) -> V3cUnitPayload {
  if (vuh.vuh_unit_type() == VuhUnitType::V3C_VPS) {
    return V3cUnitPayload{V3cParameterSet::decodeFrom(stream)};
  }
  if (vuh.vuh_unit_type() == VuhUnitType::V3C_AD || vuh.vuh_unit_type() == VuhUnitType::V3C_CAD) {
    return V3cUnitPayload{AtlasSubBitstream::decodeFrom(stream)};
  }
  if (vuh.vuh_unit_type() == VuhUnitType::V3C_OVD || vuh.vuh_unit_type() == VuhUnitType::V3C_GVD ||
      vuh.vuh_unit_type() == VuhUnitType::V3C_AVD || vuh.vuh_unit_type() == VuhUnitType::V3C_PVD) {
    return V3cUnitPayload{VideoSubBitstream::decodeFrom(stream)};
  }
  return V3cUnitPayload{std::monostate{}};
}

void V3cUnitPayload::encodeTo(std::ostream &stream, const V3cUnitHeader & /* vuh */) const {
  visit(overload([&](const std::monostate & /* unused */) { V3CBITSTREAM_ERROR("No payload"); },
                 [&](const auto &payload) { payload.encodeTo(stream); }),
        payload());
}

auto operator<<(std::ostream &stream, const V3cUnit &x) -> std::ostream & {
  return stream << x.v3c_unit_header() << x.v3c_unit_payload();
}

auto V3cUnit::operator==(const V3cUnit &other) const -> bool {
  return v3c_unit_header() == other.v3c_unit_header() &&
         v3c_unit_payload() == other.v3c_unit_payload();
}

auto V3cUnit::operator!=(const V3cUnit &other) const -> bool { return !operator==(other); }

auto V3cUnit::decodeFrom(std::istream &stream, size_t numBytesInV3CUnit) -> V3cUnit {
  const auto endPosition = stream.tellg() + std::streamoff(numBytesInV3CUnit);
  const auto v3c_unit_header = V3cUnitHeader::decodeFrom(stream);
  const auto v3c_payload = V3cUnitPayload::decodeFrom(stream, v3c_unit_header);
  VERIFY_V3CBITSTREAM(stream.tellg() <= endPosition);
  return V3cUnit{v3c_unit_header, v3c_payload};
}

auto V3cUnit::encodeTo(std::ostream &stream) const -> size_t {
  const auto position = stream.tellp();
  v3c_unit_header().encodeTo(stream);
  v3c_unit_payload().encodeTo(stream, v3c_unit_header());
  return static_cast<size_t>(stream.tellp() - position);
}

auto commonAtlasVuhs(const V3cParameterSet &vps) -> std::vector<V3cUnitHeader> {
  return std::vector{V3cUnitHeader::cad(vps.vps_v3c_parameter_set_id())};
}

auto atlasVuhs(const V3cParameterSet &vps) -> std::vector<V3cUnitHeader> {
  auto result = std::vector<V3cUnitHeader>(vps.vps_atlas_count_minus1() + size_t{1});

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    result[k] = V3cUnitHeader::ad(vps.vps_v3c_parameter_set_id(), vps.vps_atlas_id(k));
  }
  return result;
}

auto videoVuhs(const V3cParameterSet &vps) -> std::vector<V3cUnitHeader> {
  auto result = std::vector<V3cUnitHeader>{};
  const auto vpsId = vps.vps_v3c_parameter_set_id();

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto atlasId = vps.vps_atlas_id(k);

    if (vps.vps_occupancy_video_present_flag(atlasId)) {
      result.push_back(V3cUnitHeader::ovd(vpsId, atlasId));
    }
    if (vps.vps_geometry_video_present_flag(atlasId)) {
      result.push_back(V3cUnitHeader::gvd(vpsId, atlasId));
    }
    if (vps.vps_attribute_video_present_flag(atlasId)) {
      const auto &ai = vps.attribute_information(atlasId);

      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        result.push_back(V3cUnitHeader::avd(vpsId, atlasId, i));
      }
    }
    if (vps.vps_packed_video_present_flag(atlasId)) {
      result.push_back(V3cUnitHeader::pvd(vpsId, atlasId));
    }
  }
  return result;
}
} // namespace TMIV::MivBitstream
