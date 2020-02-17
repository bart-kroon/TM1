/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/MivBitstream/VpccUnit.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Common.h>

#include "verify.h"

using namespace std;
using TMIV::Common::overload;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const VuhUnitType x) -> ostream & {
  switch (x) {
  case VuhUnitType::VPCC_VPS:
    return stream << "VPCC_VPS";
  case VuhUnitType::VPCC_AVD:
    return stream << "VPCC_AVD";
  case VuhUnitType::VPCC_GVD:
    return stream << "VPCC_GVD";
  case VuhUnitType::VPCC_OVD:
    return stream << "VPCC_OVD";
  case VuhUnitType::VPCC_AD:
    return stream << "VPCC_AD";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto VpccUnitHeader::vuh_vpcc_parameter_set_id() const noexcept -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(
      m_vuh_unit_type == VuhUnitType::VPCC_AVD || m_vuh_unit_type == VuhUnitType::VPCC_GVD ||
      m_vuh_unit_type == VuhUnitType::VPCC_OVD || m_vuh_unit_type == VuhUnitType::VPCC_AD);
  return m_vuh_vpcc_parameter_set_id;
}

auto VpccUnitHeader::vuh_atlas_id() const noexcept -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(
      m_vuh_unit_type == VuhUnitType::VPCC_AVD || m_vuh_unit_type == VuhUnitType::VPCC_GVD ||
      m_vuh_unit_type == VuhUnitType::VPCC_OVD || m_vuh_unit_type == VuhUnitType::VPCC_AD);
  return m_vuh_atlas_id;
}

auto VpccUnitHeader::vuh_attribute_index() const noexcept -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD);
  return m_vuh_attribute_index;
}

auto VpccUnitHeader::vuh_attribute_dimension_index() const noexcept -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD);
  return m_vuh_attribute_dimension_index;
}

auto VpccUnitHeader::vuh_map_index() const noexcept -> std::uint8_t {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD ||
                       m_vuh_unit_type == VuhUnitType::VPCC_GVD);
  return m_vuh_map_index;
}

auto VpccUnitHeader::vuh_raw_video_flag() const noexcept -> bool {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD ||
                       m_vuh_unit_type == VuhUnitType::VPCC_GVD);
  return m_vuh_raw_video_flag;
}

auto VpccUnitHeader::vuh_vpcc_parameter_set_id(const uint8_t value) noexcept -> VpccUnitHeader & {
  VERIFY_VPCCBITSTREAM(
      m_vuh_unit_type == VuhUnitType::VPCC_AVD || m_vuh_unit_type == VuhUnitType::VPCC_GVD ||
      m_vuh_unit_type == VuhUnitType::VPCC_OVD || m_vuh_unit_type == VuhUnitType::VPCC_AD);
  m_vuh_vpcc_parameter_set_id = value;
  return *this;
}

auto VpccUnitHeader::vuh_atlas_id(const uint8_t value) noexcept -> VpccUnitHeader & {
  VERIFY_VPCCBITSTREAM(
      m_vuh_unit_type == VuhUnitType::VPCC_AVD || m_vuh_unit_type == VuhUnitType::VPCC_GVD ||
      m_vuh_unit_type == VuhUnitType::VPCC_OVD || m_vuh_unit_type == VuhUnitType::VPCC_AD);
  m_vuh_atlas_id = value;
  return *this;
}

auto VpccUnitHeader::vuh_attribute_index(const uint8_t value) noexcept -> VpccUnitHeader & {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD);
  m_vuh_attribute_index = value;
  return *this;
}

auto VpccUnitHeader::vuh_attribute_dimension_index(const uint8_t value) noexcept
    -> VpccUnitHeader & {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD);
  m_vuh_attribute_dimension_index = value;
  return *this;
}

auto VpccUnitHeader::vuh_map_index(const uint8_t value) noexcept -> VpccUnitHeader & {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD ||
                       m_vuh_unit_type == VuhUnitType::VPCC_GVD);
  m_vuh_map_index = value;
  return *this;
}

auto VpccUnitHeader::vuh_raw_video_flag(const bool value) noexcept -> VpccUnitHeader & {
  VERIFY_VPCCBITSTREAM(m_vuh_unit_type == VuhUnitType::VPCC_AVD ||
                       m_vuh_unit_type == VuhUnitType::VPCC_GVD);
  m_vuh_raw_video_flag = value;
  return *this;
}

auto operator<<(ostream &stream, const VpccUnitHeader &x) -> ostream & {
  stream << "vuh_unit_type=" << x.vuh_unit_type();
  if (x.vuh_unit_type() == VuhUnitType::VPCC_AVD || x.vuh_unit_type() == VuhUnitType::VPCC_GVD ||
      x.vuh_unit_type() == VuhUnitType::VPCC_OVD || x.vuh_unit_type() == VuhUnitType::VPCC_AD) {
    stream << "\nvuh_vpcc_parameter_set_id=" << int(x.vuh_vpcc_parameter_set_id())
           << "\nvuh_atlas_id=" << int(x.vuh_atlas_id());
  }
  if (x.vuh_unit_type() == VuhUnitType::VPCC_AVD) {
    stream << "\nvuh_attribute_index=" << int(x.vuh_attribute_index())
           << "\nvuh_attribute_dimension_index=" << int(x.vuh_attribute_dimension_index())
           << "\nvuh_map_index=" << int(x.vuh_map_index()) << "\nvuh_raw_video_flag=" << boolalpha
           << x.vuh_raw_video_flag();
  } else if (x.vuh_unit_type() == VuhUnitType::VPCC_GVD) {
    stream << "\nvuh_map_index=" << int(x.vuh_map_index()) << "\nvuh_raw_video_flag=" << boolalpha
           << x.vuh_raw_video_flag();
  }
  return stream << '\n';
}

auto VpccUnitHeader::operator==(const VpccUnitHeader &other) const noexcept -> bool {
  if (vuh_unit_type() != other.vuh_unit_type()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::VPCC_VPS) {
    return true;
  }
  if (vuh_vpcc_parameter_set_id() != other.vuh_vpcc_parameter_set_id() ||
      vuh_atlas_id() != other.vuh_atlas_id()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::VPCC_OVD || vuh_unit_type() == VuhUnitType::VPCC_AD) {
    return true;
  }
  if (vuh_map_index() != other.vuh_map_index() ||
      vuh_raw_video_flag() != other.vuh_raw_video_flag()) {
    return false;
  }
  if (vuh_unit_type() == VuhUnitType::VPCC_GVD) {
    return true;
  }
  if (vuh_unit_type() == VuhUnitType::VPCC_AVD) {
    return vuh_attribute_index() == other.vuh_attribute_index() &&
           vuh_attribute_dimension_index() == other.vuh_attribute_dimension_index();
  }
  VPCCBITSTREAM_ERROR("Unknown vuh_unit_type");
}

auto VpccUnitHeader::operator!=(const VpccUnitHeader &other) const noexcept -> bool {
  return !operator==(other);
}

auto VpccUnitHeader::decodeFrom(istream &stream, const vector<VpccParameterSet> &vpses)
    -> VpccUnitHeader {
  InputBitstream bitstream{stream};
  auto x = VpccUnitHeader{VuhUnitType(bitstream.readBits(5))};

  if (x.vuh_unit_type() == VuhUnitType::VPCC_AVD || x.vuh_unit_type() == VuhUnitType::VPCC_GVD ||
      x.vuh_unit_type() == VuhUnitType::VPCC_OVD || x.vuh_unit_type() == VuhUnitType::VPCC_AD) {
    x.vuh_vpcc_parameter_set_id(uint8_t(bitstream.readBits(4)));
    VERIFY_VPCCBITSTREAM(x.vuh_vpcc_parameter_set_id() < vpses.size());

    x.vuh_atlas_id(uint8_t(bitstream.readBits(6)));
    VERIFY_VPCCBITSTREAM(x.vuh_atlas_id() <=
                         vpses[x.vuh_vpcc_parameter_set_id()].vps_atlas_count_minus1());
  }
  if (x.vuh_unit_type() == VuhUnitType::VPCC_AVD) {
    x.vuh_attribute_index(uint8_t(bitstream.readBits(7)));
    VERIFY_VPCCBITSTREAM(x.vuh_attribute_index() < vpses[x.vuh_vpcc_parameter_set_id()]
                                                       .attribute_information(x.vuh_atlas_id())
                                                       .ai_attribute_count());

    x.vuh_attribute_dimension_index(uint8_t(bitstream.readBits(5)));
    VERIFY_MIVBITSTREAM(x.vuh_attribute_dimension_index() == 0);

    x.vuh_map_index(uint8_t(bitstream.readBits(4)));
    VERIFY_VPCCBITSTREAM(x.vuh_map_index() <
                         vpses[x.vuh_vpcc_parameter_set_id()].vps_map_count(x.vuh_atlas_id()));

    x.vuh_raw_video_flag(bitstream.getFlag());
    VERIFY_MIVBITSTREAM(!x.vuh_raw_video_flag());

  } else if (x.vuh_unit_type() == VuhUnitType::VPCC_GVD) {
    x.vuh_map_index(uint8_t(bitstream.readBits(4)));
    VERIFY_VPCCBITSTREAM(x.vuh_map_index() <
                         vpses[x.vuh_vpcc_parameter_set_id()].vps_map_count(x.vuh_atlas_id()));

    x.vuh_raw_video_flag(bitstream.getFlag());
    VERIFY_MIVBITSTREAM(!x.vuh_raw_video_flag());

    bitstream.readBits(12);
  } else if (x.vuh_unit_type() == VuhUnitType::VPCC_OVD ||
             x.vuh_unit_type() == VuhUnitType::VPCC_AD) {
    bitstream.readBits(17);
  } else {
    bitstream.readBits(27);
  }

  return x;
}

void VpccUnitHeader::encodeTo(ostream &stream, const vector<VpccParameterSet> &vpses) const {
  OutputBitstream bitstream{stream};
  bitstream.writeBits(unsigned(vuh_unit_type()), 5);

  if (vuh_unit_type() == VuhUnitType::VPCC_AVD || vuh_unit_type() == VuhUnitType::VPCC_GVD ||
      vuh_unit_type() == VuhUnitType::VPCC_OVD || vuh_unit_type() == VuhUnitType::VPCC_AD) {
    VERIFY_VPCCBITSTREAM(vuh_vpcc_parameter_set_id() < vpses.size());
    VERIFY_VPCCBITSTREAM(vuh_vpcc_parameter_set_id() <= 15);
    bitstream.writeBits(vuh_vpcc_parameter_set_id(), 4);

    VERIFY_VPCCBITSTREAM(vuh_atlas_id() <=
                         vpses[vuh_vpcc_parameter_set_id()].vps_atlas_count_minus1());
    VERIFY_VPCCBITSTREAM(vuh_atlas_id() <= 63);
    bitstream.writeBits(vuh_atlas_id(), 6);
  }
  if (vuh_unit_type() == VuhUnitType::VPCC_AVD) {
    VERIFY_VPCCBITSTREAM(vuh_attribute_index() < vpses[vuh_vpcc_parameter_set_id()]
                                                     .attribute_information(vuh_atlas_id())
                                                     .ai_attribute_count());
    VERIFY_VPCCBITSTREAM(vuh_attribute_index() <= 127);
    bitstream.writeBits(vuh_attribute_index(), 7);

    VERIFY_MIVBITSTREAM(vuh_attribute_dimension_index() == 0);
    bitstream.writeBits(vuh_attribute_dimension_index(), 5);

    VERIFY_VPCCBITSTREAM(vuh_map_index() <
                         vpses[vuh_vpcc_parameter_set_id()].vps_map_count(vuh_atlas_id()));
    VERIFY_VPCCBITSTREAM(vuh_map_index() <= 15);
    bitstream.writeBits(vuh_map_index(), 4);

    VERIFY_MIVBITSTREAM(!vuh_raw_video_flag());
    bitstream.putFlag(vuh_raw_video_flag());

  } else if (vuh_unit_type() == VuhUnitType::VPCC_GVD) {
    VERIFY_VPCCBITSTREAM(vuh_map_index() <
                         vpses[vuh_vpcc_parameter_set_id()].vps_map_count(vuh_atlas_id()));
    VERIFY_VPCCBITSTREAM(vuh_map_index() <= 15);
    bitstream.writeBits(vuh_map_index(), 4);

    VERIFY_MIVBITSTREAM(!vuh_raw_video_flag());
    bitstream.putFlag(vuh_raw_video_flag());

    bitstream.writeBits(0, 12);
  } else if (vuh_unit_type() == VuhUnitType::VPCC_OVD || vuh_unit_type() == VuhUnitType::VPCC_AD) {
    bitstream.writeBits(0, 17);
  } else {
    bitstream.writeBits(0, 27);
  }
}

auto VpccPayload::vpcc_parameter_set() const noexcept -> const VpccParameterSet & {
  VERIFY_VPCCBITSTREAM(holds_alternative<VpccParameterSet>(m_payload));
  return *get_if<VpccParameterSet>(&m_payload);
}

auto VpccPayload::atlas_sub_bitstream() const noexcept -> const AtlasSubBitstream & {
  VERIFY_VPCCBITSTREAM(holds_alternative<AtlasSubBitstream>(m_payload));
  return *get_if<AtlasSubBitstream>(&m_payload);
}

auto VpccPayload::video_sub_bitstream() const noexcept -> const VideoSubBitstream & {
  VERIFY_VPCCBITSTREAM(holds_alternative<VideoSubBitstream>(m_payload));
  return *get_if<VideoSubBitstream>(&m_payload);
}

auto operator<<(ostream &stream, const VpccPayload &x) -> ostream & {
  visit(overload([&](const monostate & /* unused */) { stream << "[unknown]\n"; },
                 [&](const auto &payload) { stream << payload; }),
        x.payload());
  return stream;
}

auto VpccPayload::operator==(const VpccPayload &other) const noexcept -> bool {
  return m_payload == other.m_payload;
}

auto VpccPayload::operator!=(const VpccPayload &other) const noexcept -> bool {
  return !operator==(other);
}

auto VpccPayload::decodeFrom(istream &stream, const VpccUnitHeader &vuh) -> VpccPayload {
  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_VPS) {
    return VpccPayload{VpccParameterSet::decodeFrom(stream)};
  }
  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_AD) {
    return VpccPayload{AtlasSubBitstream::decodeFrom(stream)};
  }
  if (vuh.vuh_unit_type() == VuhUnitType::VPCC_OVD ||
      vuh.vuh_unit_type() == VuhUnitType::VPCC_GVD ||
      vuh.vuh_unit_type() == VuhUnitType::VPCC_AVD) {
    return VpccPayload{VideoSubBitstream::decodeFrom(stream)};
  }
  return VpccPayload{monostate{}};
}

void VpccPayload::encodeTo(ostream &stream, const VpccUnitHeader & /* vuh */) const {
  visit(overload([&](const monostate & /* unused */) { VPCCBITSTREAM_ERROR("No payload"); },
                 [&](const auto &payload) { payload.encodeTo(stream); }),
        payload());
}

auto operator<<(ostream &stream, const VpccUnit &x) -> ostream & {
  return stream << x.vpcc_unit_header() << x.vpcc_payload();
}

auto VpccUnit::operator==(const VpccUnit &other) const noexcept -> bool {
  return vpcc_unit_header() == other.vpcc_unit_header() && vpcc_payload() == other.vpcc_payload();
}

auto VpccUnit::operator!=(const VpccUnit &other) const noexcept -> bool {
  return !operator==(other);
}

auto VpccUnit::decodeFrom(istream &stream, const std::vector<VpccParameterSet> &vpses,
                          size_t numBytesInVPCCUnit) -> VpccUnit {
  const auto endPosition = stream.tellg() + streamoff(numBytesInVPCCUnit);
  const auto vpcc_unit_header = VpccUnitHeader::decodeFrom(stream, vpses);
  const auto vpcc_payload = VpccPayload::decodeFrom(stream, vpcc_unit_header);
  VERIFY_VPCCBITSTREAM(stream.tellg() <= endPosition);
  return VpccUnit{vpcc_unit_header, vpcc_payload};
}

auto VpccUnit::encodeTo(ostream &stream, const std::vector<VpccParameterSet> &vpses) const
    -> size_t {
  const auto position = stream.tellp();
  vpcc_unit_header().encodeTo(stream, vpses);
  vpcc_payload().encodeTo(stream, vpcc_unit_header());
  return size_t(stream.tellp() - position);
}
} // namespace TMIV::MivBitstream
