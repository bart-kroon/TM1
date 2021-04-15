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

#include <catch2/catch.hpp>

#include <TMIV/Encoder/Multiplexer.h>

#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <fmt/printf.h>
#include <iostream>

using TMIV::MivBitstream::AiAttributeTypeId;
using TMIV::MivBitstream::AtlasId;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::AttributeInformation;
using TMIV::MivBitstream::GeometryInformation;
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;
using TMIV::MivBitstream::OccupancyInformation;
using TMIV::MivBitstream::PackingInformation;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::SampleStreamNalUnit;
using TMIV::MivBitstream::SampleStreamV3cHeader;
using TMIV::MivBitstream::SampleStreamV3cUnit;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VuhUnitType;

using namespace std::string_view_literals;

enum class VpsContent {
  default_constructed,
  two_maps,
  with_auxiliary_video,
  two_atlases_and_some_videos,
  two_atlases_one_with_packed_video
};

template <typename Payload> auto createTestV3cUnit(const V3cUnitHeader &vuh, Payload &&payload) {
  const auto vu = V3cUnit{vuh, payload};

  std::ostringstream stream;
  vu.encodeTo(stream);
  return stream.str();
}

auto createVps(VpsContent vpsContent) {
  auto vps = V3cParameterSet{};

  switch (vpsContent) {
  case VpsContent::default_constructed:
    break;
  case VpsContent::two_maps:
    vps.vps_map_count_minus1(AtlasId{0}, 1);
    break;
  case VpsContent::with_auxiliary_video:
    vps.vps_auxiliary_video_present_flag(AtlasId{0}, true);
    break;
  case VpsContent::two_atlases_one_with_packed_video:
    vps.vps_atlas_count_minus1(1)
        .vps_atlas_id(1, AtlasId{1})
        .vps_extension_present_flag(true)
        .vps_packing_information_present_flag(true)
        .vps_packed_video_present_flag(AtlasId{0}, true)
        .packing_information(AtlasId{0}, PackingInformation{})
        .vps_geometry_video_present_flag(AtlasId{1}, true)
        .geometry_information(AtlasId{1}, GeometryInformation{});
    break;
  case VpsContent::two_atlases_and_some_videos:
    AttributeInformation ai{};
    vps.vps_atlas_count_minus1(1)
        .vps_atlas_id(1, AtlasId{1})
        .vps_geometry_video_present_flag(AtlasId{0}, true)
        .geometry_information(AtlasId{0}, GeometryInformation{})
        .vps_geometry_video_present_flag(AtlasId{1}, true)
        .geometry_information(AtlasId{1}, GeometryInformation{})
        .vps_attribute_video_present_flag(AtlasId{0}, true)
        .attribute_information(AtlasId{0},
                               ai.ai_attribute_count(2)
                                   .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_NORMAL)
                                   .ai_attribute_type_id(1, AiAttributeTypeId::ATTR_TEXTURE))
        .vps_occupancy_video_present_flag(AtlasId{1}, true)
        .occupancy_information(AtlasId{1}, OccupancyInformation{});
    break;
  };

  return vps;
}

auto createTestVpsBitstream(VpsContent vpsType = VpsContent::default_constructed) {
  const auto vuh = V3cUnitHeader{VuhUnitType::V3C_VPS};
  const auto vps = createVps(vpsType);
  return createTestV3cUnit(vuh, vps);
}

auto createTestAtlasData(const SampleStreamNalHeader &ssnh) { return AtlasSubBitstream{ssnh}; }

auto createTestAdBitstream() {
  const auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
  const auto ssnh = SampleStreamNalHeader{0};
  const auto ad = createTestAtlasData(ssnh);
  return createTestV3cUnit(vuh, ad);
}
auto createTestBitstream(VpsContent vpsType = VpsContent::default_constructed) {
  std::ostringstream stream;

  const auto ssvh = SampleStreamV3cHeader{2};
  ssvh.encodeTo(stream);

  const auto ssvu1 = SampleStreamV3cUnit{createTestVpsBitstream(vpsType)};
  ssvu1.encodeTo(stream, ssvh);

  const auto ssvu2 = SampleStreamV3cUnit{createTestAdBitstream()};
  ssvu2.encodeTo(stream, ssvh);

  return stream.str();
}

auto checkStreamBeginning(std::istream &stream, VpsContent expectedVpsContent)
    -> SampleStreamV3cHeader {
  const auto ssvh = SampleStreamV3cHeader::decodeFrom(stream);
  REQUIRE(ssvh.ssvh_unit_size_precision_bytes_minus1() == 0U);
  const auto ssvu0 = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  std::istringstream substream{ssvu0.ssvu_v3c_unit()};
  const auto vuh = V3cUnitHeader::decodeFrom(substream);
  REQUIRE(vuh.vuh_unit_type() == VuhUnitType::V3C_VPS);
  const auto vps = V3cParameterSet::decodeFrom(substream);
  REQUIRE(vps == createVps(expectedVpsContent));
  const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  REQUIRE(ssvu.ssvu_v3c_unit_size() == 5U);
  return ssvh;
}

TEST_CASE("Only VPS and one atlas") {
  TMIV::Encoder::Multiplexer unit{};
  std::istringstream inStream{createTestBitstream()};

  unit.readInputBitstream(inStream);
  unit.appendVideoSubBitstreams();
  std::ostringstream outStream;
  unit.writeOutputBitstream(outStream);

  std::istringstream result{outStream.str()};
  checkStreamBeginning(result, VpsContent::default_constructed);
}

auto createTestBitstreamWithWrongVpsOrder() {
  std::ostringstream stream;

  const auto ssvh = SampleStreamV3cHeader{2};
  ssvh.encodeTo(stream);

  const auto ssvu2 = SampleStreamV3cUnit{createTestAdBitstream()};
  ssvu2.encodeTo(stream, ssvh);

  const auto ssvu1 = SampleStreamV3cUnit{createTestVpsBitstream()};
  ssvu1.encodeTo(stream, ssvh);

  return stream.str();
}

TEST_CASE("Unsupported input") {
  TMIV::Encoder::Multiplexer unit{};

  SECTION("VPS with multiple maps") {
    std::istringstream inStream{createTestBitstream(VpsContent::two_maps)};

    unit.readInputBitstream(inStream);
    REQUIRE_THROWS(unit.appendVideoSubBitstreams());
  }

  SECTION("VPS with auxiliary video") {
    std::istringstream inStream{createTestBitstream(VpsContent::with_auxiliary_video)};

    unit.readInputBitstream(inStream);
    REQUIRE_THROWS(unit.appendVideoSubBitstreams());
  }

  SECTION("VPS is not first V3C unit") {
    std::istringstream inStream{createTestBitstreamWithWrongVpsOrder()};

    REQUIRE_THROWS(unit.readInputBitstream(inStream));
  }
}

auto makeMultiplexerWithFakeVideoBitstreamServers() -> TMIV::Encoder::Multiplexer {
  TMIV::Encoder::Multiplexer unit{};

  unit.setAttributeVideoBitstreamServer(
      [](AiAttributeTypeId typeId, const AtlasId &atlasId, int attributeIdx) {
        return std::make_unique<std::istringstream>(
            fmt::format("\0\0\0\1test_attribute typeId={} attributeIdx={} atlasId={}"sv, typeId,
                        attributeIdx, atlasId));
      });

  unit.setGeometryVideoBitstreamServer([](const AtlasId &atlasId) {
    return std::make_unique<std::istringstream>(
        fmt::format("\0\0\0\1test_geometry atlasId={}"sv, atlasId));
  });

  unit.setOccupancyVideoBitstreamServer([](const AtlasId &atlasId) {
    return std::make_unique<std::istringstream>(
        fmt::format("\0\0\0\1test_occupancy atlasId={}"sv, atlasId));
  });

  unit.setPackedVideoBitstreamServer([](const AtlasId &atlasId) {
    return std::make_unique<std::istringstream>(
        fmt::format("\0\0\0\1test_packed atlasId={}"sv, atlasId));
  });

  return unit;
}

void requireThatNextV3cUnitContains(std::istream &stream, const SampleStreamV3cHeader &ssvh,
                                    const std::string &&message) {
  const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  REQUIRE(ssvu.ssvu_v3c_unit().find(message) != std::string::npos);
}

TEST_CASE("VPS with two atlases") {
  auto unit = makeMultiplexerWithFakeVideoBitstreamServers();
  std::istringstream inStream{createTestBitstream(VpsContent::two_atlases_and_some_videos)};

  unit.readInputBitstream(inStream);
  unit.appendVideoSubBitstreams();
  std::ostringstream outStream;
  unit.writeOutputBitstream(outStream);

  std::istringstream result{outStream.str()};
  const auto ssvh = checkStreamBeginning(result, VpsContent::two_atlases_and_some_videos);
  requireThatNextV3cUnitContains(result, ssvh, "test_geometry atlasId=0");
  requireThatNextV3cUnitContains(result, ssvh,
                                 "test_attribute typeId=ATTR_NORMAL attributeIdx=0 atlasId=0");
  requireThatNextV3cUnitContains(result, ssvh,
                                 "test_attribute typeId=ATTR_TEXTURE attributeIdx=1 atlasId=0");
  requireThatNextV3cUnitContains(result, ssvh, "test_geometry atlasId=1");
  requireThatNextV3cUnitContains(result, ssvh, "test_occupancy atlasId=1");
}

TEST_CASE("VPS with two atlases, one with packed video") {
  auto unit = makeMultiplexerWithFakeVideoBitstreamServers();
  std::istringstream inStream{createTestBitstream(VpsContent::two_atlases_one_with_packed_video)};

  unit.readInputBitstream(inStream);
  unit.addPackingInformation();
  unit.appendVideoSubBitstreams();
  std::ostringstream outStream;
  unit.writeOutputBitstream(outStream);

  std::istringstream result{outStream.str()};
  const auto ssvh = checkStreamBeginning(result, VpsContent::two_atlases_one_with_packed_video);
  requireThatNextV3cUnitContains(result, ssvh, "test_packed atlasId=0");
  requireThatNextV3cUnitContains(result, ssvh, "test_geometry atlasId=1");
}
