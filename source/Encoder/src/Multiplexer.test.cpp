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

#include <TMIV/Common/Json.h>
#include <TMIV/Encoder/Multiplexer.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <fmt/printf.h>
#include <iostream>

using TMIV::Common::Json;
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
  two_atlases_one_with_packed_video,
  one_atlas_three_videos
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
  case VpsContent::two_atlases_one_with_packed_video: {
    PackingInformation packInfo{};
    packInfo.pin_occupancy_2d_bit_depth_minus1(7)
        .pin_occupancy_MSB_align_flag(true)
        .pin_lossy_occupancy_compression_threshold(64);
    vps.vps_atlas_count_minus1(1)
        .vps_atlas_id(1, AtlasId{1})
        .vps_packed_video_present_flag(AtlasId{0}, true)
        .packing_information(AtlasId{0}, packInfo)
        .vps_geometry_video_present_flag(AtlasId{1}, true)
        .geometry_information(AtlasId{1}, GeometryInformation{});
  } break;
  case VpsContent::two_atlases_and_some_videos: {
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
  } break;
  case VpsContent::one_atlas_three_videos: {
    AttributeInformation ai{};
    vps.vps_atlas_count_minus1(0)
        .vps_atlas_id(0, AtlasId{0})
        .vps_geometry_video_present_flag(AtlasId{0}, true)
        .geometry_information(AtlasId{0}, GeometryInformation{})
        .vps_attribute_video_present_flag(AtlasId{0}, true)
        .attribute_information(AtlasId{0}, ai.ai_attribute_count(1).ai_attribute_type_id(
                                               0, AiAttributeTypeId::ATTR_TEXTURE))
        .vps_occupancy_video_present_flag(AtlasId{0}, true)
        .occupancy_information(AtlasId{0}, OccupancyInformation{});
  } break;
  };

  return vps;
}

auto createTestVpsBitstream(VpsContent vpsType = VpsContent::default_constructed) {
  const auto vuh = V3cUnitHeader::vps();
  const auto vps = createVps(vpsType);
  return createTestV3cUnit(vuh, vps);
}

auto createTestAtlasData(const SampleStreamNalHeader &ssnh) { return AtlasSubBitstream{ssnh}; }

auto createTestAdBitstream() {
  const auto vuh = V3cUnitHeader::ad(0, {});
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

auto checkHeaderAndVerifyThatFirstUnitIsVps(std::istream &stream)
    -> std::pair<V3cParameterSet, SampleStreamV3cHeader> {
  const auto ssvh = SampleStreamV3cHeader::decodeFrom(stream);
  REQUIRE(ssvh.ssvh_unit_size_precision_bytes_minus1() == 0U);
  const auto ssvu0 = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  std::istringstream substream{ssvu0.ssvu_v3c_unit()};
  const auto vuh = V3cUnitHeader::decodeFrom(substream);
  REQUIRE(vuh.vuh_unit_type() == VuhUnitType::V3C_VPS);
  return {V3cParameterSet::decodeFrom(substream), ssvh};
}

auto checkStreamBeginning(std::istream &stream, VpsContent expectedVpsContent)
    -> SampleStreamV3cHeader {
  const auto [vps, ssvh] = checkHeaderAndVerifyThatFirstUnitIsVps(stream);
  REQUIRE(vps == createVps(expectedVpsContent));
  const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  REQUIRE(ssvu.ssvu_v3c_unit_size() == 5U);
  return ssvh;
}
auto checkModifiedVPSWithPackedInformation(std::istream &stream, const Json &packingInformationNode)
    -> SampleStreamV3cHeader {
  const auto [vps, ssvh] = checkHeaderAndVerifyThatFirstUnitIsVps(stream);

  for (const auto &info : packingInformationNode.as<Json::Array>()) {
    AtlasId atlasId(info.require("pin_atlas_id").as<uint8_t>());
    const auto &vpi = vps.packing_information(atlasId);
    REQUIRE(vpi.pin_codec_id() == info.require("pin_codec_id").as<uint8_t>());
    REQUIRE(vpi.pin_occupancy_present_flag() ==
            info.require("pin_occupancy_present_flag").as<bool>());
    REQUIRE(vpi.pin_geometry_present_flag() ==
            info.require("pin_geometry_present_flag").as<bool>());
    REQUIRE(vpi.pin_attribute_present_flag() ==
            info.require("pin_attributes_present_flag").as<bool>());

    if (vpi.pin_occupancy_present_flag()) {
      REQUIRE_FALSE(vps.vps_occupancy_video_present_flag(atlasId));
    }

    if (vpi.pin_geometry_present_flag()) {
      REQUIRE_FALSE(vps.vps_geometry_video_present_flag(atlasId));
    }

    if (vpi.pin_attribute_present_flag()) {
      REQUIRE_FALSE(vps.vps_attribute_video_present_flag(atlasId));
    }

    auto regionIdx = 0;
    for (const auto &region : info.require("pin_regions").as<Json::Array>()) {
      REQUIRE(vpi.pin_region_tile_id(regionIdx) ==
              region.require("pin_region_tile_id").as<uint8_t>());
      REQUIRE(vpi.pin_region_type_id_minus2(regionIdx) ==
              region.require("pin_region_type_id_minus2").as<uint8_t>());
      REQUIRE(vpi.pin_region_top_left_x(regionIdx) ==
              region.require("pin_region_top_left_x").as<uint16_t>());
      REQUIRE(vpi.pin_region_top_left_y(regionIdx) ==
              region.require("pin_region_top_left_y").as<uint16_t>());
      REQUIRE(vpi.pin_region_width_minus1(regionIdx) ==
              region.require("pin_region_width_minus1").as<uint16_t>());
      REQUIRE(vpi.pin_region_height_minus1(regionIdx) ==
              region.require("pin_region_height_minus1").as<uint16_t>());
      REQUIRE(vpi.pin_region_unpack_top_left_x(regionIdx) ==
              region.require("pin_region_unpack_top_left_x").as<uint16_t>());
      REQUIRE(vpi.pin_region_unpack_top_left_y(regionIdx) ==
              region.require("pin_region_unpack_top_left_y").as<uint16_t>());
      REQUIRE(vpi.pin_region_rotation_flag(regionIdx) ==
              region.require("pin_region_rotation_flag").as<bool>());
      if (vpi.pinRegionTypeId(regionIdx) == TMIV::MivBitstream::VuhUnitType::V3C_AVD ||
          vpi.pinRegionTypeId(regionIdx) == TMIV::MivBitstream::VuhUnitType::V3C_GVD) {
        REQUIRE(vpi.pin_region_map_index(regionIdx) ==
                region.require("pin_region_map_index").as<uint8_t>());
        REQUIRE(vpi.pin_region_auxiliary_data_flag(regionIdx) ==
                region.require("pin_region_auxiliary_data_flag").as<bool>());
      }

      if (vpi.pinRegionTypeId(regionIdx) == TMIV::MivBitstream::VuhUnitType::V3C_AVD) {
        auto k = vpi.pin_region_attr_index(regionIdx);
        REQUIRE(k == region.require("pin_region_attr_index").as<uint8_t>());
        if (vpi.pin_attribute_dimension_minus1(k) > 0U) {
          REQUIRE(vpi.pin_region_attr_partition_index(regionIdx) ==
                  region.require("pin_region_attr_partition_index").as<uint8_t>());
        }
      }
      regionIdx++;
    }
  }
  const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  REQUIRE(ssvu.ssvu_v3c_unit_size() == 5U);
  return ssvh;
}

TEST_CASE("Only VPS and one atlas") {
  TMIV::Encoder::Multiplexer unit{Json::null};
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
  TMIV::Encoder::Multiplexer unit{Json::null};

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

auto makeMultiplexerWithFakeVideoBitstreamServers(const Json &packingInformationNode = Json::null)
    -> TMIV::Encoder::Multiplexer {
  TMIV::Encoder::Multiplexer unit{packingInformationNode};

  unit.setVideoBitstreamServer(
      [](TMIV::MivBitstream::V3cUnitHeader vuh, TMIV::MivBitstream::AiAttributeTypeId attrTypeId) {
        return std::make_unique<std::istringstream>(
            fmt::format("\0\0\0\1test {}"sv, vuh.summary(), attrTypeId));
      });

  return unit;
}

void requireThatNextV3cUnitContains(std::istream &stream, const SampleStreamV3cHeader &ssvh,
                                    const std::string &&message) {
  const auto ssvu = SampleStreamV3cUnit::decodeFrom(stream, ssvh);
  REQUIRE_THAT(ssvu.ssvu_v3c_unit(), Catch::Matchers::Contains(message));
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
  requireThatNextV3cUnitContains(result, ssvh, "V3C_GVD vps:0 atlas:0 map:0 aux:false");
  requireThatNextV3cUnitContains(result, ssvh,
                                 "V3C_AVD vps:0 atlas:0 attr:0 part:0 map:0 aux:false");
  requireThatNextV3cUnitContains(result, ssvh,
                                 "V3C_AVD vps:0 atlas:0 attr:1 part:0 map:0 aux:false");
  requireThatNextV3cUnitContains(result, ssvh, "V3C_GVD vps:0 atlas:1 map:0 aux:false");
  requireThatNextV3cUnitContains(result, ssvh, "V3C_OVD vps:0 atlas:1");
}

TEST_CASE("VPS with two atlases, one with packed video (no calling add packing information from "
          "Multiplexer)") {
  auto unit = makeMultiplexerWithFakeVideoBitstreamServers();
  std::istringstream inStream{createTestBitstream(VpsContent::two_atlases_one_with_packed_video)};

  unit.readInputBitstream(inStream);
  unit.appendVideoSubBitstreams();
  std::ostringstream outStream;
  unit.writeOutputBitstream(outStream);

  std::istringstream result{outStream.str()};
  const auto ssvh = checkStreamBeginning(result, VpsContent::two_atlases_one_with_packed_video);
  requireThatNextV3cUnitContains(result, ssvh, "V3C_PVD vps:0 atlas:0");
  requireThatNextV3cUnitContains(result, ssvh, "V3C_GVD vps:0 atlas:1 map:0 aux:false");
}

TEST_CASE("External Packing Information") {
  const auto json = Json::parse(R"({
  "packingInformation": [
    {
      "pin_atlas_id": 0,
      "pin_attributes_present_flag": true,
      "pin_codec_id": 17,
      "pin_geometry_present_flag": true,
      "pin_occupancy_present_flag": true,
      "pin_regions": [
        {
          "pin_region_attr_index": 0,
          "pin_region_attr_partition_index": 0,
          "pin_region_auxiliary_data_flag": false,
          "pin_region_height_minus1": 320,
          "pin_region_map_index": 0,
          "pin_region_rotation_flag": false,
          "pin_region_tile_id": 0,
          "pin_region_top_left_x": 0,
          "pin_region_top_left_y": 0,
          "pin_region_type_id_minus2": 2,
          "pin_region_unpack_top_left_x": 0,
          "pin_region_unpack_top_left_y": 0,
          "pin_region_width_minus1": 640
        },
        {
          "pin_region_auxiliary_data_flag": false,
          "pin_region_height_minus1": 120,
          "pin_region_map_index": 0,
          "pin_region_rotation_flag": false,
          "pin_region_tile_id": 0,
          "pin_region_top_left_x": 0,
          "pin_region_top_left_y": 320,
          "pin_region_type_id_minus2": 1,
          "pin_region_unpack_top_left_x": 0,
          "pin_region_unpack_top_left_y": 0,
          "pin_region_width_minus1": 320
        },
        {
          "pin_region_auxiliary_data_flag": false,
          "pin_region_height_minus1": 120,
          "pin_region_map_index": 0,
          "pin_region_rotation_flag": false,
          "pin_region_tile_id": 0,
          "pin_region_top_left_x": 320,
          "pin_region_top_left_y": 320,
          "pin_region_type_id_minus2": 0,
          "pin_region_unpack_top_left_x": 0,
          "pin_region_unpack_top_left_y": 0,
          "pin_region_width_minus1": 320
        }
      ]
    }
  ]
}
)"sv);

  auto unit = makeMultiplexerWithFakeVideoBitstreamServers(json.require("packingInformation"));
  std::istringstream inStream{createTestBitstream(VpsContent::one_atlas_three_videos)};

  unit.readInputBitstream(inStream);
  unit.addPackingInformation();
  unit.appendVideoSubBitstreams();
  std::ostringstream outStream;
  unit.writeOutputBitstream(outStream);

  std::istringstream result{outStream.str()};
  const auto ssvh =
      checkModifiedVPSWithPackedInformation(result, json.require("packingInformation"));
  requireThatNextV3cUnitContains(result, ssvh, "test V3C_PVD vps:0 atlas:0");
}
