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

#include <catch2/catch_test_macros.hpp>

#include <TMIV/Decoder/PreRenderer.h>

using namespace std::string_view_literals;

namespace test {
auto createUnit() {
  const auto componentNode = TMIV::Common::Json::parse(R"({
        "GeometryScaler": {
            "minForegroundConfidence": 0,
            "geometryEdgeMagnitudeTh": 0,
            "maxCurvature": 0
        }
})"sv);

  return TMIV::Decoder::PreRenderer{componentNode};
}
} // namespace test

TEST_CASE("Decoder::PreRenderer") {
  auto frame = TMIV::MivBitstream::AccessUnit{};
  frame.atlas.emplace_back();

  SECTION("Construction") { REQUIRE_NOTHROW(test::createUnit()); }

  SECTION("Minimal operation") {
    const auto unit = test::createUnit();

    unit.preRenderFrame(frame);
  }

  SECTION("Check that the coordinate system is OMAF") {
    const auto unit = test::createUnit();

    frame.vui = TMIV::MivBitstream::VuiParameters{};
    frame.vui->coordinate_system_parameters().csp_left_sign(false);

    REQUIRE_THROWS(unit.preRenderFrame(frame));
  }

  SECTION("Decoded occupancy frame, in nominal format") {
    const auto unit = test::createUnit();

    frame.vps.vps_frame_width({}, 20)
        .vps_frame_height({}, 10)
        .vps_occupancy_video_present_flag({}, true)
        .occupancy_information({})
        .oi_occupancy_2d_bit_depth_minus1(6)
        .oi_lossy_occupancy_compression_threshold(100);

    auto &atlas = frame.atlas.back();

    atlas.asps.asps_frame_width(20).asps_frame_height(10).asps_log2_patch_packing_block_size(1);

    atlas.blockToPatchMap.createY({10, 5});

    atlas.decOccFrame.createY({20, 10}, 7);

    for (int32_t i = 0; i < 10; ++i) {
      for (int32_t j = 0; j < 20; ++j) {
        atlas.decOccFrame.getPlane(0)(i, j) = static_cast<uint8_t>(i + 4 * j);
      }
    }

    CHECK(std::accumulate(atlas.decOccFrame.getPlane(0).cbegin(),
                          atlas.decOccFrame.getPlane(0).cend(), uint32_t{}) == 8500);

    unit.preRenderFrame(frame);

    REQUIRE(atlas.occFrameNF.getWidth() == 20);
    REQUIRE(atlas.occFrameNF.getHeight() == 10);
    REQUIRE(atlas.occFrameNF.getBitDepth() == 1);
    REQUIRE(atlas.occFrameNF.getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.occFrameNF.getPlane(0).cbegin(),
                          atlas.occFrameNF.getPlane(0).cend(), uint32_t{}) == 80);

    REQUIRE(atlas.occFrame.getWidth() == 20);
    REQUIRE(atlas.occFrame.getHeight() == 10);
    REQUIRE(atlas.occFrame.getBitDepth() == 1);
    REQUIRE(atlas.occFrame.getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.occFrame.getPlane(0).cbegin(), atlas.occFrame.getPlane(0).cend(),
                          uint32_t{}) == 80);
  }

  SECTION("Decoded geometry frame, in nominal format") {
    const auto unit = test::createUnit();

    frame.vps.vps_frame_width({}, 20)
        .vps_frame_height({}, 10)
        .vps_geometry_video_present_flag({}, true)
        .geometry_information({})
        .gi_geometry_2d_bit_depth_minus1(6);

    auto &atlas = frame.atlas.back();

    atlas.asps.asps_frame_width(20).asps_frame_height(10).asps_log2_patch_packing_block_size(1);

    atlas.blockToPatchMap.createY({10, 5});

    atlas.decGeoFrame.createY({20, 10}, 7);

    for (int32_t i = 0; i < 10; ++i) {
      for (int32_t j = 0; j < 20; ++j) {
        atlas.decGeoFrame.getPlane(0)(i, j) = static_cast<uint8_t>(i + 4 * j);
      }
    }

    CHECK(std::accumulate(atlas.decGeoFrame.getPlane(0).cbegin(),
                          atlas.decGeoFrame.getPlane(0).cend(), uint32_t{}) == 8500);

    unit.preRenderFrame(frame);

    REQUIRE(atlas.geoFrameNF.getWidth() == 20);
    REQUIRE(atlas.geoFrameNF.getHeight() == 10);
    REQUIRE(atlas.geoFrameNF.getBitDepth() == 7);
    REQUIRE(atlas.geoFrameNF.getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.geoFrameNF.getPlane(0).cbegin(),
                          atlas.geoFrameNF.getPlane(0).cend(), uint32_t{}) == 8500);

    REQUIRE(atlas.geoFrame.getWidth() == 20);
    REQUIRE(atlas.geoFrame.getHeight() == 10);
    REQUIRE(atlas.geoFrame.getBitDepth() == 7);
    REQUIRE(atlas.geoFrame.getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.geoFrame.getPlane(0).cbegin(), atlas.geoFrame.getPlane(0).cend(),
                          uint32_t{}) == 8500);
  }

  SECTION("Decoded material ID and texture frame, in nominal format except for YUV 4:2:0 texture") {
    const auto unit = test::createUnit();

    frame.vps.vps_frame_width({}, 20)
        .vps_frame_height({}, 10)
        .vps_attribute_video_present_flag({}, true)
        .attribute_information({})
        .ai_attribute_count(2)
        .ai_attribute_2d_bit_depth_minus1(0, 3)
        .ai_attribute_dimension_minus1(0, 0)
        .ai_attribute_type_id(0, TMIV::MivBitstream::AiAttributeTypeId::ATTR_MATERIAL_ID)
        .ai_attribute_2d_bit_depth_minus1(1, 6)
        .ai_attribute_dimension_minus1(1, 2)
        .ai_attribute_type_id(1, TMIV::MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);

    auto &atlas = frame.atlas.back();

    atlas.asps.asps_frame_width(20).asps_frame_height(10).asps_log2_patch_packing_block_size(1);

    atlas.blockToPatchMap.createY({10, 5});

    atlas.decAttrFrame.emplace_back().createY({20, 10}, 4);

    for (int32_t i = 0; i < 10; ++i) {
      for (int32_t j = 0; j < 20; ++j) {
        atlas.decAttrFrame.back().getPlane(0)(i, j) = static_cast<uint8_t>(i);
      }
    }

    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(0).cbegin(),
                          atlas.decAttrFrame.back().getPlane(0).cend(), uint32_t{}) == 900);

    atlas.decAttrFrame.emplace_back().createYuv420({20, 10}, 7);

    for (int32_t i = 0; i < 10; ++i) {
      for (int32_t j = 0; j < 20; ++j) {
        atlas.decAttrFrame.back().getPlane(0)(i, j) = static_cast<uint8_t>(i + 4 * j);
        atlas.decAttrFrame.back().getPlane(1)(i / 2, j / 2) = static_cast<uint8_t>(i + 4 * j + 1);
        atlas.decAttrFrame.back().getPlane(2)(i / 2, j / 2) = static_cast<uint8_t>(i + 4 * j + 2);
      }
    }

    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(0).cbegin(),
                          atlas.decAttrFrame.back().getPlane(0).cend(), uint32_t{}) == 8500);
    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(1).cbegin(),
                          atlas.decAttrFrame.back().getPlane(1).cend(), uint32_t{}) == 2300);
    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(2).cbegin(),
                          atlas.decAttrFrame.back().getPlane(2).cend(), uint32_t{}) == 2350);

    unit.preRenderFrame(frame);

    REQUIRE(atlas.attrFrameNF.size() == 2);

    CHECK(atlas.attrFrameNF[0].getWidth() == 20);
    CHECK(atlas.attrFrameNF[0].getHeight() == 10);
    CHECK(atlas.attrFrameNF[0].getBitDepth() == 4);
    CHECK(atlas.attrFrameNF[0].getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(0).cbegin(),
                          atlas.attrFrameNF[0].getPlane(0).cend(), uint32_t{}) == 900);

    CHECK(atlas.attrFrameNF[1].getWidth() == 20);
    CHECK(atlas.attrFrameNF[1].getHeight() == 10);
    CHECK(atlas.attrFrameNF[1].getBitDepth() == 7);
    CHECK(atlas.attrFrameNF[1].getColorFormat() == TMIV::Common::ColorFormat::YUV444);

    CHECK(std::accumulate(atlas.attrFrameNF[1].getPlane(0).cbegin(),
                          atlas.attrFrameNF[1].getPlane(0).cend(), uint32_t{}) == 8500);
    CHECK(std::accumulate(atlas.attrFrameNF[1].getPlane(1).cbegin(),
                          atlas.attrFrameNF[1].getPlane(1).cend(), uint32_t{}) == 9200);
    CHECK(std::accumulate(atlas.attrFrameNF[1].getPlane(2).cbegin(),
                          atlas.attrFrameNF[1].getPlane(2).cend(), uint32_t{}) == 9400);

    CHECK(atlas.texFrame.getWidth() == 20);
    CHECK(atlas.texFrame.getHeight() == 10);
    CHECK(atlas.texFrame.getBitDepth() == 7);
    CHECK(atlas.texFrame.getColorFormat() == TMIV::Common::ColorFormat::YUV444);

    CHECK(std::accumulate(atlas.texFrame.getPlane(0).cbegin(), atlas.texFrame.getPlane(0).cend(),
                          uint32_t{}) == 8500);
    CHECK(std::accumulate(atlas.texFrame.getPlane(1).cbegin(), atlas.texFrame.getPlane(1).cend(),
                          uint32_t{}) == 9200);
    CHECK(std::accumulate(atlas.texFrame.getPlane(2).cbegin(), atlas.texFrame.getPlane(2).cend(),
                          uint32_t{}) == 9400);
  }

  SECTION("Packed video data with occupancy and geometry, plus attribute video data with texture; "
          "all at non-nominal formats") {
    const auto unit = test::createUnit();

    const auto pin = TMIV::MivBitstream::PackingInformation{}
                         .pin_occupancy_2d_bit_depth_minus1(7)
                         .pin_occupancy_MSB_align_flag(true)
                         .pin_lossy_occupancy_compression_threshold(50)
                         .pin_geometry_2d_bit_depth_minus1(8)
                         .pin_geometry_MSB_align_flag(false)
                         .pin_regions_count_minus1(1) // [0, 0] - [20, 10]
                         .pinRegionTypeId(0, TMIV::MivBitstream::VuhUnitType::V3C_OVD)
                         .pin_region_map_index(0, 0)
                         .pin_region_auxiliary_data_flag(0, false)
                         .pin_region_width_minus1(0, 14) // [0, 0] - [15, 10]
                         .pin_region_height_minus1(0, 9)
                         .pin_region_top_left_x(0, 0)
                         .pin_region_top_left_y(0, 0)
                         .pin_region_unpack_top_left_x(0, 0)
                         .pin_region_unpack_top_left_y(0, 0)
                         .pin_region_rotation_flag(0, false)
                         .pinRegionTypeId(1, TMIV::MivBitstream::VuhUnitType::V3C_GVD)
                         .pin_region_map_index(1, 0)
                         .pin_region_auxiliary_data_flag(1, false)
                         .pin_region_width_minus1(1, 4) // [15, 0] - [20, 10]
                         .pin_region_height_minus1(1, 9)
                         .pin_region_top_left_x(1, 15)
                         .pin_region_top_left_y(1, 0)
                         .pin_region_unpack_top_left_x(1, 0)
                         .pin_region_unpack_top_left_y(1, 0)
                         .pin_region_rotation_flag(1, true);

    REQUIRE_FALSE(pin.pin_attribute_present_flag());

    frame.vps.vps_frame_width({}, 20)
        .vps_frame_height({}, 10)
        .vps_attribute_video_present_flag({}, true)
        .vps_packed_video_present_flag({}, true)
        .packing_information({}, pin)
        .attribute_information({})
        .ai_attribute_count(1)
        .ai_attribute_2d_bit_depth_minus1(0, 6)
        .ai_attribute_dimension_minus1(0, 2)
        .ai_attribute_type_id(0, TMIV::MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);

    auto &atlas = frame.atlas.back();

    atlas.asps.asps_frame_width(20).asps_frame_height(10).asps_log2_patch_packing_block_size(1);

    atlas.blockToPatchMap.createY({10, 5});

    atlas.decAttrFrame.emplace_back().createYuv420({16, 8}, 6);

    for (int32_t i = 0; i < 7; ++i) {
      for (int32_t j = 0; j < 15; ++j) {
        atlas.decAttrFrame.back().getPlane(0)(i, j) = static_cast<uint8_t>(i + 4 * j);
        atlas.decAttrFrame.back().getPlane(1)(i / 2, j / 2) = static_cast<uint8_t>(i + 4 * j + 1);
        atlas.decAttrFrame.back().getPlane(2)(i / 2, j / 2) = static_cast<uint8_t>(i + 4 * j + 2);
      }
    }

    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(0).cbegin(),
                          atlas.decAttrFrame.back().getPlane(0).cend(), uint32_t{}) == 3255);
    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(1).cbegin(),
                          atlas.decAttrFrame.back().getPlane(1).cend(), uint32_t{}) == 1160);
    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(2).cbegin(),
                          atlas.decAttrFrame.back().getPlane(2).cend(), uint32_t{}) == 1192);

    atlas.decPckFrame.createYuv420({20, 10}, 4);

    for (int32_t i = 0; i < 10; ++i) {
      for (int32_t j = 0; j < 20; ++j) {
        atlas.decPckFrame.getPlane(0)(i, j) = static_cast<uint8_t>(i);
        atlas.decPckFrame.getPlane(1)(i / 2, j / 2) = static_cast<uint8_t>(j / 2);
        atlas.decPckFrame.getPlane(2)(i / 2, j / 2) = static_cast<uint8_t>(j);
      }
    }

    unit.preRenderFrame(frame);

    REQUIRE(atlas.attrFrameNF.size() == 1);

    CHECK(atlas.attrFrameNF[0].getWidth() == 20);
    CHECK(atlas.attrFrameNF[0].getHeight() == 10);
    CHECK(atlas.attrFrameNF[0].getBitDepth() == 7);
    CHECK(atlas.attrFrameNF[0].getColorFormat() == TMIV::Common::ColorFormat::YUV444);

    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(0).cbegin(),
                          atlas.attrFrameNF[0].getPlane(0).cend(), uint32_t{}) == 5119);
    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(1).cbegin(),
                          atlas.attrFrameNF[0].getPlane(1).cend(), uint32_t{}) == 7080);
    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(2).cbegin(),
                          atlas.attrFrameNF[0].getPlane(2).cend(), uint32_t{}) == 7280);

    CHECK(atlas.texFrame.getWidth() == 20);
    CHECK(atlas.texFrame.getHeight() == 10);
    CHECK(atlas.texFrame.getBitDepth() == 7);
    CHECK(atlas.texFrame.getColorFormat() == TMIV::Common::ColorFormat::YUV444);

    CHECK(std::accumulate(atlas.texFrame.getPlane(0).cbegin(), atlas.texFrame.getPlane(0).cend(),
                          uint32_t{}) == 5119);
    CHECK(std::accumulate(atlas.texFrame.getPlane(1).cbegin(), atlas.texFrame.getPlane(1).cend(),
                          uint32_t{}) == 7080);
    CHECK(std::accumulate(atlas.texFrame.getPlane(2).cbegin(), atlas.texFrame.getPlane(2).cend(),
                          uint32_t{}) == 7280);
  }

  SECTION("Texture and 4 x 2 downscaled geometry") {
    const auto unit = test::createUnit();

    frame.vps.vps_frame_width({}, 20)
        .vps_frame_height({}, 10)
        .vps_geometry_video_present_flag({}, true)
        .geometry_information({})
        .gi_geometry_2d_bit_depth_minus1(6);
    frame.vps.vps_attribute_video_present_flag({}, true)
        .attribute_information({})
        .ai_attribute_count(1)
        .ai_attribute_2d_bit_depth_minus1(0, 6)
        .ai_attribute_dimension_minus1(0, 2)
        .ai_attribute_type_id(0, TMIV::MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);
    frame.vps.vps_miv_extension().vme_geometry_scale_enabled_flag(true);

    auto &atlas = frame.atlas.back();

    atlas.asps.asps_frame_width(20)
        .asps_frame_height(10)
        .asps_log2_patch_packing_block_size(1)
        .asps_miv_extension()
        .asme_geometry_scale_enabled_flag(true)
        .asme_geometry_scale_factor_x_minus1(3)
        .asme_geometry_scale_factor_y_minus1(1);

    atlas.blockToPatchMap.createY({10, 5});

    atlas.decGeoFrame.createY({5, 5}, 7);

    for (int32_t i = 0; i < 5; ++i) {
      for (int32_t j = 0; j < 5; ++j) {
        atlas.decGeoFrame.getPlane(0)(i, j) = static_cast<uint8_t>(i + 4 * j);
      }
    }

    CHECK(std::accumulate(atlas.decGeoFrame.getPlane(0).cbegin(),
                          atlas.decGeoFrame.getPlane(0).cend(), uint32_t{}) == 250);

    atlas.decAttrFrame.emplace_back().createYuv420({16, 8}, 6);

    for (int32_t i = 0; i < 7; ++i) {
      for (int32_t j = 0; j < 15; ++j) {
        atlas.decAttrFrame.back().getPlane(0)(i, j) = static_cast<uint8_t>(i + 4 * j);
        atlas.decAttrFrame.back().getPlane(1)(i / 2, j / 2) = static_cast<uint8_t>(i + 4 * j + 1);
        atlas.decAttrFrame.back().getPlane(2)(i / 2, j / 2) = static_cast<uint8_t>(i + 4 * j + 2);
      }
    }

    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(0).cbegin(),
                          atlas.decAttrFrame.back().getPlane(0).cend(), uint32_t{}) == 3255);
    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(1).cbegin(),
                          atlas.decAttrFrame.back().getPlane(1).cend(), uint32_t{}) == 1160);
    CHECK(std::accumulate(atlas.decAttrFrame.back().getPlane(2).cbegin(),
                          atlas.decAttrFrame.back().getPlane(2).cend(), uint32_t{}) == 1192);

    unit.preRenderFrame(frame);

    REQUIRE(atlas.geoFrameNF.getWidth() == 5);
    REQUIRE(atlas.geoFrameNF.getHeight() == 5);
    REQUIRE(atlas.geoFrameNF.getBitDepth() == 7);
    REQUIRE(atlas.geoFrameNF.getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.geoFrameNF.getPlane(0).cbegin(),
                          atlas.geoFrameNF.getPlane(0).cend(), uint32_t{}) == 250);

    REQUIRE(atlas.geoFrame.getWidth() == 20);
    REQUIRE(atlas.geoFrame.getHeight() == 10);
    REQUIRE(atlas.geoFrame.getBitDepth() == 7);
    REQUIRE(atlas.geoFrame.getColorFormat() == TMIV::Common::ColorFormat::YUV400);

    CHECK(std::accumulate(atlas.geoFrame.getPlane(0).cbegin(), atlas.geoFrame.getPlane(0).cend(),
                          uint32_t{}) == 1960);

    REQUIRE(atlas.attrFrameNF.size() == 1);

    CHECK(atlas.attrFrameNF[0].getWidth() == 20);
    CHECK(atlas.attrFrameNF[0].getHeight() == 10);
    CHECK(atlas.attrFrameNF[0].getBitDepth() == 7);
    CHECK(atlas.attrFrameNF[0].getColorFormat() == TMIV::Common::ColorFormat::YUV444);

    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(0).cbegin(),
                          atlas.attrFrameNF[0].getPlane(0).cend(), uint32_t{}) == 5119);
    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(1).cbegin(),
                          atlas.attrFrameNF[0].getPlane(1).cend(), uint32_t{}) == 7080);
    CHECK(std::accumulate(atlas.attrFrameNF[0].getPlane(2).cbegin(),
                          atlas.attrFrameNF[0].getPlane(2).cend(), uint32_t{}) == 7280);

    CHECK(atlas.texFrame.getWidth() == 20);
    CHECK(atlas.texFrame.getHeight() == 10);
    CHECK(atlas.texFrame.getBitDepth() == 7);
    CHECK(atlas.texFrame.getColorFormat() == TMIV::Common::ColorFormat::YUV444);

    CHECK(std::accumulate(atlas.texFrame.getPlane(0).cbegin(), atlas.texFrame.getPlane(0).cend(),
                          uint32_t{}) == 5119);
    CHECK(std::accumulate(atlas.texFrame.getPlane(1).cbegin(), atlas.texFrame.getPlane(1).cend(),
                          uint32_t{}) == 7080);
    CHECK(std::accumulate(atlas.texFrame.getPlane(2).cbegin(), atlas.texFrame.getPlane(2).cend(),
                          uint32_t{}) == 7280);
  }
}
