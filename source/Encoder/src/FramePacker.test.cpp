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

#include <TMIV/Common/Common.h>
#include <TMIV/Encoder/Encoder.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <catch2/catch.hpp>

using namespace std::string_view_literals;
using Catch::Contains;
using TMIV::Common::Json;

TEST_CASE("TMIV::Encoder::FramePacker, 1 Atlas with texture and geometry") {
  auto params = TMIV::Encoder::EncoderParams{};
  params.vps.vps_atlas_count_minus1(0);
  const auto atlasId = params.vps.vps_atlas_id(0);
  params.vps.vps_geometry_video_present_flag(atlasId, true)
      .vps_attribute_video_present_flag(atlasId, true)
      .vps_miv_extension_present_flag(true)
      .vps_frame_width(atlasId, 32)
      .vps_frame_height(atlasId, 64)
      .attribute_information(atlasId)
      .ai_attribute_count(1);
  params.atlas.push_back(TMIV::Encoder::EncoderAtlasParams{});
  auto &asmeAtlas = params.atlas[0].asps.asps_miv_extension();

  auto frame = TMIV::Common::V3cFrameList{};
  auto atlas = TMIV::Common::V3cFrame{};

  SECTION("Full-Scale Geoemtry") {
    asmeAtlas.asme_geometry_scale_enabled_flag(false);
    atlas.texture.createYuv420({32, 64}, 10);
    atlas.geometry.createY({32, 64}, 10);
    frame.push_back(atlas);

    TMIV::Encoder::FramePacker unit{};
    const auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 1);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    unit.packFrame(frame, 10);
    CHECK(frame[0].packed.getWidth() == 32);
    CHECK(frame[0].packed.getHeight() == 128);
  }

  SECTION("Downscaled Geometry [2, 2]") {
    asmeAtlas.asme_geometry_scale_factor_x_minus1(1).asme_geometry_scale_factor_y_minus1(1);
    atlas.texture.createYuv420({32, 64}, 10);
    atlas.geometry.createY({16, 32}, 10);
    frame.push_back(atlas);

    TMIV::Encoder::FramePacker unit{};
    auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 2);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 16);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 16);
    unit.packFrame(frame, 10);
    CHECK(frame[0].packed.getWidth() == 32);
    CHECK(frame[0].packed.getHeight() == 80);
  }

  SECTION("Downscaled Geometry [4, 1]") {
    asmeAtlas.asme_geometry_scale_factor_x_minus1(3).asme_geometry_scale_factor_y_minus1(0);
    atlas.texture.createYuv420({32, 64}, 10);
    atlas.geometry.createY({8, 64}, 10);
    frame.push_back(atlas);

    TMIV::Encoder::FramePacker unit{};
    const auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 4);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 7);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 8);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 7);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 16);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(3) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(3) == 16);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(3) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(3) == 7);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(3) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(3) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(3) == 32);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(4) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(4) == 24);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(4) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(4) == 7);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(4) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(4) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(4) == 48);
    unit.packFrame(frame, 10);
    CHECK(frame[0].packed.getWidth() == 32);
    CHECK(frame[0].packed.getHeight() == 80);
  }
}

TEST_CASE("TMIV::Encoder::FramePacker, 1 Atlas with texture, geometry, and occupancy") {
  auto params = TMIV::Encoder::EncoderParams{};
  params.vps.vps_atlas_count_minus1(0);
  TMIV::MivBitstream::AtlasId atlasId = params.vps.vps_atlas_id(0);
  params.vps.vps_occupancy_video_present_flag(atlasId, true)
      .vps_geometry_video_present_flag(atlasId, true)
      .vps_attribute_video_present_flag(atlasId, true)
      .vps_miv_extension_present_flag(true)
      .vps_frame_width(atlasId, 32)
      .vps_frame_height(atlasId, 64)
      .attribute_information(atlasId)
      .ai_attribute_count(1);

  params.atlas.push_back(TMIV::Encoder::EncoderAtlasParams{});
  auto &asmeAtlas = params.atlas[0].asps.asps_miv_extension();

  auto frame = TMIV::Common::V3cFrameList{};
  auto atlas = TMIV::Common::V3cFrame{};

  SECTION("Full-Scale Geoemtry & Full-Scale Occupancy") {
    asmeAtlas.asme_geometry_scale_enabled_flag(false).asme_occupancy_scale_enabled_flag(false);
    atlas.texture.createYuv420({32, 64}, 10);
    atlas.geometry.createY({32, 64}, 10);
    atlas.occupancy.createY({32, 64}, 10);
    frame.push_back(atlas);

    TMIV::Encoder::FramePacker unit{};
    const auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_occupancy_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 2);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 0); // V3C_OVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 128);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 0);
  }
}

TEST_CASE("TMIV::Encoder::FramePacker, 1 Atlas with attribute and occupancy only") {
  auto params = TMIV::Encoder::EncoderParams{};
  params.vps.vps_atlas_count_minus1(0);
  TMIV::MivBitstream::AtlasId atlasId = params.vps.vps_atlas_id(0);
  params.vps.vps_occupancy_video_present_flag(atlasId, true)
      .vps_geometry_video_present_flag(atlasId, false)
      .vps_attribute_video_present_flag(atlasId, true)
      .vps_miv_extension_present_flag(true)
      .vps_frame_width(atlasId, 32)
      .vps_frame_height(atlasId, 64)
      .attribute_information(atlasId)
      .ai_attribute_count(1);

  params.atlas.push_back(TMIV::Encoder::EncoderAtlasParams{});
  auto &asmeAtlas = params.atlas[0].asps.asps_miv_extension();

  auto frame = TMIV::Common::V3cFrameList{};
  auto atlas = TMIV::Common::V3cFrame{};

  SECTION("Downscaled Occupancy [2 2]") {
    asmeAtlas.asme_occupancy_scale_factor_x_minus1(1).asme_occupancy_scale_factor_y_minus1(1);
    atlas.texture.createYuv420({32, 64}, 10);
    atlas.occupancy.createY({16, 32}, 10);
    frame.push_back(atlas);

    TMIV::Encoder::FramePacker unit{};
    const auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_occupancy_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 2);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 31);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 63);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 0); // V3C_OVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 0); // V3C_OVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 16);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 64);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 15);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 16);
  }
}
