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

TEST_CASE("TMIV::Encoder::FramePack, 1 Atlas with texture and geometry") {
  auto params = TMIV::Encoder::EncoderParams{};
  params.vps.vps_atlas_count_minus1(0);
  const auto atlasId = params.vps.vps_atlas_id(0);
  params.vps.vps_geometry_video_present_flag(atlasId, true)
      .vps_attribute_video_present_flag(atlasId, true)
      .vps_extension_present_flag(true)
      .vps_miv_extension_present_flag(true)
      .vps_frame_width(atlasId, 1920)
      .vps_frame_height(atlasId, 4640)
      .attribute_information(atlasId)
      .ai_attribute_count(1);
  params.atlas.push_back(TMIV::Encoder::EncoderAtlasParams{});
  params.atlas[0].asps.asps_extension_present_flag(true).asps_miv_extension_present_flag(true);
  auto &asmeAtlas = params.atlas[0].asps.asps_miv_extension();

  auto frame = TMIV::Common::MVD10Frame{};
  auto atlas = TMIV::Common::TextureDepth10Frame{};

  SECTION("Full-Scale Geoemtry") {
    asmeAtlas.asme_geometry_scale_enabled_flag(false);
    atlas.texture.resize(1920, 4640);
    atlas.depth.resize(1920, 4640);
    frame.push_back(atlas);

    TMIV::Encoder::FramePack unit{};
    auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 1);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    unit.constructFramePack(frame);
    CHECK(frame[0].framePack.getWidth() == 1920);
    CHECK(frame[0].framePack.getHeight() == 9280);
  }

  SECTION("Downscaled Geometry [2, 2]") {
    asmeAtlas.asme_geometry_scale_enabled_flag(true)
        .asme_geometry_scale_factor_x_minus1(1)
        .asme_geometry_scale_factor_y_minus1(1);
    atlas.texture.resize(1920, 4640);
    atlas.depth.resize(960, 2320);
    frame.push_back(atlas);

    TMIV::Encoder::FramePack unit{};
    auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 2);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 959);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 1159);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 960);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 959);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 1159);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 1160);
    unit.constructFramePack(frame);
    CHECK(frame[0].framePack.getWidth() == 1920);
    CHECK(frame[0].framePack.getHeight() == 5800);
  }

  SECTION("Downscaled Geometry [4, 1]") {
    asmeAtlas.asme_geometry_scale_enabled_flag(true)
        .asme_geometry_scale_factor_x_minus1(3)
        .asme_geometry_scale_factor_y_minus1(0);
    atlas.texture.resize(1920, 4640);
    atlas.depth.resize(480, 4640);
    frame.push_back(atlas);

    TMIV::Encoder::FramePack unit{};
    auto outParams = unit.setPackingInformation(params);
    CHECK(outParams.vps.vps_attribute_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_geometry_video_present_flag(atlasId) == false);
    CHECK(outParams.vps.vps_packed_video_present_flag(atlasId) == true);
    CHECK(outParams.vps.vps_packing_information_present_flag() == true);
    CHECK(outParams.vps.packing_information(atlasId).pin_regions_count_minus1() == 4);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(0) == 2); // V3C_AVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 479);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 1159);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 480);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 479);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 1159);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 1160);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(3) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(3) == 960);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(3) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(3) == 479);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(3) == 1159);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(3) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(3) == 2320);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(4) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(4) == 1440);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(4) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(4) == 479);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(4) == 1159);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(4) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(4) == 3480);
    unit.constructFramePack(frame);
    CHECK(frame[0].framePack.getWidth() == 1920);
    CHECK(frame[0].framePack.getHeight() == 5800);
  }
}

TEST_CASE("TMIV::Encoder::FramePack, 1 Atlas with texture, geometry, and occupancy") {
  auto params = TMIV::Encoder::EncoderParams{};
  params.vps.vps_atlas_count_minus1(0);
  TMIV::MivBitstream::AtlasId atlasId = params.vps.vps_atlas_id(0);
  params.vps.vps_occupancy_video_present_flag(atlasId, true)
      .vps_geometry_video_present_flag(atlasId, true)
      .vps_attribute_video_present_flag(atlasId, true)
      .vps_extension_present_flag(true)
      .vps_miv_extension_present_flag(true)
      .vps_frame_width(atlasId, 1920)
      .vps_frame_height(atlasId, 4640)
      .attribute_information(atlasId)
      .ai_attribute_count(1);

  params.atlas.push_back(TMIV::Encoder::EncoderAtlasParams{});
  params.atlas[0].asps.asps_extension_present_flag(true).asps_miv_extension_present_flag(true);
  auto &asmeAtlas = params.atlas[0].asps.asps_miv_extension();

  auto frame = TMIV::Common::MVD10Frame{};
  auto atlas = TMIV::Common::TextureDepth10Frame{};

  SECTION("Full-Scale Geoemtry & Full-Scale Occupancy") {
    asmeAtlas.asme_geometry_scale_enabled_flag(false).asme_occupancy_scale_enabled_flag(false);
    atlas.texture.resize(1920, 4640);
    atlas.depth.resize(1920, 4640);
    atlas.occupancy.resize(1920, 4640);
    frame.push_back(atlas);

    TMIV::Encoder::FramePack unit{};
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
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(0) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(0) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(0) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(1) == 1); // V3C_GVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(1) == 4640);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(1) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(1) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(1) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_type_id_minus2(2) == 0); // V3C_OVD
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_top_left_y(2) == 9280);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_width_minus1(2) == 1919);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_height_minus1(2) == 4639);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_x(2) == 0);
    CHECK(outParams.vps.packing_information(atlasId).pin_region_unpack_top_left_y(2) == 0);
    // unit.constructFramePack(frame); // TODO(Basel Salahieh): Add occupancy support
    // CHECK(frame[0].framePack.getWidth() == 1920);
    // CHECK(frame[0].framePack.getHeight() == 13920);
  }
}
