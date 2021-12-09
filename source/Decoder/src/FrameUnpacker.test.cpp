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

#include <TMIV/Common/Common.h>
#include <TMIV/Decoder/FrameUnpacker.h>

using namespace std::string_view_literals;

TEST_CASE("TMIV::Decoder::FrameUnpacker: 1 Atlas with texture and geometry") {
  using Catch::Contains;

  TMIV::Decoder::FrameUnpacker unit{};
  auto frame = TMIV::MivBitstream::AccessUnit{};

  frame.atlas.resize(1);
  const auto atlasId = TMIV::MivBitstream::AtlasId{0};

  auto &vps = frame.vps;
  vps.vps_map_count_minus1(atlasId, 0).vps_packed_video_present_flag(atlasId, true);

  TMIV::MivBitstream::PackingInformation packingInformation = {};
  packingInformation.pin_geometry_2d_bit_depth_minus1(9)
      .pin_geometry_MSB_align_flag(false)
      .pin_geometry_3d_coordinates_bit_depth_minus1(9)
      .pin_attribute_count(1)
      .pin_attribute_type_id(0, TMIV::MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)
      .pin_attribute_2d_bit_depth_minus1(0, 9)
      .pin_attribute_MSB_align_flag(0, true)
      .pin_attribute_map_absolute_coding_persistence_flag(0, false)
      .pin_attribute_dimension_minus1(0, 0)
      .pin_attribute_dimension_partitions_minus1(0, 0);

  SECTION("Full-Scale Geoemtry") {
    frame.atlas[0].decPacFrame = TMIV::Common::FramePack444Frame::yuv444({32, 16}, 10);
    packingInformation.pin_regions_count_minus1(1)
        .pin_region_tile_id(0, static_cast<uint8_t>(0))
        .pin_region_type_id_minus2(0, 2)
        .pin_region_top_left_x(0, 0)
        .pin_region_top_left_y(0, 0)
        .pin_region_width_minus1(0, 31)
        .pin_region_height_minus1(0, 7)
        .pin_region_unpack_top_left_x(0, 0)
        .pin_region_unpack_top_left_y(0, 0)
        .pin_region_rotation_flag(0, false)
        .pin_region_map_index(0, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(0, false)
        .pin_region_attr_index(0, static_cast<uint8_t>(0))
        .pin_region_tile_id(1, static_cast<uint8_t>(0))
        .pin_region_type_id_minus2(1, 1)
        .pin_region_top_left_x(1, 0)
        .pin_region_top_left_y(1, 8)
        .pin_region_width_minus1(1, 31)
        .pin_region_height_minus1(1, 7)
        .pin_region_unpack_top_left_x(1, 0)
        .pin_region_unpack_top_left_y(1, 0)
        .pin_region_rotation_flag(1, false)
        .pin_region_map_index(1, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(1, false);
    vps.packing_information(atlasId, packingInformation);

    unit.inplaceUnpack(frame);
    CHECK(frame.atlas[0].attrFrame.getWidth() == 32);
    CHECK(frame.atlas[0].attrFrame.getHeight() == 8);
    CHECK(frame.atlas[0].decGeoFrame.getWidth() == 32);
    CHECK(frame.atlas[0].decGeoFrame.getHeight() == 8);
  }

  SECTION("Downscaled Geometry [2, 2]") {
    frame.atlas[0].decPacFrame = TMIV::Common::FramePack444Frame::yuv444({8, 30}, 10);
    packingInformation.pin_regions_count_minus1(2)
        .pin_region_tile_id(0, static_cast<uint8_t>(0))
        .pin_region_type_id_minus2(0, 2)
        .pin_region_top_left_x(0, 0)
        .pin_region_top_left_y(0, 0)
        .pin_region_width_minus1(0, 7)
        .pin_region_height_minus1(0, 23)
        .pin_region_unpack_top_left_x(0, 0)
        .pin_region_unpack_top_left_y(0, 0)
        .pin_region_rotation_flag(0, false)
        .pin_region_map_index(0, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(0, false)
        .pin_region_attr_index(0, static_cast<uint8_t>(0))
        .pin_region_tile_id(1, static_cast<uint8_t>(0))
        .pin_region_type_id_minus2(1, 1)
        .pin_region_top_left_x(1, 0)
        .pin_region_top_left_y(1, 24)
        .pin_region_width_minus1(1, 3)
        .pin_region_height_minus1(1, 5)
        .pin_region_unpack_top_left_x(1, 0)
        .pin_region_unpack_top_left_y(1, 0)
        .pin_region_rotation_flag(1, false)
        .pin_region_map_index(1, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(1, false)
        .pin_region_tile_id(2, static_cast<uint8_t>(0))
        .pin_region_type_id_minus2(2, 1)
        .pin_region_top_left_x(2, 4)
        .pin_region_top_left_y(2, 24)
        .pin_region_width_minus1(2, 3)
        .pin_region_height_minus1(2, 5)
        .pin_region_unpack_top_left_x(2, 0)
        .pin_region_unpack_top_left_y(2, 6)
        .pin_region_rotation_flag(2, false)
        .pin_region_map_index(2, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(2, false);
    vps.packing_information(atlasId, packingInformation);

    unit.inplaceUnpack(frame);
    CHECK(frame.atlas[0].attrFrame.getWidth() == 8);
    CHECK(frame.atlas[0].attrFrame.getHeight() == 24);
    CHECK(frame.atlas[0].decGeoFrame.getWidth() == 4);
    CHECK(frame.atlas[0].decGeoFrame.getHeight() == 12);
  }
}
