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

#include "test.h"

#include <TMIV/MivBitstream/V3cParameterSet.h>

using namespace std::string_literals;

namespace TMIV::MivBitstream {
TEST_CASE("profile_tier_level", "[V3C Parameter Set]") {
  auto x = ProfileTierLevel{};

  REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_toolset_constraints_present_flag=false
)");

  CHECK(x.profile() == "AVC Progressive High V-PCC Basic Rec0");

  bitCodingTest(x, 72);

  SECTION("Example 1") {
    x.ptl_tier_flag(true)
        .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main)
        .ptl_profile_toolset_idc(PtlProfileToolsetIdc::VPCC_Extended)
        .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained)
        .ptl_max_decodes_idc(PtlMaxDecodesIdc::max_4)
        .ptl_level_idc(PtlLevelIdc::Level_2_0)
        .ptl_num_sub_profiles(2)
        .ptl_extended_sub_profile_flag(true)
        .ptl_sub_profile_idc(0, 3)
        .ptl_sub_profile_idc(1, UINT64_MAX)
        .ptl_toolset_constraints_present_flag(false);

    REQUIRE(toString(x) == R"(ptl_tier_flag=true
ptl_profile_codec_group_idc=HEVC Main
ptl_profile_toolset_idc=V-PCC Extended
ptl_profile_reconstruction_idc=Rec Unconstrained
ptl_max_decodes_idc=max_4
ptl_level_idc=2.0
ptl_num_sub_profiles=2
ptl_extended_sub_profile_flag=true
ptl_sub_profile_idc[ 0 ]=3
ptl_sub_profile_idc[ 1 ]=18446744073709551615
ptl_toolset_constraints_present_flag=false
)");

    CHECK(x.profile() == "HEVC Main V-PCC Extended");

    bitCodingTest(x, 200);
  }

  SECTION("Example 2") {
    auto ptci = ProfileToolsetConstraintsInformation{}.ptc_one_v3c_frame_only_flag(true);

    x.ptl_tier_flag(true)
        .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
        .ptl_profile_toolset_idc(PtlProfileToolsetIdc::VPCC_Extended)
        .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained)
        .ptl_max_decodes_idc(PtlMaxDecodesIdc::max_4)
        .ptl_level_idc(PtlLevelIdc::Level_2_0)
        .ptl_num_sub_profiles(2)
        .ptl_extended_sub_profile_flag(true)
        .ptl_sub_profile_idc(0, 3)
        .ptl_sub_profile_idc(1, UINT64_MAX)
        .ptl_profile_toolset_constraints_information(ptci);

    REQUIRE(toString(x) == R"(ptl_tier_flag=true
ptl_profile_codec_group_idc=HEVC Main10
ptl_profile_toolset_idc=V-PCC Extended
ptl_profile_reconstruction_idc=Rec Unconstrained
ptl_max_decodes_idc=max_4
ptl_level_idc=2.0
ptl_num_sub_profiles=2
ptl_extended_sub_profile_flag=true
ptl_sub_profile_idc[ 0 ]=3
ptl_sub_profile_idc[ 1 ]=18446744073709551615
ptl_toolset_constraints_present_flag=true
ptc_one_v3c_frame_only_flag=true
ptc_eom_constraint_flag=false
ptc_max_map_count_minus1=15
ptc_max_atlas_count_minus1=15
ptc_multiple_map_streams_constraint_flag=false
ptc_plr_constraint_flag=false
ptc_attribute_max_dimension_minus1=63
ptc_attribute_max_dimension_partitions_minus1=63
ptc_no_eight_orientations_constraint_flag=false
ptc_no_45degree_projection_patch_constraint_flag=false
ptc_restricted_geometry_flag=false
ptc_num_reserved_constraint_bytes=0
)");

    CHECK(x.profile() == "HEVC Main10 V-PCC Extended Still");

    bitCodingTest(x, 240);
  }
}

TEST_CASE("profile_toolset_constraints_information", "[V3C Parameter Set]") {
  auto x = ProfileToolsetConstraintsInformation{};

  REQUIRE(toString(x) == R"(ptc_one_v3c_frame_only_flag=false
ptc_eom_constraint_flag=false
ptc_max_map_count_minus1=15
ptc_max_atlas_count_minus1=15
ptc_multiple_map_streams_constraint_flag=false
ptc_plr_constraint_flag=false
ptc_attribute_max_dimension_minus1=63
ptc_attribute_max_dimension_partitions_minus1=63
ptc_no_eight_orientations_constraint_flag=false
ptc_no_45degree_projection_patch_constraint_flag=false
ptc_restricted_geometry_flag=false
ptc_num_reserved_constraint_bytes=0
)");

  bitCodingTest(x, 40);

  SECTION("Example 1") {
    x.ptc_one_v3c_frame_only_flag(true)
        .ptc_eom_constraint_flag(true)
        .ptc_max_map_count_minus1(5)
        .ptc_max_atlas_count_minus1(2)
        .ptc_multiple_map_streams_constraint_flag(false)
        .ptc_plr_constraint_flag(true)
        .ptc_attribute_max_dimension_minus1(3)
        .ptc_attribute_max_dimension_partitions_minus1(14)
        .ptc_no_eight_orientations_constraint_flag(true)
        .ptc_no_45degree_projection_patch_constraint_flag(false);

    REQUIRE(toString(x) == R"(ptc_one_v3c_frame_only_flag=true
ptc_eom_constraint_flag=true
ptc_max_map_count_minus1=5
ptc_max_atlas_count_minus1=2
ptc_multiple_map_streams_constraint_flag=false
ptc_plr_constraint_flag=true
ptc_attribute_max_dimension_minus1=3
ptc_attribute_max_dimension_partitions_minus1=14
ptc_no_eight_orientations_constraint_flag=true
ptc_no_45degree_projection_patch_constraint_flag=false
ptc_restricted_geometry_flag=false
ptc_num_reserved_constraint_bytes=0
)");

    bitCodingTest(x, 40);
  }
}

TEST_CASE("occupancy_information", "[V3C Parameter Set]") {
  auto x = OccupancyInformation{};

  REQUIRE(toString(x, AtlasId{3}) == R"(oi_occupancy_codec_id[ 3 ]=0
oi_lossy_occupancy_compression_threshold[ 3 ]=0
oi_occupancy_2d_bit_depth_minus1[ 3 ]=0
oi_occupancy_MSB_align_flag[ 3 ]=false
)");

  bitCodingTest(x, 22);

  SECTION("Example") {
    x.oi_occupancy_codec_id(255)
        .oi_lossy_occupancy_compression_threshold(255)
        .oi_occupancy_2d_bit_depth_minus1(31)
        .oi_occupancy_MSB_align_flag(true);

    REQUIRE(toString(x, AtlasId{4}) == R"(oi_occupancy_codec_id[ 4 ]=255
oi_lossy_occupancy_compression_threshold[ 4 ]=255
oi_occupancy_2d_bit_depth_minus1[ 4 ]=31
oi_occupancy_MSB_align_flag[ 4 ]=true
)");

    bitCodingTest(x, 22);
  }
}

TEST_CASE("geometry_information", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};
  const auto atlasId = AtlasId{0};
  vps.vps_auxiliary_video_present_flag(atlasId, false);

  auto x = GeometryInformation{};

  REQUIRE(toString(x, AtlasId{0}) == R"(gi_geometry_codec_id[ 0 ]=0
gi_geometry_2d_bit_depth_minus1[ 0 ]=0
gi_geometry_msb_align_flag[ 0 ]=false
gi_geometry_3d_coordinates_bit_depth_minus1[ 0 ]=0
)");

  bitCodingTest(x, 19, vps, atlasId);

  SECTION("Example") {
    x.gi_geometry_codec_id(255)
        .gi_geometry_2d_bit_depth_minus1(31)
        .gi_geometry_msb_align_flag(true)
        .gi_geometry_3d_coordinates_bit_depth_minus1(31);

    REQUIRE(toString(x, AtlasId{0}) == R"(gi_geometry_codec_id[ 0 ]=255
gi_geometry_2d_bit_depth_minus1[ 0 ]=31
gi_geometry_msb_align_flag[ 0 ]=true
gi_geometry_3d_coordinates_bit_depth_minus1[ 0 ]=31
)");

    bitCodingTest(x, 19, vps, atlasId);
  }
}

TEST_CASE("attribute_information", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};
  const auto atlasId = AtlasId{1};
  vps.vps_atlas_count_minus1(1).vps_atlas_id(1, atlasId);

  SECTION("No attributes") {
    const auto x = AttributeInformation{};

    REQUIRE(toString(x, AtlasId{5}) == R"(ai_attribute_count[ 5 ]=0
)");

    bitCodingTest(x, 7, vps, atlasId);
  }
  SECTION("Two attributes") {
    auto x = AttributeInformation{};
    x.ai_attribute_count(2)
        .ai_attribute_msb_align_flag(0, false)
        .ai_attribute_msb_align_flag(1, true)
        .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_REFLECTANCE)
        .ai_attribute_codec_id(1, 255)
        .ai_attribute_dimension_minus1(0, 6)
        .ai_attribute_dimension_minus1(1, 1)
        .ai_attribute_2d_bit_depth_minus1(0, 31)
        .ai_attribute_2d_bit_depth_minus1(1, 12);

    REQUIRE(toString(x, AtlasId{7}) ==
            R"(ai_attribute_count[ 7 ]=2
ai_attribute_type_id[ 7 ][ 0 ]=ATTR_REFLECTANCE
ai_attribute_codec_id[ 7 ][ 0 ]=0
ai_attribute_dimension_minus1[ 7 ][ 0 ]=6
ai_attribute_dimension_partitions_minus1[ 7 ][ 0 ]=0
ai_attribute_partition_channels_minus1[ 7 ][ 0 ][ 0 ]=6
ai_attribute_2d_bit_depth_minus1[ 7 ][ 0 ]=31
ai_attribute_msb_align_flag[ 7 ][ 0 ]=false
ai_attribute_type_id[ 7 ][ 1 ]=ATTR_TEXTURE
ai_attribute_codec_id[ 7 ][ 1 ]=255
ai_attribute_dimension_minus1[ 7 ][ 1 ]=1
ai_attribute_dimension_partitions_minus1[ 7 ][ 1 ]=0
ai_attribute_partition_channels_minus1[ 7 ][ 1 ][ 0 ]=1
ai_attribute_2d_bit_depth_minus1[ 7 ][ 1 ]=12
ai_attribute_msb_align_flag[ 7 ][ 1 ]=true
)");

    bitCodingTest(x, 67, vps, atlasId);
  }
}

TEST_CASE("packing_information", "[V3C Parameter Set]") {
  SECTION("Default Constructor + 1 Occupancy Region") {
    PackingInformation unit{};
    unit.pin_occupancy_2d_bit_depth_minus1(7)
        .pin_occupancy_MSB_align_flag(true)
        .pin_lossy_occupancy_compression_threshold(64);

    REQUIRE(toString(unit, AtlasId{4}) == R"(pin_codec_id[ 4 ]=0
pin_occupancy_present_flag[ 4 ]=true
pin_geometry_present_flag[ 4 ]=false
pin_attribute_present_flag[ 4 ]=false
pin_occupancy_2d_bit_depth_minus1[ 4 ]=7
pin_occupancy_MSB_align_flag[ 4 ]=true
pin_lossy_occupancy_compression_threshold[ 4 ]=64
pin_regions_count_minus1[ 4 ]=0
pin_region_tile_id[ 4 ][ 0 ]=0
pin_region_type_id_minus2[ 4 ][ 0 ]=0 (V3C_OVD)
pin_region_top_left_x[ 4 ][ 0 ]=0
pin_region_top_left_y[ 4 ][ 0 ]=0
pin_region_width_minus1[ 4 ][ 0 ]=0
pin_region_height_minus1[ 4 ][ 0 ]=0
pin_region_unpack_top_left_x[ 4 ][ 0 ]=0
pin_region_unpack_top_left_y[ 4 ][ 0 ]=0
pin_region_rotation_flag[ 4 ][ 0 ]=false
)");

    bitCodingTest(unit, 133);
  }

  SECTION("2 Regions: 1 Attribute + 1 Geometry") {
    PackingInformation unit{};
    unit.pin_codec_id(2)
        .pin_geometry_2d_bit_depth_minus1(9)
        .pin_geometry_MSB_align_flag(false)
        .pin_geometry_3d_coordinates_bit_depth_minus1(9)
        .pin_attribute_count(1)
        .pin_attribute_type_id(0, AiAttributeTypeId::ATTR_TEXTURE)
        .pin_attribute_2d_bit_depth_minus1(0, 9)
        .pin_attribute_MSB_align_flag(0, true)
        .pin_attribute_map_absolute_coding_persistence_flag(0, false)
        .pin_attribute_dimension_minus1(0, 0)
        .pin_attribute_dimension_partitions_minus1(0, 0)
        .pin_regions_count_minus1(1);

    for (size_t i = 0; i <= unit.pin_regions_count_minus1(); ++i) {
      unit.pin_region_tile_id(i, static_cast<uint8_t>(i + 5));
      unit.pin_region_top_left_y(i, static_cast<uint16_t>(i + 6));
      unit.pin_region_width_minus1(i, static_cast<uint16_t>(2 * i + 6));
      unit.pinRegionTypeId(i, i == 1 ? VuhUnitType::V3C_GVD : VuhUnitType::V3C_AVD);
      unit.pin_region_map_index(i, static_cast<uint8_t>(i));
      unit.pin_region_auxiliary_data_flag(i, static_cast<bool>(i));
      if (i == 0) {
        unit.pin_region_attr_index(i, static_cast<uint8_t>(0));
      }
    }

    REQUIRE(toString(unit, AtlasId{3}) == R"(pin_codec_id[ 3 ]=2
pin_occupancy_present_flag[ 3 ]=false
pin_geometry_present_flag[ 3 ]=true
pin_attribute_present_flag[ 3 ]=true
pin_geometry_2d_bit_depth_minus1[ 3 ]=9
pin_geometry_MSB_align_flag[ 3 ]=false
pin_geometry_3d_coordinates_bit_depth_minus1[ 3 ]=9
pin_attribute_count[ 3 ]=1
pin_attribute_type_id[ 3 ][ 0 ]=ATTR_TEXTURE
pin_attribute_2d_bit_depth_minus1[ 3 ][ 0 ]=9
pin_attribute_MSB_align_flag[ 3 ][ 0 ]=true
pin_attribute_map_absolute_coding_persistence_flag[ 3 ][ 0 ]=false
pin_attribute_dimension_minus1[ 3 ][ 0 ]=0
pin_regions_count_minus1[ 3 ]=1
pin_region_tile_id[ 3 ][ 0 ]=5
pin_region_type_id_minus2[ 3 ][ 0 ]=2 (V3C_AVD)
pin_region_top_left_x[ 3 ][ 0 ]=0
pin_region_top_left_y[ 3 ][ 0 ]=6
pin_region_width_minus1[ 3 ][ 0 ]=6
pin_region_height_minus1[ 3 ][ 0 ]=0
pin_region_unpack_top_left_x[ 3 ][ 0 ]=0
pin_region_unpack_top_left_y[ 3 ][ 0 ]=0
pin_region_rotation_flag[ 3 ][ 0 ]=false
pin_region_map_index[ 3 ][ 0 ]=0
pin_region_auxiliary_data_flag[ 3 ][ 0 ]=false
pin_region_attr_index[ 3 ][ 0 ]=0
pin_region_tile_id[ 3 ][ 1 ]=6
pin_region_type_id_minus2[ 3 ][ 1 ]=1 (V3C_GVD)
pin_region_top_left_x[ 3 ][ 1 ]=0
pin_region_top_left_y[ 3 ][ 1 ]=7
pin_region_width_minus1[ 3 ][ 1 ]=8
pin_region_height_minus1[ 3 ][ 1 ]=0
pin_region_unpack_top_left_x[ 3 ][ 1 ]=0
pin_region_unpack_top_left_y[ 3 ][ 1 ]=0
pin_region_rotation_flag[ 3 ][ 1 ]=false
pin_region_map_index[ 3 ][ 1 ]=1
pin_region_auxiliary_data_flag[ 3 ][ 1 ]=true
)");

    bitCodingTest(unit, 280);
  }

  SECTION("4 Regions: 1 Attribute + 2 Geometry + 1 Occupancy") {
    PackingInformation unit{};
    unit.pin_codec_id(2)
        .pin_occupancy_2d_bit_depth_minus1(1)
        .pin_occupancy_MSB_align_flag(true)
        .pin_lossy_occupancy_compression_threshold(1)
        .pin_geometry_2d_bit_depth_minus1(1)
        .pin_geometry_MSB_align_flag(true)
        .pin_geometry_3d_coordinates_bit_depth_minus1(1)
        .pin_attribute_count(static_cast<uint8_t>(1))
        .pin_attribute_type_id(0, AiAttributeTypeId::ATTR_TEXTURE)
        .pin_attribute_2d_bit_depth_minus1(0, 1)
        .pin_attribute_MSB_align_flag(0, true)
        .pin_attribute_map_absolute_coding_persistence_flag(static_cast<uint8_t>(0), true)
        .pin_attribute_dimension_minus1(0, 1)
        .pin_attribute_dimension_partitions_minus1(0, 1)
        .pin_regions_count_minus1(3)
        .pin_region_tile_id(0, static_cast<uint8_t>(5))
        .pinRegionTypeId(0, VuhUnitType::V3C_AVD)
        .pin_region_top_left_y(0, static_cast<uint16_t>(6))
        .pin_region_width_minus1(0, static_cast<uint16_t>(6))
        .pin_region_map_index(0, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(0, false)
        .pin_region_attr_index(0, static_cast<uint8_t>(0))
        .pin_region_attr_partition_index(0, 0)
        .pin_region_tile_id(1, static_cast<uint8_t>(5))
        .pinRegionTypeId(1, VuhUnitType::V3C_GVD)
        .pin_region_top_left_y(1, static_cast<uint16_t>(6))
        .pin_region_width_minus1(1, static_cast<uint16_t>(6))
        .pin_region_map_index(1, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(1, false)
        .pin_region_tile_id(2, static_cast<uint8_t>(5))
        .pinRegionTypeId(2, VuhUnitType::V3C_GVD)
        .pin_region_top_left_y(2, static_cast<uint16_t>(6))
        .pin_region_width_minus1(2, static_cast<uint16_t>(6))
        .pin_region_map_index(2, static_cast<uint8_t>(0))
        .pin_region_auxiliary_data_flag(2, false)
        .pin_region_tile_id(3, static_cast<uint8_t>(5))
        .pinRegionTypeId(3, VuhUnitType::V3C_OVD)
        .pin_region_top_left_y(3, static_cast<uint16_t>(6))
        .pin_region_width_minus1(3, static_cast<uint16_t>(6));

    REQUIRE(toString(unit, AtlasId{3}) == R"(pin_codec_id[ 3 ]=2
pin_occupancy_present_flag[ 3 ]=true
pin_geometry_present_flag[ 3 ]=true
pin_attribute_present_flag[ 3 ]=true
pin_occupancy_2d_bit_depth_minus1[ 3 ]=1
pin_occupancy_MSB_align_flag[ 3 ]=true
pin_lossy_occupancy_compression_threshold[ 3 ]=1
pin_geometry_2d_bit_depth_minus1[ 3 ]=1
pin_geometry_MSB_align_flag[ 3 ]=true
pin_geometry_3d_coordinates_bit_depth_minus1[ 3 ]=1
pin_attribute_count[ 3 ]=1
pin_attribute_type_id[ 3 ][ 0 ]=ATTR_TEXTURE
pin_attribute_2d_bit_depth_minus1[ 3 ][ 0 ]=1
pin_attribute_MSB_align_flag[ 3 ][ 0 ]=true
pin_attribute_map_absolute_coding_persistence_flag[ 3 ][ 0 ]=true
pin_attribute_dimension_minus1[ 3 ][ 0 ]=1
pin_attribute_dimension_partitions_minus1[ 3 ][ 0 ]=1
pin_regions_count_minus1[ 3 ]=3
pin_region_tile_id[ 3 ][ 0 ]=5
pin_region_type_id_minus2[ 3 ][ 0 ]=2 (V3C_AVD)
pin_region_top_left_x[ 3 ][ 0 ]=0
pin_region_top_left_y[ 3 ][ 0 ]=6
pin_region_width_minus1[ 3 ][ 0 ]=6
pin_region_height_minus1[ 3 ][ 0 ]=0
pin_region_unpack_top_left_x[ 3 ][ 0 ]=0
pin_region_unpack_top_left_y[ 3 ][ 0 ]=0
pin_region_rotation_flag[ 3 ][ 0 ]=false
pin_region_map_index[ 3 ][ 0 ]=0
pin_region_auxiliary_data_flag[ 3 ][ 0 ]=false
pin_region_attr_index[ 3 ][ 0 ]=0
pin_region_attr_partition_index[ 3 ][ 0 ]=0
pin_region_tile_id[ 3 ][ 1 ]=5
pin_region_type_id_minus2[ 3 ][ 1 ]=1 (V3C_GVD)
pin_region_top_left_x[ 3 ][ 1 ]=0
pin_region_top_left_y[ 3 ][ 1 ]=6
pin_region_width_minus1[ 3 ][ 1 ]=6
pin_region_height_minus1[ 3 ][ 1 ]=0
pin_region_unpack_top_left_x[ 3 ][ 1 ]=0
pin_region_unpack_top_left_y[ 3 ][ 1 ]=0
pin_region_rotation_flag[ 3 ][ 1 ]=false
pin_region_map_index[ 3 ][ 1 ]=0
pin_region_auxiliary_data_flag[ 3 ][ 1 ]=false
pin_region_tile_id[ 3 ][ 2 ]=5
pin_region_type_id_minus2[ 3 ][ 2 ]=1 (V3C_GVD)
pin_region_top_left_x[ 3 ][ 2 ]=0
pin_region_top_left_y[ 3 ][ 2 ]=6
pin_region_width_minus1[ 3 ][ 2 ]=6
pin_region_height_minus1[ 3 ][ 2 ]=0
pin_region_unpack_top_left_x[ 3 ][ 2 ]=0
pin_region_unpack_top_left_y[ 3 ][ 2 ]=0
pin_region_rotation_flag[ 3 ][ 2 ]=false
pin_region_map_index[ 3 ][ 2 ]=0
pin_region_auxiliary_data_flag[ 3 ][ 2 ]=false
pin_region_tile_id[ 3 ][ 3 ]=5
pin_region_type_id_minus2[ 3 ][ 3 ]=0 (V3C_OVD)
pin_region_top_left_x[ 3 ][ 3 ]=0
pin_region_top_left_y[ 3 ][ 3 ]=6
pin_region_width_minus1[ 3 ][ 3 ]=6
pin_region_height_minus1[ 3 ][ 3 ]=0
pin_region_unpack_top_left_x[ 3 ][ 3 ]=0
pin_region_unpack_top_left_y[ 3 ][ 3 ]=0
pin_region_rotation_flag[ 3 ][ 3 ]=false
)");

    bitCodingTest(unit, 526);
  }
}

TEST_CASE("group_mapping", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};
  vps.vps_atlas_count_minus1(1);

  SECTION("Default constructor") {
    const auto unit = GroupMapping{};

    REQUIRE(unit.gm_group_count() == 0);
    REQUIRE(toString(unit) == R"(gm_group_count=0
)");
    bitCodingTest(unit, 4, vps);
  }

  SECTION("Two groups") {
    auto unit = GroupMapping{};
    unit.gm_group_count(2);
    unit.gm_group_id(0, 1);
    unit.gm_group_id(1, 0);

    REQUIRE(toString(unit) == R"(gm_group_count=2
gm_group_id[ 0 ]=1
gm_group_id[ 1 ]=0
)");
    bitCodingTest(unit, 6, vps);
  }
}

TEST_CASE("v3c_parameter_set", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};

  SECTION("Example 1") {
    vps.vps_atlas_id(0, {});
    vps.vps_frame_width({}, 1920);
    vps.vps_frame_height({}, 1080);
    vps.vps_miv_extension()
        .vme_geometry_scale_enabled_flag(true)
        .vme_embedded_occupancy_enabled_flag(true);
    vps.calculateExtensionLengths();

    REQUIRE(toString(vps) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_toolset_constraints_present_flag=false
vps_v3c_parameter_set_id=0
vps_atlas_count_minus1=0
vps_atlas_id[ 0 ]=0
vps_frame_width[ 0 ]=1920
vps_frame_height[ 0 ]=1080
vps_map_count_minus1[ 0 ]=0
vps_map_absolute_coding_enabled_flag[ 0 ][ 0 ]=true
vps_map_predictor_index_diff[ 0 ][ 0 ]=0
vps_auxiliary_video_present_flag[ 0 ]=false
vps_occupancy_video_present_flag[ 0 ]=false
vps_geometry_video_present_flag[ 0 ]=false
vps_attribute_video_present_flag[ 0 ]=false
vps_extension_present_flag=true
vps_extension_count=1
vps_extensions_length_minus1=3
vps_extension_type[ 0 ]=VPS_EXT_MIV
vps_extension_length[ 0 ]=1
vme_geometry_scale_enabled_flag=true
vme_embedded_occupancy_enabled_flag=true
gm_group_count=0
)");

    REQUIRE(vps.summary() == R"(V3C parameter set 0:
  AVC Progressive High V-PCC Basic Rec0, tier false, level [unknown:0], decodes unconstrained
  Atlas 0: 1920 x 1080
, geometry scaling true, groups 0, embedded occupancy true, occupancy scaling false)");

    byteCodingTest(vps, 25);
  }

  SECTION("Example 2") {
    const auto j0 = AtlasId{30};
    const auto j1 = AtlasId{31};
    const auto j2 = AtlasId{32};
    PackingInformation packInfo{};
    packInfo.pin_occupancy_2d_bit_depth_minus1(7)
        .pin_occupancy_MSB_align_flag(true)
        .pin_lossy_occupancy_compression_threshold(64)
        .pin_attribute_count(2)
        .pin_attribute_type_id(0, AiAttributeTypeId::ATTR_REFLECTANCE)
        .pin_attribute_2d_bit_depth_minus1(0, 1)
        .pin_attribute_MSB_align_flag(0, true)
        .pin_attribute_map_absolute_coding_persistence_flag(0, false)
        .pin_attribute_dimension_minus1(0, 0)
        .pin_attribute_dimension_partitions_minus1(0, 0)
        .pin_attribute_type_id(1, AiAttributeTypeId::ATTR_NORMAL)
        .pin_attribute_2d_bit_depth_minus1(1, 5)
        .pin_attribute_MSB_align_flag(1, true)
        .pin_attribute_map_absolute_coding_persistence_flag(1, false)
        .pin_attribute_dimension_minus1(1, 0)
        .pin_attribute_dimension_partitions_minus1(1, 0);

    const auto vme2 = []() {
      auto result = VpsMiv2Extension{};
      result.vps_miv_extension().vme_embedded_occupancy_enabled_flag(true);
      result.vme_decoder_side_depth_estimation_flag(true);
      result.vme_patch_margin_enabled_flag(true);
      result.capture_device_information().cdi_device_model_count_minus1(0);
      return result;
    }();

    vps.vps_v3c_parameter_set_id(15)
        .vps_atlas_count_minus1(2)
        .vps_atlas_id(0, j0)
        .vps_atlas_id(1, j1)
        .vps_atlas_id(2, j2)
        .vps_frame_width(j0, 1920)
        .vps_frame_width(j1, 2048)
        .vps_frame_height(j0, 1080)
        .vps_frame_height(j1, 2080)
        .vps_map_count_minus1(j1, 3)
        .vps_multiple_map_streams_present_flag(j1, true)
        .vps_map_count_minus1(j2, 15)
        .vps_map_absolute_coding_enabled_flag(j2, 5, true)
        .vps_map_predictor_index_diff(j2, 8, 3)
        .vps_auxiliary_video_present_flag(j0, false)
        .vps_occupancy_video_present_flag(j0, true)
        .occupancy_information(j0, {})
        .vps_geometry_video_present_flag(j1, true)
        .geometry_information(j1, {})
        .vps_attribute_video_present_flag(j1, true)
        .attribute_information(j1, {})
        .vps_packed_video_present_flag(j2, true)
        .packing_information(j2, packInfo)
        .vps_miv_2_extension(vme2)
        .vps_extension(static_cast<VpsExtensionType>(63))
        .vps_extension_data_byte() = {2, 250, 15};
    vps.calculateExtensionLengths();

    REQUIRE(toString(vps) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_toolset_constraints_present_flag=false
vps_v3c_parameter_set_id=15
vps_atlas_count_minus1=2
vps_atlas_id[ 0 ]=30
vps_frame_width[ 30 ]=1920
vps_frame_height[ 30 ]=1080
vps_map_count_minus1[ 30 ]=0
vps_map_absolute_coding_enabled_flag[ 30 ][ 0 ]=true
vps_map_predictor_index_diff[ 30 ][ 0 ]=0
vps_auxiliary_video_present_flag[ 30 ]=false
vps_occupancy_video_present_flag[ 30 ]=true
vps_geometry_video_present_flag[ 30 ]=false
vps_attribute_video_present_flag[ 30 ]=false
oi_occupancy_codec_id[ 30 ]=0
oi_lossy_occupancy_compression_threshold[ 30 ]=0
oi_occupancy_2d_bit_depth_minus1[ 30 ]=0
oi_occupancy_MSB_align_flag[ 30 ]=false
vps_atlas_id[ 1 ]=31
vps_frame_width[ 31 ]=2048
vps_frame_height[ 31 ]=2080
vps_map_count_minus1[ 31 ]=3
vps_multiple_map_streams_present_flag[ 31 ]=true
vps_map_absolute_coding_enabled_flag[ 31 ][ 0 ]=true
vps_map_predictor_index_diff[ 31 ][ 0 ]=0
vps_map_absolute_coding_enabled_flag[ 31 ][ 1 ]=true
vps_map_absolute_coding_enabled_flag[ 31 ][ 2 ]=true
vps_map_absolute_coding_enabled_flag[ 31 ][ 3 ]=true
vps_auxiliary_video_present_flag[ 31 ]=false
vps_occupancy_video_present_flag[ 31 ]=false
vps_geometry_video_present_flag[ 31 ]=true
vps_attribute_video_present_flag[ 31 ]=true
gi_geometry_codec_id[ 31 ]=0
gi_geometry_2d_bit_depth_minus1[ 31 ]=0
gi_geometry_msb_align_flag[ 31 ]=false
gi_geometry_3d_coordinates_bit_depth_minus1[ 31 ]=0
ai_attribute_count[ 31 ]=0
vps_atlas_id[ 2 ]=32
vps_frame_width[ 32 ]=0
vps_frame_height[ 32 ]=0
vps_map_count_minus1[ 32 ]=15
vps_multiple_map_streams_present_flag[ 32 ]=false
vps_map_absolute_coding_enabled_flag[ 32 ][ 0 ]=true
vps_map_predictor_index_diff[ 32 ][ 0 ]=0
vps_map_absolute_coding_enabled_flag[ 32 ][ 1 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 2 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 3 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 4 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 5 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 6 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 7 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 8 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 9 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 10 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 11 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 12 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 13 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 14 ]=true
vps_map_absolute_coding_enabled_flag[ 32 ][ 15 ]=true
vps_auxiliary_video_present_flag[ 32 ]=false
vps_occupancy_video_present_flag[ 32 ]=false
vps_geometry_video_present_flag[ 32 ]=false
vps_attribute_video_present_flag[ 32 ]=false
vps_extension_present_flag=true
vps_extension_count=3
vps_extensions_length_minus1=38
vps_extension_type[ 0 ]=VPS_EXT_PACKED
vps_extension_length[ 0 ]=23
vps_packed_video_present_flag[ 30 ]=false
vps_packed_video_present_flag[ 31 ]=false
vps_packed_video_present_flag[ 32 ]=true
pin_codec_id[ 32 ]=0
pin_occupancy_present_flag[ 32 ]=true
pin_geometry_present_flag[ 32 ]=false
pin_attribute_present_flag[ 32 ]=true
pin_occupancy_2d_bit_depth_minus1[ 32 ]=7
pin_occupancy_MSB_align_flag[ 32 ]=true
pin_lossy_occupancy_compression_threshold[ 32 ]=64
pin_attribute_count[ 32 ]=2
pin_attribute_type_id[ 32 ][ 0 ]=ATTR_REFLECTANCE
pin_attribute_2d_bit_depth_minus1[ 32 ][ 0 ]=1
pin_attribute_MSB_align_flag[ 32 ][ 0 ]=true
pin_attribute_map_absolute_coding_persistence_flag[ 32 ][ 0 ]=false
pin_attribute_dimension_minus1[ 32 ][ 0 ]=0
pin_attribute_type_id[ 32 ][ 1 ]=ATTR_NORMAL
pin_attribute_2d_bit_depth_minus1[ 32 ][ 1 ]=5
pin_attribute_MSB_align_flag[ 32 ][ 1 ]=true
pin_attribute_map_absolute_coding_persistence_flag[ 32 ][ 1 ]=false
pin_attribute_dimension_minus1[ 32 ][ 1 ]=0
pin_regions_count_minus1[ 32 ]=0
pin_region_tile_id[ 32 ][ 0 ]=0
pin_region_type_id_minus2[ 32 ][ 0 ]=0 (V3C_OVD)
pin_region_top_left_x[ 32 ][ 0 ]=0
pin_region_top_left_y[ 32 ][ 0 ]=0
pin_region_width_minus1[ 32 ][ 0 ]=0
pin_region_height_minus1[ 32 ][ 0 ]=0
pin_region_unpack_top_left_x[ 32 ][ 0 ]=0
pin_region_unpack_top_left_y[ 32 ][ 0 ]=0
pin_region_rotation_flag[ 32 ][ 0 ]=false
vps_extension_type[ 1 ]=VPS_EXT_MIV_2
vps_extension_length[ 1 ]=4
vme_geometry_scale_enabled_flag=false
vme_embedded_occupancy_enabled_flag=true
gm_group_count=0
vme_decoder_side_depth_estimation_flag=true
vme_patch_margin_enabled_flag=true
vme_capture_device_information_present_flag=true
cdi_device_model_count_minus1=0
cdi_device_model_id[0]=0
cdi_device_class_id[0]=0
vme_colorized_geometry_enabled_flag=false
vps_extension_type[ 2 ]=[unknown:63]
vps_extension_length[ 2 ]=3
vps_extension_data_byte=2
vps_extension_data_byte=250
vps_extension_data_byte=15
)");
    REQUIRE(vps.summary() == R"(V3C parameter set 15:
  AVC Progressive High V-PCC Basic Rec0, tier false, level [unknown:0], decodes unconstrained
  Atlas 30: 1920 x 1080; [OI: codec 0, 0, 2D 1, align false]
  Atlas 31: 2048 x 2080; [GI: codec 0, 2D 1, algin false, 3D 1]; [AI: 0
  Atlas 32: 0 x 0; [PIN: regions 1]
, geometry scaling false, groups 0, embedded occupancy true, occupancy scaling false)");

    byteCodingTest(vps, 76);
  }

  SECTION("Example 3 for mpi") {
    auto ptci = ProfileToolsetConstraintsInformation{};
    ptci.ptc_restricted_geometry_flag(true);

    auto x = ProfileTierLevel{};
    x.ptl_profile_toolset_constraints_information(ptci);

    auto y = AttributeInformation{};
    y.ai_attribute_count(2)
        .ai_attribute_msb_align_flag(0, false)
        .ai_attribute_msb_align_flag(1, false)
        .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_TEXTURE)
        .ai_attribute_type_id(1, AiAttributeTypeId::ATTR_TRANSPARENCY)
        .ai_attribute_codec_id(0, 1)
        .ai_attribute_codec_id(1, 1)
        .ai_attribute_dimension_minus1(0, 2)
        .ai_attribute_dimension_minus1(1, 0)
        .ai_attribute_2d_bit_depth_minus1(0, 9)
        .ai_attribute_2d_bit_depth_minus1(1, 9);

    const auto j0 = AtlasId{20};

    vps.profile_tier_level(x)
        .vps_v3c_parameter_set_id(15)
        .vps_atlas_count_minus1(0)
        .vps_atlas_id(0, j0)
        .vps_frame_width(j0, 4096)
        .vps_frame_height(j0, 2048)
        .vps_map_count_minus1(j0, 0)
        .vps_auxiliary_video_present_flag(j0, false)
        .vps_occupancy_video_present_flag(j0, false)
        .vps_attribute_video_present_flag(j0, true)
        .attribute_information(j0, y)
        .vps_miv_extension(VpsMivExtension{}.vme_embedded_occupancy_enabled_flag(true));
    vps.calculateExtensionLengths();

    REQUIRE(toString(vps) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_toolset_constraints_present_flag=true
ptc_one_v3c_frame_only_flag=false
ptc_eom_constraint_flag=false
ptc_max_map_count_minus1=15
ptc_max_atlas_count_minus1=15
ptc_multiple_map_streams_constraint_flag=false
ptc_plr_constraint_flag=false
ptc_attribute_max_dimension_minus1=63
ptc_attribute_max_dimension_partitions_minus1=63
ptc_no_eight_orientations_constraint_flag=false
ptc_no_45degree_projection_patch_constraint_flag=false
ptc_restricted_geometry_flag=true
ptc_num_reserved_constraint_bytes=0
vps_v3c_parameter_set_id=15
vps_atlas_count_minus1=0
vps_atlas_id[ 0 ]=20
vps_frame_width[ 20 ]=4096
vps_frame_height[ 20 ]=2048
vps_map_count_minus1[ 20 ]=0
vps_map_absolute_coding_enabled_flag[ 20 ][ 0 ]=true
vps_map_predictor_index_diff[ 20 ][ 0 ]=0
vps_auxiliary_video_present_flag[ 20 ]=false
vps_occupancy_video_present_flag[ 20 ]=false
vps_geometry_video_present_flag[ 20 ]=false
vps_attribute_video_present_flag[ 20 ]=true
ai_attribute_count[ 20 ]=2
ai_attribute_type_id[ 20 ][ 0 ]=ATTR_TEXTURE
ai_attribute_codec_id[ 20 ][ 0 ]=1
ai_attribute_dimension_minus1[ 20 ][ 0 ]=2
ai_attribute_dimension_partitions_minus1[ 20 ][ 0 ]=0
ai_attribute_partition_channels_minus1[ 20 ][ 0 ][ 0 ]=2
ai_attribute_2d_bit_depth_minus1[ 20 ][ 0 ]=9
ai_attribute_msb_align_flag[ 20 ][ 0 ]=false
ai_attribute_type_id[ 20 ][ 1 ]=ATTR_TRANSPARENCY
ai_attribute_codec_id[ 20 ][ 1 ]=1
ai_attribute_dimension_minus1[ 20 ][ 1 ]=0
ai_attribute_dimension_partitions_minus1[ 20 ][ 1 ]=0
ai_attribute_partition_channels_minus1[ 20 ][ 1 ][ 0 ]=0
ai_attribute_2d_bit_depth_minus1[ 20 ][ 1 ]=9
ai_attribute_msb_align_flag[ 20 ][ 1 ]=false
vps_extension_present_flag=true
vps_extension_count=1
vps_extensions_length_minus1=3
vps_extension_type[ 0 ]=VPS_EXT_MIV
vps_extension_length[ 0 ]=1
vme_geometry_scale_enabled_flag=false
vme_embedded_occupancy_enabled_flag=true
gm_group_count=0
)");

    REQUIRE(vps.summary() == R"(V3C parameter set 15:
  AVC Progressive High V-PCC Basic Restricted Geometry Rec0, tier false, level [unknown:0], decodes unconstrained
  Atlas 20: 4096 x 2048; [AI: 2, ATTR_TEXTURE, codec 1, dims 3, 2D 10, align false, ATTR_TRANSPARENCY, codec 1, dims 1, 2D 10, align false]
, geometry scaling false, groups 0, embedded occupancy true, occupancy scaling false)");

    byteCodingTest(vps, 38);

    REQUIRE(vps.attrIdxOf(AtlasId{20}, AiAttributeTypeId::ATTR_TEXTURE) == 0);
    REQUIRE(vps.attrIdxOf(AtlasId{20}, AiAttributeTypeId::ATTR_TRANSPARENCY) == 1);
  }
}

TEST_CASE("MivBitstream::VpsExtensionType") {
  using TMIV::MivBitstream::VpsExtensionType;

  CHECK(toString(VpsExtensionType::VPS_EXT_UNSPECIFIED) == "VPS_EXT_UNSPECIFIED"s);
  CHECK(toString(VpsExtensionType::VPS_EXT_PACKED) == "VPS_EXT_PACKED"s);
  CHECK(toString(VpsExtensionType::VPS_EXT_MIV) == "VPS_EXT_MIV"s);
  CHECK(toString(VpsExtensionType::VPS_EXT_MIV_2) == "VPS_EXT_MIV_2"s);
  CHECK(toString(static_cast<VpsExtensionType>(4)) == "[unknown:4]"s);
  CHECK(toString(static_cast<VpsExtensionType>(255)) == "[unknown:255]"s);
}
} // namespace TMIV::MivBitstream
