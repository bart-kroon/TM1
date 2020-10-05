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

#include "test.h"

#include <TMIV/MivBitstream/V3cParameterSet.h>

namespace TMIV::MivBitstream {
TEST_CASE("AtlasId", "[V3C Parameter Set]") {
  SECTION("Default constructor") {
    const auto j = AtlasId{};
    REQUIRE(toString(j) == "0");
    REQUIRE(j == j);
    REQUIRE(bitCodingTest(j, 6));
  }

  SECTION("Explicit conversion constructor") {
    const auto j = AtlasId{42};
    REQUIRE(toString(j) == "42");
    REQUIRE(j == j);
    REQUIRE(j != AtlasId{});
    REQUIRE(bitCodingTest(j, 6));
  }
}

TEST_CASE("profile_tier_level", "[V3C Parameter Set]") {
  auto x = ProfileTierLevel{};

  REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0 (V-PCC)
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_tool_constraints_present_flag=false
)");

  REQUIRE(bitCodingTest(x, 72));

  SECTION("Example") {
    x.ptl_tier_flag(true)
        .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
        .ptl_profile_toolset_idc(PtlProfilePccToolsetIdc::VPCC_Extended)
        .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained)
        .ptl_max_decodes_idc(PtlMaxDecodesIdc::max_4)
        .ptl_level_idc(PtlLevelIdc::Level_2_0)
        .ptl_num_sub_profiles(2)
        .ptl_extended_sub_profile_flag(true)
        .ptl_sub_profile_idc(0, 3)
        .ptl_sub_profile_idc(1, UINT64_MAX)
        .ptl_tool_constraints_present_flag(false);

    REQUIRE(toString(x) == R"(ptl_tier_flag=true
ptl_profile_codec_group_idc=HEVC Main10
ptl_profile_toolset_idc=V-PCC Extended
ptl_profile_reconstruction_idc=Rec Unconstrained
ptl_max_decodes_idc=max_4
ptl_level_idc=Level 2.0
ptl_num_sub_profiles=2
ptl_extended_sub_profile_flag=true
ptl_sub_profile_idc[ 0 ]=3
ptl_sub_profile_idc[ 1 ]=18446744073709551615
ptl_tool_constraints_present_flag=false
)");

    REQUIRE(bitCodingTest(x, 200));
  }
}

TEST_CASE("occupancy_information", "[V3C Parameter Set]") {
  auto x = OccupancyInformation{};

  REQUIRE(toString(x, AtlasId{3}) == R"(oi_occupancy_codec_id( 3 )=0
oi_lossy_occupancy_compression_threshold( 3 )=0
oi_occupancy_2d_bit_depth_minus1( 3 )=0
oi_occupancy_MSB_align_flag( 3 )=false
)");

  REQUIRE(bitCodingTest(x, 22));

  SECTION("Example") {
    x.oi_occupancy_codec_id(255)
        .oi_lossy_occupancy_compression_threshold(255)
        .oi_occupancy_2d_bit_depth_minus1(31)
        .oi_occupancy_MSB_align_flag(true);

    REQUIRE(toString(x, AtlasId{4}) == R"(oi_occupancy_codec_id( 4 )=255
oi_lossy_occupancy_compression_threshold( 4 )=255
oi_occupancy_2d_bit_depth_minus1( 4 )=31
oi_occupancy_MSB_align_flag( 4 )=true
)");

    REQUIRE(bitCodingTest(x, 22));
  }
}

TEST_CASE("geometry_information", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};
  const auto atlasId = AtlasId{0};
  vps.vps_auxiliary_video_present_flag(atlasId, false);

  auto x = GeometryInformation{};

  REQUIRE(toString(x, AtlasId{0}) == R"(gi_geometry_codec_id( 0 )=0
gi_geometry_2d_bit_depth_minus1( 0 )=0
gi_geometry_MSB_align_flag( 0 )=false
gi_geometry_3d_coordinates_bit_depth_minus1( 0 )=0
)");

  REQUIRE(bitCodingTest(x, 19, vps, atlasId));

  SECTION("Example") {
    x.gi_geometry_codec_id(255)
        .gi_geometry_2d_bit_depth_minus1(31)
        .gi_geometry_MSB_align_flag(true)
        .gi_geometry_3d_coordinates_bit_depth_minus1(31);

    REQUIRE(toString(x, AtlasId{0}) == R"(gi_geometry_codec_id( 0 )=255
gi_geometry_2d_bit_depth_minus1( 0 )=31
gi_geometry_MSB_align_flag( 0 )=true
gi_geometry_3d_coordinates_bit_depth_minus1( 0 )=31
)");

    REQUIRE(bitCodingTest(x, 19, vps, atlasId));
  }
}

TEST_CASE("attribute_information", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};
  const auto atlasId = AtlasId{1};
  vps.vps_atlas_count_minus1(1).vps_atlas_id(1, atlasId);

  SECTION("No attributes") {
    const auto x = AttributeInformation{};

    REQUIRE(toString(x, AtlasId{5}) == R"(ai_attribute_count( 5 )=0
)");

    REQUIRE(bitCodingTest(x, 7, vps, atlasId));
  }
  SECTION("Two attributes") {
    auto x = AttributeInformation{};
    x.ai_attribute_count(2)
        .ai_attribute_MSB_align_flag(0, false)
        .ai_attribute_MSB_align_flag(1, true)
        .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_REFLECTANCE)
        .ai_attribute_codec_id(1, 255)
        .ai_attribute_dimension_minus1(0, 6)
        .ai_attribute_dimension_minus1(1, 1)
        .ai_attribute_2d_bit_depth_minus1(0, 31)
        .ai_attribute_2d_bit_depth_minus1(1, 12);

    REQUIRE(toString(x, AtlasId{7}) ==
            R"(ai_attribute_count( 7 )=2
ai_attribute_type_id( 7, 0 )=ATTR_REFLECTANCE
ai_attribute_codec_id( 7, 0 )=0
ai_attribute_dimension_minus1( 7, 0 )=6
ai_attribute_2d_bit_depth_minus1( 7, 0 )=31
ai_attribute_MSB_align_flag( 7, 0 )=false
ai_attribute_type_id( 7, 1 )=ATTR_TEXTURE
ai_attribute_codec_id( 7, 1 )=255
ai_attribute_dimension_minus1( 7, 1 )=1
ai_attribute_2d_bit_depth_minus1( 7, 1 )=12
ai_attribute_MSB_align_flag( 7, 1 )=true
)");

    REQUIRE(bitCodingTest(x, 67, vps, atlasId));
  }
}

TEST_CASE("packing_information", "[V3C Parameter Set]") {
  SECTION("Default Constructor") {
    const PackingInformation unit{};

    REQUIRE(toString(unit, 4) == R"(pin_codec_id(4)=0
pin_regions_count_minus1(4)=0
pin_region_tile_id(4,0)=0
pin_region_type_id_minus2(4,0)=V3C_VPS
pin_region_top_left_x(4,0)=0
pin_region_top_left_y(4,0)=0
pin_region_width_minus1(4,0)=0
pin_region_height_minus1(4,0)=0
pin_region_map_index(4,0)=0
pin_region_rotation_flag(4,0)=false
)");

    const std::size_t expectedNumberOfBits = 8             // pin_codec_id
                                             + 1           // pin_regions_count_minus1
                                             + 1 * (       // number in pin_regions_count_minus1 + 1
                                                       +8  // pin_region_tile_id
                                                       + 2 // pin_region_type_id_minus2
                                                       + 16  // pin_region_top_left_x
                                                       + 16  // pin_region_top_left_y
                                                       + 16  // pin_region_width_minus1
                                                       + 16  // pin_region_height_minus1
                                                       + 4   // pin_region_map_index
                                                       + 1); // pin_region_rotation_flag

    REQUIRE(bitCodingTest(unit, expectedNumberOfBits));
  }

  SECTION("Relevant region types") {
    PackingInformation unit{};
    unit.pin_codec_id(2);
    unit.pin_regions_count_minus1(1);
    for (std::size_t i = 0; i <= unit.pin_regions_count_minus1(); ++i) {
      // Pseudorandom values
      unit.pin_region_tile_id(i, i + 5);
      unit.pin_region_top_left_y(i, i + 6);
      unit.pin_region_width_minus1(i, 2 * i + 6);
      // Testing with V3C_AVD an V3C_GVD
      unit.pin_region_type_id_minus2(
          i, static_cast<VuhUnitType>(static_cast<std::uint8_t>(VuhUnitType::V3C_AVD) - 2U -
                                      static_cast<unsigned>(i)));
      unit.pin_region_auxiliary_data_flag(i, static_cast<bool>(i));
      if (i == 0) {
        unit.pin_region_attr_type_id(i, i + 1);
        unit.pin_region_attr_partitions_flag(i, true);
        unit.pin_region_attr_partition_index(i, 0);
        unit.pin_region_attr_partitions_minus1(i, i + 3);
      }
    }

    REQUIRE(toString(unit, 3) == R"(pin_codec_id(3)=2
pin_regions_count_minus1(3)=1
pin_region_tile_id(3,0)=5
pin_region_type_id_minus2(3,0)=V3C_OVD
pin_region_top_left_x(3,0)=0
pin_region_top_left_y(3,0)=6
pin_region_width_minus1(3,0)=6
pin_region_height_minus1(3,0)=0
pin_region_map_index(3,0)=0
pin_region_rotation_flag(3,0)=false
pin_region_auxiliary_data_flag(3,0)=false
pin_region_attr_type_id(3,0)=1
pin_region_attr_partitions_flag(3,0)=true
pin_region_attr_partition_index(3,0)=0
pin_region_attr_partitions_minus1(3,0)=3
pin_region_tile_id(3,1)=6
pin_region_type_id_minus2(3,1)=V3C_AD
pin_region_top_left_x(3,1)=0
pin_region_top_left_y(3,1)=7
pin_region_width_minus1(3,1)=8
pin_region_height_minus1(3,1)=0
pin_region_map_index(3,1)=0
pin_region_rotation_flag(3,1)=false
pin_region_auxiliary_data_flag(3,1)=true
)");

    const std::size_t expectedNumberOfBits = 8               // pin_codec_id
                                             + 3             // pin_regions_count_minus1
                                             + 1 * (         // first region, with V3C_AVD
                                                       8     // pin_region_tile_id
                                                       + 2   // pin_region_type_id_minus2
                                                       + 16  // pin_region_top_left_x
                                                       + 16  // pin_region_top_left_y
                                                       + 16  // pin_region_width_minus1
                                                       + 16  // pin_region_height_minus1
                                                       + 4   // pin_region_map_index
                                                       + 1   // pin_region_rotation_flag
                                                       + 1)  // pin_region_auxiliary_data_flag
                                             + 1 * (         // second region, with V3C_GVD
                                                       8     // pin_region_tile_id
                                                       + 2   // pin_region_type_id_minus2
                                                       + 16  // pin_region_top_left_x
                                                       + 16  // pin_region_top_left_y
                                                       + 16  // pin_region_width_minus1
                                                       + 16  // pin_region_height_minus1
                                                       + 4   // pin_region_map_index
                                                       + 1   // pin_region_rotation_flag
                                                       + 1   // pin_region_auxiliary_data_flag
                                                       + 4   // pin_region_attr_type_id
                                                       + 1   // pin_region_attr_partitions_flag
                                                       + 5   // pin_region_attr_partition_index
                                                       + 6); // pin_region_attr_partitions_minus1

    REQUIRE(bitCodingTest(unit, expectedNumberOfBits));
  }
}

TEST_CASE("v3c_parameter_set", "[V3C Parameter Set]") {
  auto vps = V3cParameterSet{};

  SECTION("Example 1") {
    vps.vps_atlas_id(0, {});
    vps.vps_frame_width({}, 1920);
    vps.vps_frame_height({}, 1080);
    vps.vps_extension_present_flag(true);
    vps.vps_miv_extension_present_flag(true);
    vps.vps_miv_extension()
        .vme_depth_low_quality_flag(true)
        .vme_geometry_scale_enabled_flag(true)
        .vme_num_groups_minus1(3)
        .vme_max_entities_minus1(20)
        .vme_embedded_occupancy_flag(true);

    REQUIRE(toString(vps) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0 (V-PCC)
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_tool_constraints_present_flag=false
vps_v3c_parameter_set_id=0
vps_atlas_count_minus1=0
vps_atlas_id( 0 )=0
vps_frame_width( 0 )=1920
vps_frame_height( 0 )=1080
vps_map_count_minus1( 0 )=0
vps_auxiliary_video_present_flag( 0 )=false
vps_occupancy_video_present_flag( 0 )=false
vps_geometry_video_present_flag( 0 )=false
vps_attribute_video_present_flag( 0 )=false
vps_extension_present_flag=true
vps_miv_extension_present_flag=true
vps_extension_7bits=0
vme_depth_low_quality_flag=true
vme_geometry_scale_enabled_flag=true
vme_num_groups_minus1=3
vme_max_entities_minus1=20
vme_embedded_occupancy_flag=true
)");

    REQUIRE(byteCodingTest(vps, 21));
  }

  SECTION("Example 2") {
    const auto j0 = AtlasId{30};
    const auto j1 = AtlasId{31};
    const auto j2 = AtlasId{32};
    vps.vps_v3c_parameter_set_id(15)
        .vps_atlas_count_minus1(2)
        .vps_atlas_id(0, j0)
        .vps_atlas_id(1, j1)
        .vps_atlas_id(2, j2)
        .vps_frame_width(j0, 1920)
        .vps_frame_width(j1, 2048)
        .vps_frame_height(j0, 1080)
        .vps_frame_height(j1, 2080)
        .vps_map_count_minus1(j2, 15)
        .vps_auxiliary_video_present_flag(j0, false)
        .vps_occupancy_video_present_flag(j0, true)
        .occupancy_information(j0, {})
        .vps_geometry_video_present_flag(j1, true)
        .geometry_information(j1, {})
        .vps_attribute_video_present_flag(j2, true)
        .attribute_information(j2, {})
        .vps_extension_present_flag(true)
        .vps_miv_extension_present_flag(true)
        .vps_miv_extension(VpsMivExtension{})
        .vps_extension_7bits(127)
        .vpsExtensionData({2, 250, 15});

    REQUIRE(toString(vps) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0 (V-PCC)
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_tool_constraints_present_flag=false
vps_v3c_parameter_set_id=15
vps_atlas_count_minus1=2
vps_atlas_id( 0 )=30
vps_frame_width( 30 )=1920
vps_frame_height( 30 )=1080
vps_map_count_minus1( 30 )=0
vps_auxiliary_video_present_flag( 30 )=false
vps_occupancy_video_present_flag( 30 )=true
vps_geometry_video_present_flag( 30 )=false
vps_attribute_video_present_flag( 30 )=false
oi_occupancy_codec_id( 30 )=0
oi_lossy_occupancy_compression_threshold( 30 )=0
oi_occupancy_2d_bit_depth_minus1( 30 )=0
oi_occupancy_MSB_align_flag( 30 )=false
vps_atlas_id( 1 )=31
vps_frame_width( 31 )=2048
vps_frame_height( 31 )=2080
vps_map_count_minus1( 31 )=0
vps_auxiliary_video_present_flag( 31 )=false
vps_occupancy_video_present_flag( 31 )=false
vps_geometry_video_present_flag( 31 )=true
vps_attribute_video_present_flag( 31 )=false
gi_geometry_codec_id( 31 )=0
gi_geometry_2d_bit_depth_minus1( 31 )=0
gi_geometry_MSB_align_flag( 31 )=false
gi_geometry_3d_coordinates_bit_depth_minus1( 31 )=0
vps_atlas_id( 2 )=32
vps_frame_width( 32 )=0
vps_frame_height( 32 )=0
vps_map_count_minus1( 32 )=15
vps_auxiliary_video_present_flag( 32 )=false
vps_occupancy_video_present_flag( 32 )=false
vps_geometry_video_present_flag( 32 )=false
vps_attribute_video_present_flag( 32 )=true
ai_attribute_count( 32 )=0
vps_extension_present_flag=true
vps_miv_extension_present_flag=true
vps_extension_7bits=127
vme_depth_low_quality_flag=false
vme_geometry_scale_enabled_flag=false
vme_num_groups_minus1=0
vme_max_entities_minus1=0
vme_embedded_occupancy_flag=true
vps_extension_length_minus1=2
vps_extension_data_byte=2
vps_extension_data_byte=250
vps_extension_data_byte=15
)");

    REQUIRE(byteCodingTest(vps, 41));
  }
}
} // namespace TMIV::MivBitstream
