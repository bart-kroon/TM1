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

#include "test.h"

#include <TMIV/MivBitstream/VpccParameterSet.h>

using namespace TMIV::MivBitstream;

TEST_CASE("profile_tier_level", "[VPCC Parameter Set]") {
  auto x = ProfileTierLevel{};

  REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
)");

  REQUIRE(bitCodingTest(x, 64));

  SECTION("Example") {
    x.ptl_tier_flag(true)
        .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
        .ptl_profile_pcc_toolset_idc(PtlProfilePccToolsetIdc::Extended)
        .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Unconstrained)
        .ptl_level_idc(PtlLevelIdc::Level_2_0);

    REQUIRE(toString(x) == R"(ptl_tier_flag=true
ptl_profile_codec_group_idc=HEVC Main10
ptl_profile_pcc_toolset_idc=Extended
ptl_profile_reconstruction_idc=Unconstrained
ptl_level_idc=Level 2.0
)");

    REQUIRE(bitCodingTest(x, 64));
  }
}

TEST_CASE("occupancy_information", "[VPCC Parameter Set]") {
  auto x = OccupancyInformation{};

  REQUIRE(toString(x, 3) == R"(oi_occupancy_codec_id( 3 )=0
oi_lossy_occupancy_map_compression_threshold( 3 )=0
oi_occupancy_nominal_2d_bitdepth_minus1( 3 )=0
oi_occupancy_MSB_align_flag( 3 )=false
)");

  REQUIRE(bitCodingTest(x, 22));

  SECTION("Example") {
    x.oi_occupancy_codec_id(255)
        .oi_lossy_occupancy_map_compression_threshold(255)
        .oi_occupancy_nominal_2d_bitdepth_minus1(31)
        .oi_occupancy_MSB_align_flag(true);

    REQUIRE(toString(x, 4) == R"(oi_occupancy_codec_id( 4 )=255
oi_lossy_occupancy_map_compression_threshold( 4 )=255
oi_occupancy_nominal_2d_bitdepth_minus1( 4 )=31
oi_occupancy_MSB_align_flag( 4 )=true
)");

    REQUIRE(bitCodingTest(x, 22));
  }
}

TEST_CASE("geometry_information", "[VPCC Parameter Set]") {
  auto vps = VpccParameterSet{};
  const auto atlasId = uint8_t(0);
  vps.vps_auxiliary_video_present_flag(atlasId, false);

  auto x = GeometryInformation{};

  REQUIRE(toString(x, 0) == R"(gi_geometry_codec_id( 0 )=0
gi_geometry_nominal_2d_bitdepth_minus1( 0 )=0
gi_geometry_MSB_align_flag( 0 )=false
gi_geometry_3d_coordinates_bitdepth_minus1( 0 )=0
)");

  REQUIRE(bitCodingTest(x, 19, vps, atlasId));

  SECTION("Example") {
    x.gi_geometry_codec_id(255)
        .gi_geometry_nominal_2d_bitdepth_minus1(31)
        .gi_geometry_MSB_align_flag(true)
        .gi_geometry_3d_coordinates_bitdepth_minus1(31);

    REQUIRE(toString(x, 0) == R"(gi_geometry_codec_id( 0 )=255
gi_geometry_nominal_2d_bitdepth_minus1( 0 )=31
gi_geometry_MSB_align_flag( 0 )=true
gi_geometry_3d_coordinates_bitdepth_minus1( 0 )=31
)");

    REQUIRE(bitCodingTest(x, 19, vps, atlasId));
  }
}

TEST_CASE("attribute_information", "[VPCC Parameter Set]") {
  auto vps = VpccParameterSet{};
  vps.vps_atlas_count_minus1(1);
  const auto atlasId = 1;

  SECTION("No attributes") {
    const auto x = AttributeInformation{};

    REQUIRE(toString(x, 5) == R"(ai_attribute_count( 5 )=0
)");

    REQUIRE(bitCodingTest(x, 7, vps, atlasId));
  }
  SECTION("Two attributes") {
    auto x = AttributeInformation{};
    x.ai_attribute_count(2)
        .ai_attribute_MSB_align_flag(true)
        .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_REFLECTANCE)
        .ai_attribute_codec_id(1, 255)
        .ai_attribute_dimension_minus1(0, 6)
        .ai_attribute_dimension_minus1(1, 1)
        .ai_attribute_nominal_2d_bitdepth(0, 32)
        .ai_attribute_nominal_2d_bitdepth(1, 13);

    REQUIRE(toString(x, 7) ==
            R"(ai_attribute_count( 7 )=2
ai_attribute_MSB_align_flag( 7 )=true
ai_attribute_type_id( 7, 0 )=ATTR_REFLECTANCE
ai_attribute_codec_id( 7, 0 )=0
ai_attribute_dimension_minus1( 7, 0 )=6
ai_attribute_nominal_2d_bitdepth( 7, 0 )=32
ai_attribute_type_id( 7, 1 )=ATTR_TEXTURE
ai_attribute_codec_id( 7, 1 )=255
ai_attribute_dimension_minus1( 7, 1 )=1
ai_attribute_nominal_2d_bitdepth( 7, 1 )=13
)");

    REQUIRE(bitCodingTest(x, 66, vps, atlasId));
  }
}

TEST_CASE("vpcc_parameter_set", "[VPCC Parameter Set]") {
  auto vps = VpccParameterSet{};

  SECTION("Example 1") {
    vps.vps_frame_width(0, 1920);
    vps.vps_frame_height(0, 1080);
    vps.vps_extension_present_flag(true);
    vps.vps_miv_extension_flag(true);
    vps.miv_sequence_params()
        .msp_depth_low_quality_flag(true)
        .msp_geometry_scale_enabled_flag(true)
        .msp_num_groups_minus1(3)
        .msp_max_entities_minus1(20);
    vps.vps_miv_sequence_vui_params_present_flag(false);

    REQUIRE(toString(vps) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
vps_vpcc_parameter_set_id=0
vps_miv_mode_flag=false
vps_atlas_count_minus1=0
vps_frame_width( 0 )=1920
vps_frame_height( 0 )=1080
vps_map_count_minus1( 0 )=0
vps_auxiliary_video_present_flag( 0 )=false
oi_occupancy_codec_id( 0 )=0
oi_lossy_occupancy_map_compression_threshold( 0 )=0
oi_occupancy_nominal_2d_bitdepth_minus1( 0 )=0
oi_occupancy_MSB_align_flag( 0 )=false
gi_geometry_codec_id( 0 )=0
gi_geometry_nominal_2d_bitdepth_minus1( 0 )=0
gi_geometry_MSB_align_flag( 0 )=false
gi_geometry_3d_coordinates_bitdepth_minus1( 0 )=0
ai_attribute_count( 0 )=0
vps_extension_present_flag=true
vps_miv_extension_flag=true
msp_depth_low_quality_flag=true
msp_geometry_scale_enabled_flag=true
msp_num_groups_minus1=3
msp_max_entities_minus1=20
vps_miv_sequence_vui_params_present_flag=false
)");

    REQUIRE(byteCodingTest(vps, 25));
  }
}
