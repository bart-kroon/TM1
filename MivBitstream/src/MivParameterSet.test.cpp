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

#define CATCH_CONFIG_MAIN
#include "test.h"

#include <TMIV/MivBitstream/MivParameterSet.h>

using namespace TMIV::MivBitstream;
using namespace TMIV::Metadata;
using namespace TMIV::VpccBitstream;

TEST_CASE("miv_sequence_params", "[MIV Parameter Set]") {
  auto x = MivSequenceParams{};

  REQUIRE(toString(x) == R"(msp_profile_idc=Basic
msp_depth_params_num_bits=10
msp_depth_low_quality_flag=false
msp_num_groups=1
msp_max_entities=1
msp_viewing_space_present_flag=false
msp_extension_present_flag=false
)");

  SECTION("Example 1") {
    auto vp = ViewParams{};
    vp.size = {1, 1};

    x.view_params_list(ViewParamsList{{vp}});

    REQUIRE(toString(x) == R"(msp_profile_idc=Basic
msp_depth_params_num_bits=10
View  0: [1, 1], ERP [0, 0] x [0, 0] deg, norm. disp in [0, 0] m^-1, hasOccupancy false, depthOccMapThreshold 0, pose [ 0.000,  0.000,  0.000] m, [0, 0, 0] deg
msp_depth_low_quality_flag=false
msp_num_groups=1
msp_max_entities=1
msp_viewing_space_present_flag=false
msp_extension_present_flag=false
)");
    REQUIRE(bitCodingTest(x, 478));
  }

  SECTION("Example 2") {
    auto vp1 = ViewParams{};
    vp1.size = {4096, 2048};
    vp1.position = {1.F, 2.F, 3.F};
    vp1.rotation = {0.4F, 0.5F, 0.6F};
    vp1.projection = ErpParams{{-10.F, 20.F}, {-30.F, 40.F}};
    vp1.normDispRange = {0.1F, 0.2F};
    vp1.depthOccMapThreshold = 345;
    vp1.depthStart = 1100;

    auto vp2 = ViewParams{};
    vp2.size = {1, 1};

    const auto vpl = ViewParamsList{{vp1, vp2}};

    const auto vs =
        ViewingSpace{{{ElementaryShapeOperation::add,
                       ElementaryShape{{PrimitiveShape{Cuboid{{}, {}}, {}, {}, {}}}, {}}}}};

    x.msp_profile_idc(MspProfileIdc::Extended)
        .msp_depth_params_num_bits(11)
        .view_params_list(vpl)
        .msp_depth_low_quality_flag(true)
        .msp_num_groups(3)
        .msp_max_entities(4)
        .msp_viewing_space_present_flag(true)
        .viewing_space(vs)
        .msp_extension_present_flag(true);

    REQUIRE(toString(x) == R"(msp_profile_idc=Extended
msp_depth_params_num_bits=11
View  0: [4096, 2048], ERP [-10, 20] x [-30, 40] deg, norm. disp in [0.1, 0.2] m^-1, hasOccupancy false, depthOccMapThreshold 345, depthStart 1100, pose [ 1.000,  2.000,  3.000] m, [0.4, 0.5, 0.6] deg
View  1: [1, 1], ERP [0, 0] x [0, 0] deg, norm. disp in [0, 0] m^-1, hasOccupancy false, depthOccMapThreshold 0, pose [ 0.000,  0.000,  0.000] m, [0, 0, 0] deg
msp_depth_low_quality_flag=true
msp_num_groups=3
msp_max_entities=4
msp_viewing_space_present_flag=true
Viewing space:
add (add, cuboid [0, 0, 0] size [0, 0, 0])
msp_extension_present_flag=true
)");
    REQUIRE(bitCodingTest(x, 1053));
  }
}

TEST_CASE("miv_parameter_set", "[MIV Parameter Set]") {
  auto x = MivParameterSet{};
  REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
vps_vpcc_parameter_set_id=0
vps_atlas_count=0
vps_extension_present_flag=false
miv_sequence_params_present_flag=vps_extension_present_flag
)");

  SECTION("MIV extension") {
    x.miv_sequence_params_present_flag(true);
    REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
vps_vpcc_parameter_set_id=0
vps_atlas_count=0
vps_extension_present_flag=true
miv_sequence_params_present_flag=vps_extension_present_flag
msp_profile_idc=Basic
msp_depth_params_num_bits=10
msp_depth_low_quality_flag=false
msp_num_groups=1
msp_max_entities=1
msp_viewing_space_present_flag=false
msp_extension_present_flag=false
)");

    SECTION("Coding") {
      auto vp = ViewParams{};
      vp.size = {1, 1};

      x.vps_atlas_count(1).vps_frame_width(0, 1920).vps_frame_height(0, 1080).vps_map_count(0, 1);
      x.occupancy_information(0).oi_occupancy_nominal_2d_bitdepth(1);
      x.geometry_information(0).gi_geometry_nominal_2d_bitdepth(1);
      x.geometry_information(0).gi_geometry_3d_coordinates_bitdepth(1);
      x.miv_sequence_params().view_params_list(ViewParamsList{{vp}});
      x.updateOverridePduProjectionIdNumBits();

      REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
vps_vpcc_parameter_set_id=0
vps_atlas_count=1
vps_frame_width( 0 )=1920
vps_frame_height( 0 )=1080
vps_map_count( 0 )=1
vps_raw_patch_enabled_flag( 0 )=false
oi_occupancy_codec_id( 0 )=0
oi_lossy_occupancy_map_compression_threshold( 0 )=0
oi_occupancy_nominal_2d_bitdepth( 0 )=1
oi_occupancy_MSB_align_flag( 0 )=false
gi_geometry_codec_id( 0 )=0
gi_geometry_nominal_2d_bitdepth( 0 )=1
gi_geometry_MSB_align_flag( 0 )=false
gi_geometry_3d_coordinates_bitdepth( 0 )=1
ai_attribute_count( 0 )=0
vps_extension_present_flag=true
overridePduProjectionIdNumBits=0
miv_sequence_params_present_flag=vps_extension_present_flag
msp_profile_idc=Basic
msp_depth_params_num_bits=10
View  0: [1, 1], ERP [0, 0] x [0, 0] deg, norm. disp in [0, 0] m^-1, hasOccupancy false, depthOccMapThreshold 0, pose [ 0.000,  0.000,  0.000] m, [0, 0, 0] deg
msp_depth_low_quality_flag=false
msp_num_groups=1
msp_max_entities=1
msp_viewing_space_present_flag=false
msp_extension_present_flag=false
)");

      REQUIRE(byteCodingTest(x, 80));
    }
  }
}