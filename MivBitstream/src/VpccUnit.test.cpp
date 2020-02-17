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

#include <TMIV/MivBitstream/VpccUnit.h>

using namespace TMIV::MivBitstream;

namespace examples {
auto vps() {
  auto x = VpccParameterSet{};
  x.vps_miv_mode_flag(true);
  x.vps_frame_width(0, 640);
  x.vps_frame_height(0, 480);
  x.geometry_information(0).gi_geometry_nominal_2d_bitdepth_minus1(8);
  x.geometry_information(0).gi_geometry_3d_coordinates_bitdepth(11);
  x.vps_extension_present_flag(true);
  return x;
}

auto vpses() {
  auto x = VpccParameterSet{};
  x.vps_atlas_count_minus1(2);
  for (int j = 0; j <= x.vps_atlas_count_minus1(); ++j) {
    x.vps_frame_width(j, 640);
    x.vps_frame_height(j, 480);
    x.occupancy_information(j).oi_occupancy_nominal_2d_bitdepth_minus1(7);
    x.geometry_information(j).gi_geometry_nominal_2d_bitdepth_minus1(8);
    x.geometry_information(j).gi_geometry_3d_coordinates_bitdepth(11);
    x.attribute_information(j).ai_attribute_count(4);
  }
  return std::vector<VpccParameterSet>{3, x};
}
} // namespace examples

TEST_CASE("vpcc_unit_header", "[VPCC Unit]") {
  SECTION("VPS") {
    const auto x = VpccUnitHeader{VuhUnitType::VPCC_VPS};
    const auto vpses = std::vector<VpccParameterSet>{};

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_VPS
)");

    REQUIRE(byteCodingTest(x, 4, vpses));
  }

  SECTION("AD") {
    auto x = VpccUnitHeader{VuhUnitType::VPCC_AD};
    const auto vpses = examples::vpses();

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_AD
vuh_vpcc_parameter_set_id=0
vuh_atlas_id=0
)");

    REQUIRE(byteCodingTest(x, 4, vpses));

    SECTION("Example") {
      x.vuh_vpcc_parameter_set_id(1).vuh_atlas_id(2);

      REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_AD
vuh_vpcc_parameter_set_id=1
vuh_atlas_id=2
)");

      REQUIRE(byteCodingTest(x, 4, vpses));
    }
  }

  SECTION("OVD") {
    auto x = VpccUnitHeader{VuhUnitType::VPCC_OVD};
    const auto vpses = examples::vpses();

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_OVD
vuh_vpcc_parameter_set_id=0
vuh_atlas_id=0
)");

    REQUIRE(byteCodingTest(x, 4, vpses));

    SECTION("Example") {
      x.vuh_vpcc_parameter_set_id(2).vuh_atlas_id(1);

      REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_OVD
vuh_vpcc_parameter_set_id=2
vuh_atlas_id=1
)");

      REQUIRE(byteCodingTest(x, 4, vpses));
    }
  }

  SECTION("GVD") {
    auto x = VpccUnitHeader{VuhUnitType::VPCC_GVD};
    const auto vpses = examples::vpses();

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_GVD
vuh_vpcc_parameter_set_id=0
vuh_atlas_id=0
vuh_map_index=0
vuh_raw_video_flag=false
)");

    REQUIRE(byteCodingTest(x, 4, vpses));

    SECTION("Example") {
      x.vuh_vpcc_parameter_set_id(2).vuh_atlas_id(0).vuh_map_index(0).vuh_raw_video_flag(false);

      REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_GVD
vuh_vpcc_parameter_set_id=2
vuh_atlas_id=0
vuh_map_index=0
vuh_raw_video_flag=false
)");

      REQUIRE(byteCodingTest(x, 4, vpses));
    }
  }

  SECTION("AVD") {
    auto x = VpccUnitHeader{VuhUnitType::VPCC_AVD};
    const auto vpses = examples::vpses();

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_AVD
vuh_vpcc_parameter_set_id=0
vuh_atlas_id=0
vuh_attribute_index=0
vuh_attribute_dimension_index=0
vuh_map_index=0
vuh_raw_video_flag=false
)");

    REQUIRE(byteCodingTest(x, 4, vpses));

    SECTION("Example") {
      x.vuh_vpcc_parameter_set_id(2)
          .vuh_atlas_id(2)
          .vuh_attribute_index(3)
          .vuh_attribute_dimension_index(0)
          .vuh_map_index(0)
          .vuh_raw_video_flag(false);

      REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_AVD
vuh_vpcc_parameter_set_id=2
vuh_atlas_id=2
vuh_attribute_index=3
vuh_attribute_dimension_index=0
vuh_map_index=0
vuh_raw_video_flag=false
)");

      REQUIRE(byteCodingTest(x, 4, vpses));
    }
  }
}

TEST_CASE("vpcc_unit_payload", "[VPCC Unit]") {
  SECTION("VPS") {
    const auto vuh = VpccUnitHeader{VuhUnitType::VPCC_VPS};
    const auto x = VpccPayload{examples::vps()};

    REQUIRE(toString(x) == R"(ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
vps_vpcc_parameter_set_id=0
vps_miv_mode_flag=true
vps_atlas_count_minus1=0
vps_frame_width( 0 )=640
vps_frame_height( 0 )=480
vps_map_count_minus1( 0 )=0
vps_auxiliary_video_present_flag( 0 )=false
gi_geometry_codec_id( 0 )=0
gi_geometry_nominal_2d_bitdepth_minus1( 0 )=8
gi_geometry_MSB_align_flag( 0 )=false
gi_geometry_3d_coordinates_bitdepth( 0 )=11
ai_attribute_count( 0 )=0
vps_extension_present_flag=true
vps_miv_extension_flag=false
)");

    REQUIRE(byteCodingTest(x, 20, vuh));
  }

  SECTION("AD") {
    const auto vuh = VpccUnitHeader{VuhUnitType::VPCC_AD};
    const auto x = VpccPayload{AtlasSubBitstream{SampleStreamNalHeader{5}}};

    REQUIRE(toString(x) == R"(ssnh_unit_size_precision_bytes=5
)");

    REQUIRE(byteCodingTest(x, 1, vuh));
  }

  SECTION("OVD") {
    const auto vuh = VpccUnitHeader{VuhUnitType::VPCC_OVD};
    const auto x = VpccPayload{VideoSubBitstream{}};

    REQUIRE(toString(x).empty());

    REQUIRE(byteCodingTest(x, 0, vuh));
  }

  SECTION("GVD") {
    const auto vuh = VpccUnitHeader{VuhUnitType::VPCC_GVD};
    const auto x = VpccPayload{VideoSubBitstream{}};

    REQUIRE(toString(x).empty());

    REQUIRE(byteCodingTest(x, 0, vuh));
  }

  SECTION("AVD") {
    const auto vuh = VpccUnitHeader{VuhUnitType::VPCC_AVD};
    const auto x = VpccPayload{VideoSubBitstream{}};

    REQUIRE(toString(x).empty());

    REQUIRE(byteCodingTest(x, 0, vuh));
  }
}

TEST_CASE("vpcc_unit", "[VPCC Unit]") {
  SECTION("Example 1") {
    const auto vpses = std::vector<VpccParameterSet>{};
    const auto vps = examples::vps();
    const auto x = VpccUnit{VpccUnitHeader{VuhUnitType::VPCC_VPS}, vps};

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_VPS
ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_pcc_toolset_idc=Basic
ptl_profile_reconstruction_idc=Rec0
ptl_level_idc=[unknown:0]
vps_vpcc_parameter_set_id=0
vps_miv_mode_flag=true
vps_atlas_count_minus1=0
vps_frame_width( 0 )=640
vps_frame_height( 0 )=480
vps_map_count_minus1( 0 )=0
vps_auxiliary_video_present_flag( 0 )=false
gi_geometry_codec_id( 0 )=0
gi_geometry_nominal_2d_bitdepth_minus1( 0 )=8
gi_geometry_MSB_align_flag( 0 )=false
gi_geometry_3d_coordinates_bitdepth( 0 )=11
ai_attribute_count( 0 )=0
vps_extension_present_flag=true
vps_miv_extension_flag=false
)");

    REQUIRE(unitCodingTest(x, 24, vpses));
  }

  SECTION("Example 2") {
    const auto vpses = examples::vpses();
    auto vuh = VpccUnitHeader{VuhUnitType::VPCC_AVD};
    vuh.vuh_vpcc_parameter_set_id(2)
        .vuh_atlas_id(1)
        .vuh_attribute_index(2)
        .vuh_attribute_dimension_index(0)
        .vuh_map_index(0)
        .vuh_raw_video_flag(false);

    const auto x = VpccUnit{vuh, VideoSubBitstream{}};

    REQUIRE(toString(x) == R"(vuh_unit_type=VPCC_AVD
vuh_vpcc_parameter_set_id=2
vuh_atlas_id=1
vuh_attribute_index=2
vuh_attribute_dimension_index=0
vuh_map_index=0
vuh_raw_video_flag=false
)");

    REQUIRE(unitCodingTest(x, 4, vpses));
  }
}
