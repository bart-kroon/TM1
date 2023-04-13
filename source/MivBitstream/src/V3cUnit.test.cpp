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

#include <TMIV/MivBitstream/V3cUnit.h>

namespace TMIV::MivBitstream {
namespace examples {
auto vps() {
  auto x = V3cParameterSet{};
  x.vps_atlas_id(0, {});
  x.vps_frame_width({}, 640);
  x.vps_frame_height({}, 480);
  x.vps_geometry_video_present_flag({}, true);
  x.geometry_information({}).gi_geometry_2d_bit_depth_minus1(8);
  x.geometry_information({}).gi_geometry_3d_coordinates_bit_depth_minus1(10);
  x.calculateExtensionLengths();
  return x;
}
} // namespace examples

TEST_CASE("v3c_unit_header", "[V3C Unit]") {
  SECTION("VPS") {
    const auto x = V3cUnitHeader::vps();

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_VPS
)");

    byteCodingTest(x, 4);
    REQUIRE(x.summary() == "V3C_VPS");
  }

  SECTION("AD") {
    auto x = V3cUnitHeader::ad(0, {});

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
)");

    byteCodingTest(x, 4);
    REQUIRE(x.summary() == "V3C_AD vps:0 atlas:0");

    SECTION("Example") {
      x = V3cUnitHeader::ad(1, AtlasId{2});

      REQUIRE(toString(x) == R"(vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=1
vuh_atlas_id=2
)");

      byteCodingTest(x, 4);
      REQUIRE(x.summary() == "V3C_AD vps:1 atlas:2");
    }
  }

  SECTION("OVD") {
    auto x = V3cUnitHeader::ovd(0, {});

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_OVD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
)");

    byteCodingTest(x, 4);

    SECTION("Example") {
      x = V3cUnitHeader::ovd(2, AtlasId{1});

      REQUIRE(toString(x) == R"(vuh_unit_type=V3C_OVD
vuh_v3c_parameter_set_id=2
vuh_atlas_id=1
)");

      byteCodingTest(x, 4);
      REQUIRE(x.summary() == "V3C_OVD vps:2 atlas:1");
    }
  }

  SECTION("GVD") {
    auto x = V3cUnitHeader::gvd(0, {});

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_GVD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
vuh_map_index=0
vuh_auxiliary_video_flag=false
)");

    byteCodingTest(x, 4);

    SECTION("Example") {
      x = V3cUnitHeader::gvd(2, {}, 0, false);

      REQUIRE(toString(x) == R"(vuh_unit_type=V3C_GVD
vuh_v3c_parameter_set_id=2
vuh_atlas_id=0
vuh_map_index=0
vuh_auxiliary_video_flag=false
)");

      byteCodingTest(x, 4);
      REQUIRE(x.summary() == "V3C_GVD vps:2 atlas:0 map:0 aux:false");
    }
  }

  SECTION("AVD") {
    auto x = V3cUnitHeader::avd(0, {}, 0);

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_AVD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
vuh_attribute_index=0
vuh_attribute_partition_index=0
vuh_map_index=0
vuh_auxiliary_video_flag=false
)");

    byteCodingTest(x, 4);
    REQUIRE(x.summary() == "V3C_AVD vps:0 atlas:0 attr:0 part:0 map:0 aux:false");

    SECTION("Example") {
      x = V3cUnitHeader::avd(2, AtlasId{2}, 3, 0, 0, false);

      REQUIRE(toString(x) == R"(vuh_unit_type=V3C_AVD
vuh_v3c_parameter_set_id=2
vuh_atlas_id=2
vuh_attribute_index=3
vuh_attribute_partition_index=0
vuh_map_index=0
vuh_auxiliary_video_flag=false
)");

      byteCodingTest(x, 4);
      REQUIRE(x.summary() == "V3C_AVD vps:2 atlas:2 attr:3 part:0 map:0 aux:false");
    }
  }

  SECTION("CAD") {
    auto unit = V3cUnitHeader::cad(0);
    REQUIRE(toString(unit) == R"(vuh_unit_type=V3C_CAD
vuh_v3c_parameter_set_id=0
)");

    byteCodingTest(unit, 4);
    REQUIRE(unit.summary() == "V3C_CAD vps:0");

    SECTION("Example") {
      unit = V3cUnitHeader::cad(2);

      REQUIRE(toString(unit) == R"(vuh_unit_type=V3C_CAD
vuh_v3c_parameter_set_id=2
)");

      byteCodingTest(unit, 4);
      REQUIRE(unit.summary() == "V3C_CAD vps:2");
    }
  }

  SECTION("PVD") {
    auto unit = V3cUnitHeader::pvd(0, {});
    REQUIRE(toString(unit) == R"(vuh_unit_type=V3C_PVD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
)");

    byteCodingTest(unit, 4);
    REQUIRE(unit.summary() == "V3C_PVD vps:0 atlas:0");

    SECTION("Example") {
      unit = V3cUnitHeader::pvd(2, {});

      REQUIRE(toString(unit) == R"(vuh_unit_type=V3C_PVD
vuh_v3c_parameter_set_id=2
vuh_atlas_id=0
)");

      byteCodingTest(unit, 4);
      REQUIRE(unit.summary() == "V3C_PVD vps:2 atlas:0");
    }
  }
}

TEST_CASE("v3c_unit_payload", "[V3C Unit]") {
  SECTION("VPS") {
    const auto vuh = V3cUnitHeader::vps();
    const auto x = V3cUnitPayload{examples::vps()};

    REQUIRE(toString(x) == R"(ptl_tier_flag=false
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
vps_frame_width[ 0 ]=640
vps_frame_height[ 0 ]=480
vps_map_count_minus1[ 0 ]=0
vps_map_absolute_coding_enabled_flag[ 0 ][ 0 ]=true
vps_map_predictor_index_diff[ 0 ][ 0 ]=0
vps_auxiliary_video_present_flag[ 0 ]=false
vps_occupancy_video_present_flag[ 0 ]=false
vps_geometry_video_present_flag[ 0 ]=true
vps_attribute_video_present_flag[ 0 ]=false
gi_geometry_codec_id[ 0 ]=0
gi_geometry_2d_bit_depth_minus1[ 0 ]=8
gi_geometry_MSB_align_flag[ 0 ]=false
gi_geometry_3d_coordinates_bit_depth_minus1[ 0 ]=10
vps_extension_present_flag=false
)");

    byteCodingTest(x, 21, vuh);
  }

  SECTION("AD") {
    const auto vuh = V3cUnitHeader::ad(0, {});
    const auto x = V3cUnitPayload{AtlasSubBitstream{SampleStreamNalHeader{4}}};

    REQUIRE(toString(x) == R"(ssnh_unit_size_precision_bytes_minus1=4
)");

    byteCodingTest(x, 1, vuh);
  }

  SECTION("OVD") {
    const auto vuh = V3cUnitHeader::ovd(0, {});
    const auto x = V3cUnitPayload{VideoSubBitstream{}};

    REQUIRE(toString(x).empty());

    byteCodingTest(x, 0, vuh);
  }

  SECTION("GVD") {
    const auto vuh = V3cUnitHeader::gvd(0, {});
    const auto x = V3cUnitPayload{VideoSubBitstream{}};

    REQUIRE(toString(x).empty());

    byteCodingTest(x, 0, vuh);
  }

  SECTION("AVD") {
    const auto vuh = V3cUnitHeader::avd(0, {}, 0);
    const auto x = V3cUnitPayload{VideoSubBitstream{}};

    REQUIRE(toString(x).empty());

    byteCodingTest(x, 0, vuh);
  }
}

TEST_CASE("v3c_unit", "[V3C Unit]") {
  SECTION("Example 1") {
    const auto vps = examples::vps();
    const auto x = V3cUnit{V3cUnitHeader::vps(), vps};

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_VPS
ptl_tier_flag=false
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
vps_frame_width[ 0 ]=640
vps_frame_height[ 0 ]=480
vps_map_count_minus1[ 0 ]=0
vps_map_absolute_coding_enabled_flag[ 0 ][ 0 ]=true
vps_map_predictor_index_diff[ 0 ][ 0 ]=0
vps_auxiliary_video_present_flag[ 0 ]=false
vps_occupancy_video_present_flag[ 0 ]=false
vps_geometry_video_present_flag[ 0 ]=true
vps_attribute_video_present_flag[ 0 ]=false
gi_geometry_codec_id[ 0 ]=0
gi_geometry_2d_bit_depth_minus1[ 0 ]=8
gi_geometry_MSB_align_flag[ 0 ]=false
gi_geometry_3d_coordinates_bit_depth_minus1[ 0 ]=10
vps_extension_present_flag=false
)");

    REQUIRE(unitCodingTest(x, 25));
  }

  SECTION("Example 2") {
    auto vuh = V3cUnitHeader::avd(2, AtlasId{1}, 2, 0, 0, false);

    const auto x = V3cUnit{vuh, VideoSubBitstream{}};

    REQUIRE(toString(x) == R"(vuh_unit_type=V3C_AVD
vuh_v3c_parameter_set_id=2
vuh_atlas_id=1
vuh_attribute_index=2
vuh_attribute_partition_index=0
vuh_map_index=0
vuh_auxiliary_video_flag=false
)");

    REQUIRE(unitCodingTest(x, 4));
  }
}
} // namespace TMIV::MivBitstream
