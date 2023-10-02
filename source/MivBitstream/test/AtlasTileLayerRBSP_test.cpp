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

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("atlas_tile_header", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  aspsV.front().asps_num_ref_atlas_frame_lists_in_asps(1).asps_log2_patch_packing_block_size(7);

  auto x = AtlasTileHeader{};

  REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=P_TILE
ath_atlas_frm_order_cnt_lsb=0
)");

  SECTION("Example 1") {
    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    x.ath_no_output_of_prior_atlas_frames_flag(true)
        .ath_type(AthType::SKIP_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 1};

    REQUIRE(toString(x) == R"(ath_no_output_of_prior_atlas_frames_flag=true
ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=SKIP_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
)");

    bitCodingTest(x, 16, nuh, aspsV, afpsV);
  }

  SECTION("Example 2") {
    aspsV.front()
        .asps_patch_size_quantizer_present_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true);

    const auto afpsV = std::vector{AtlasFrameParameterSetRBSP{}.atlas_frame_tile_information(
        AtlasFrameTileInformation{}
            .afti_single_tile_in_atlas_frame_flag(false)
            .afti_uniform_partition_spacing_flag(false)
            .afti_num_partition_columns_minus1(7)
            .afti_num_partition_rows_minus1(5)
            .afti_num_tiles_in_atlas_frame_minus1(47))};

    x.ath_id(46)
        .ath_type(AthType::I_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true)
        .ath_patch_size_x_info_quantizer(6)
        .ath_patch_size_y_info_quantizer(5)
        .ath_atlas_adaptation_parameter_set_id(63)
        .ath_pos_min_d_quantizer(3)
        .ath_pos_delta_max_d_quantizer(7);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_TSA_R, 0, 2};

    REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=63
ath_id=46
ath_type=I_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
ath_pos_min_d_quantizer=3
ath_pos_delta_max_d_quantizer=7
ath_patch_size_x_info_quantizer=6
ath_patch_size_y_info_quantizer=5
)");

    bitCodingTest(x, 48, nuh, aspsV, afpsV);
  }
}

TEST_CASE("skip_patch_data_unit", "[Atlas Tile Layer RBSP]") {
  auto x = SkipPatchDataUnit{};
  REQUIRE(toString(x).empty());
  bitCodingTest(x, 0);
}

TEST_CASE("patch_data_unit", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);
  const auto ath = AtlasTileHeader{};

  auto x = PatchDataUnit{};

  REQUIRE(toString(x, 0, 101) == R"(pdu_2d_pos_x[ 0 ][ 101 ]=0
pdu_2d_pos_y[ 0 ][ 101 ]=0
pdu_2d_size_x_minus1[ 0 ][ 101 ]=0
pdu_2d_size_y_minus1[ 0 ][ 101 ]=0
pdu_3d_offset_u[ 0 ][ 101 ]=0
pdu_3d_offset_v[ 0 ][ 101 ]=0
pdu_3d_offset_d[ 0 ][ 101 ]=0
pdu_projection_id[ 0 ][ 101 ]=0
pdu_orientation_index[ 0 ][ 101 ]=FPO_NULL
)");

  bitCodingTest(x, 11, aspsV, afpsV, ath);

  SECTION("Minimal MIV extension (no change)") {
    aspsV.front().asps_miv_extension() = {};
    x.pdu_miv_extension() = {};

    REQUIRE(toString(x, 0, 101) == R"(pdu_2d_pos_x[ 0 ][ 101 ]=0
pdu_2d_pos_y[ 0 ][ 101 ]=0
pdu_2d_size_x_minus1[ 0 ][ 101 ]=0
pdu_2d_size_y_minus1[ 0 ][ 101 ]=0
pdu_3d_offset_u[ 0 ][ 101 ]=0
pdu_3d_offset_v[ 0 ][ 101 ]=0
pdu_3d_offset_d[ 0 ][ 101 ]=0
pdu_projection_id[ 0 ][ 101 ]=0
pdu_orientation_index[ 0 ][ 101 ]=FPO_NULL
)");

    bitCodingTest(x, 11, aspsV, afpsV, ath);
  }

  SECTION("Example 1") {
    aspsV.front()
        .asps_geometry_3d_bit_depth_minus1(14)
        .asps_geometry_2d_bit_depth_minus1(9)
        .asps_use_eight_orientations_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_max_number_projections_minus1(511)
        .asps_miv_extension()
        .asme_embedded_occupancy_enabled_flag(true)
        .asme_depth_occ_threshold_flag(true)
        .asme_max_entity_id(100)
        .asme_patch_texture_offset_bit_depth_minus1(5)
        .asme_inpaint_enabled_flag(true);

    x.pdu_2d_pos_x(34)
        .pdu_2d_pos_y(57)
        .pdu_2d_size_x_minus1(1234)
        .pdu_2d_size_y_minus1(1002)
        .pdu_3d_offset_u(1234)
        .pdu_3d_offset_v(21345)
        .pdu_3d_offset_d(623)
        .pdu_3d_range_d(789)
        .pdu_projection_id(ViewId{300})
        .pdu_orientation_index(FlexiblePatchOrientation::FPO_MROT180)
        .pdu_miv_extension()
        .pdu_entity_id(35)
        .pdu_depth_occ_threshold(600)
        .pdu_texture_offset(0, 4)
        .pdu_texture_offset(1, 5)
        .pdu_texture_offset(2, 6)
        .pdu_inpaint_flag(false);

    REQUIRE(toString(x, 12, 102) == R"(pdu_2d_pos_x[ 12 ][ 102 ]=34
pdu_2d_pos_y[ 12 ][ 102 ]=57
pdu_2d_size_x_minus1[ 12 ][ 102 ]=1234
pdu_2d_size_y_minus1[ 12 ][ 102 ]=1002
pdu_3d_offset_u[ 12 ][ 102 ]=1234
pdu_3d_offset_v[ 12 ][ 102 ]=21345
pdu_3d_offset_d[ 12 ][ 102 ]=623
pdu_3d_range_d[ 12 ][ 102 ]=789
pdu_projection_id[ 12 ][ 102 ]=300
pdu_orientation_index[ 12 ][ 102 ]=FPO_MROT180
pdu_entity_id[ 12 ][ 102 ]=35
pdu_depth_occ_threshold[ 12 ][ 102 ]=600
pdu_texture_offset[ 12 ][ 102 ][ 0 ]=4
pdu_texture_offset[ 12 ][ 102 ][ 1 ]=5
pdu_texture_offset[ 12 ][ 102 ][ 2 ]=6
pdu_inpaint_flag[ 12 ][ 102 ]=false
)");

    bitCodingTest(x, 165, aspsV, afpsV, ath);
  }

  SECTION("Extend with only pdu_entity_id") {
    aspsV.front()
        .asps_geometry_2d_bit_depth_minus1(9)
        .asps_geometry_3d_bit_depth_minus1(14)
        .asps_use_eight_orientations_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_max_number_projections_minus1(511)
        .asps_miv_extension()
        .asme_max_entity_id(100);

    // Create ASME with default values
    static_cast<void>(aspsV.front().asps_miv_extension());

    x.pdu_2d_pos_x(34)
        .pdu_2d_pos_y(57)
        .pdu_2d_size_x_minus1(1234)
        .pdu_2d_size_y_minus1(1002)
        .pdu_3d_offset_u(1234)
        .pdu_3d_offset_v(21345)
        .pdu_3d_offset_d(623)
        .pdu_3d_range_d(789)
        .pdu_projection_id(ViewId{300})
        .pdu_orientation_index(FlexiblePatchOrientation::FPO_MROT180)
        .pdu_miv_extension()
        .pdu_entity_id(35);

    REQUIRE(toString(x, 12, 102) == R"(pdu_2d_pos_x[ 12 ][ 102 ]=34
pdu_2d_pos_y[ 12 ][ 102 ]=57
pdu_2d_size_x_minus1[ 12 ][ 102 ]=1234
pdu_2d_size_y_minus1[ 12 ][ 102 ]=1002
pdu_3d_offset_u[ 12 ][ 102 ]=1234
pdu_3d_offset_v[ 12 ][ 102 ]=21345
pdu_3d_offset_d[ 12 ][ 102 ]=623
pdu_3d_range_d[ 12 ][ 102 ]=789
pdu_projection_id[ 12 ][ 102 ]=300
pdu_orientation_index[ 12 ][ 102 ]=FPO_MROT180
pdu_entity_id[ 12 ][ 102 ]=35
)");

    bitCodingTest(x, 136, aspsV, afpsV, ath);
  }
}

TEST_CASE("inter_patch_data_unit", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);
  const auto ath = AtlasTileHeader{};

  auto x = InterPatchDataUnit{};

  SECTION("Default initialization") {
    REQUIRE(toString(x, 8, 3) == R"(ipdu_patch_index[ 8 ][ 3 ]=0
ipdu_2d_pos_x[ 8 ][ 3 ]=0
ipdu_2d_pos_y[ 8 ][ 3 ]=0
ipdu_2d_delta_size_x[ 8 ][ 3 ]=0
ipdu_2d_delta_size_y[ 8 ][ 3 ]=0
ipdu_3d_offset_u[ 8 ][ 3 ]=0
ipdu_3d_offset_v[ 8 ][ 3 ]=0
ipdu_3d_offset_d[ 8 ][ 3 ]=0
)");

    bitCodingTest(x, 8, aspsV, afpsV, ath);
  }

  SECTION("Set values") {
    x.ipdu_patch_index(2)
        .ipdu_2d_pos_x(-3)
        .ipdu_2d_pos_y(4)
        .ipdu_2d_delta_size_x(-5)
        .ipdu_2d_delta_size_y(6)
        .ipdu_3d_offset_u(7)
        .ipdu_3d_offset_v(-8)
        .ipdu_3d_offset_d(-1);

    REQUIRE(toString(x, 3, 1) == R"(ipdu_patch_index[ 3 ][ 1 ]=2
ipdu_2d_pos_x[ 3 ][ 1 ]=-3
ipdu_2d_pos_y[ 3 ][ 1 ]=4
ipdu_2d_delta_size_x[ 3 ][ 1 ]=-5
ipdu_2d_delta_size_y[ 3 ][ 1 ]=6
ipdu_3d_offset_u[ 3 ][ 1 ]=7
ipdu_3d_offset_v[ 3 ][ 1 ]=-8
ipdu_3d_offset_d[ 3 ][ 1 ]=-1
)");

    bitCodingTest(x, 50, aspsV, afpsV, ath);
  }

  SECTION("ipdu_ref_index") {
    CHECK(x.ipdu_ref_index() == 0);
    x.ipdu_ref_index(45);

    REQUIRE(toString(x, 0, 0) == R"(ipdu_ref_index[ 0 ][ 0 ]=45
ipdu_patch_index[ 0 ][ 0 ]=0
ipdu_2d_pos_x[ 0 ][ 0 ]=0
ipdu_2d_pos_y[ 0 ][ 0 ]=0
ipdu_2d_delta_size_x[ 0 ][ 0 ]=0
ipdu_2d_delta_size_y[ 0 ][ 0 ]=0
ipdu_3d_offset_u[ 0 ][ 0 ]=0
ipdu_3d_offset_v[ 0 ][ 0 ]=0
ipdu_3d_offset_d[ 0 ][ 0 ]=0
)");
  }

  SECTION("ipdu_3d_range_d") {
    aspsV.front().asps_normal_axis_max_delta_value_enabled_flag(true);
    x.ipdu_3d_range_d(43);

    REQUIRE(toString(x, 9, 4) == R"(ipdu_patch_index[ 9 ][ 4 ]=0
ipdu_2d_pos_x[ 9 ][ 4 ]=0
ipdu_2d_pos_y[ 9 ][ 4 ]=0
ipdu_2d_delta_size_x[ 9 ][ 4 ]=0
ipdu_2d_delta_size_y[ 9 ][ 4 ]=0
ipdu_3d_offset_u[ 9 ][ 4 ]=0
ipdu_3d_offset_v[ 9 ][ 4 ]=0
ipdu_3d_offset_d[ 9 ][ 4 ]=0
ipdu_3d_range_d[ 9 ][ 4 ]=43
)");

    bitCodingTest(x, 21, aspsV, afpsV, ath);
  }
}

TEST_CASE("patch_information_data", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  aspsV.front().asps_frame_width(4000).asps_frame_height(2000);

  const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

  const auto pdu = PatchDataUnit{};

  auto x = PatchInformationData{};

  REQUIRE(toString(x, 0, 77).empty());

  SECTION("Example skip_patch_data_unit") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::P_TILE);
    const auto patchMode = AtduPatchMode::P_SKIP;
    x.skip_patch_data_unit() = {};

    REQUIRE(toString(x, 77, 88).empty());
    bitCodingTest(x, 0, aspsV, afpsV, ath, patchMode);
  }

  SECTION("Example patch_data_unit") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::I_TILE);
    const auto patchMode = AtduPatchMode::I_INTRA;
    x.patch_data_unit() = pdu;

    REQUIRE(toString(x, 13, 99) == R"(pdu_2d_pos_x[ 13 ][ 99 ]=0
pdu_2d_pos_y[ 13 ][ 99 ]=0
pdu_2d_size_x_minus1[ 13 ][ 99 ]=0
pdu_2d_size_y_minus1[ 13 ][ 99 ]=0
pdu_3d_offset_u[ 13 ][ 99 ]=0
pdu_3d_offset_v[ 13 ][ 99 ]=0
pdu_3d_offset_d[ 13 ][ 99 ]=0
pdu_projection_id[ 13 ][ 99 ]=0
pdu_orientation_index[ 13 ][ 99 ]=FPO_NULL
)");
    bitCodingTest(x, 11, aspsV, afpsV, ath, patchMode);
  }
}

TEST_CASE("atlas_tile_data_unit", "[Atlas Tile Layer RBSP]") {
  SECTION("Empty") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::I_TILE);

    const auto x = AtlasTileDataUnit{}.atdu_patch_mode(0, AtduPatchMode::I_END);
    REQUIRE(toString(x, ath) == R"(atdu_patch_mode[ 0 ][ 0 ]=I_END
AtduTotalNumPatches[ 0 ]=0
)");

    const auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    bitCodingTest(x, 7, aspsV, afpsV, ath);
  }

  SECTION("P_TILE") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::P_TILE);

    const auto x = []() {
      auto atdu = AtlasTileDataUnit{};
      atdu.atdu_patch_mode(0, AtduPatchMode::P_SKIP);
      atdu.atdu_patch_mode(1, AtduPatchMode::P_INTER);
      atdu.atdu_patch_mode(2, AtduPatchMode::P_INTRA);
      atdu.atdu_patch_mode(3, AtduPatchMode::P_SKIP);
      atdu.atdu_patch_mode(4, AtduPatchMode::P_END);

      atdu.patch_information_data(0).skip_patch_data_unit() = {};
      atdu.patch_information_data(1).inter_patch_data_unit() = {};
      atdu.patch_information_data(2).patch_data_unit() = {};
      atdu.patch_information_data(3).skip_patch_data_unit() = {};

      return atdu;
    }();

    REQUIRE(toString(x, ath) == R"(atdu_patch_mode[ 0 ][ 0 ]=P_SKIP
atdu_patch_mode[ 0 ][ 1 ]=P_INTER
ipdu_patch_index[ 0 ][ 1 ]=0
ipdu_2d_pos_x[ 0 ][ 1 ]=0
ipdu_2d_pos_y[ 0 ][ 1 ]=0
ipdu_2d_delta_size_x[ 0 ][ 1 ]=0
ipdu_2d_delta_size_y[ 0 ][ 1 ]=0
ipdu_3d_offset_u[ 0 ][ 1 ]=0
ipdu_3d_offset_v[ 0 ][ 1 ]=0
ipdu_3d_offset_d[ 0 ][ 1 ]=0
atdu_patch_mode[ 0 ][ 2 ]=P_INTRA
pdu_2d_pos_x[ 0 ][ 2 ]=0
pdu_2d_pos_y[ 0 ][ 2 ]=0
pdu_2d_size_x_minus1[ 0 ][ 2 ]=0
pdu_2d_size_y_minus1[ 0 ][ 2 ]=0
pdu_3d_offset_u[ 0 ][ 2 ]=0
pdu_3d_offset_v[ 0 ][ 2 ]=0
pdu_3d_offset_d[ 0 ][ 2 ]=0
pdu_projection_id[ 0 ][ 2 ]=0
pdu_orientation_index[ 0 ][ 2 ]=FPO_NULL
atdu_patch_mode[ 0 ][ 3 ]=P_SKIP
atdu_patch_mode[ 0 ][ 4 ]=P_END
AtduTotalNumPatches[ 0 ]=4
)");

    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front().asps_frame_width(4000).asps_frame_height(2000);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    bitCodingTest(x, 36, aspsV, afpsV, ath);
  }

  SECTION("I_TILE") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::I_TILE).ath_id(7);

    const auto x = []() {
      auto atdu = AtlasTileDataUnit{};

      atdu.atdu_patch_mode(0, AtduPatchMode::I_INTRA);
      atdu.atdu_patch_mode(1, AtduPatchMode::I_INTRA);
      atdu.atdu_patch_mode(2, AtduPatchMode::I_END);

      atdu.patch_information_data(0).patch_data_unit() = {};
      atdu.patch_information_data(1).patch_data_unit() = {};

      return atdu;
    }();

    REQUIRE(toString(x, ath) == R"(atdu_patch_mode[ 7 ][ 0 ]=I_INTRA
pdu_2d_pos_x[ 7 ][ 0 ]=0
pdu_2d_pos_y[ 7 ][ 0 ]=0
pdu_2d_size_x_minus1[ 7 ][ 0 ]=0
pdu_2d_size_y_minus1[ 7 ][ 0 ]=0
pdu_3d_offset_u[ 7 ][ 0 ]=0
pdu_3d_offset_v[ 7 ][ 0 ]=0
pdu_3d_offset_d[ 7 ][ 0 ]=0
pdu_projection_id[ 7 ][ 0 ]=0
pdu_orientation_index[ 7 ][ 0 ]=FPO_NULL
atdu_patch_mode[ 7 ][ 1 ]=I_INTRA
pdu_2d_pos_x[ 7 ][ 1 ]=0
pdu_2d_pos_y[ 7 ][ 1 ]=0
pdu_2d_size_x_minus1[ 7 ][ 1 ]=0
pdu_2d_size_y_minus1[ 7 ][ 1 ]=0
pdu_3d_offset_u[ 7 ][ 1 ]=0
pdu_3d_offset_v[ 7 ][ 1 ]=0
pdu_3d_offset_d[ 7 ][ 1 ]=0
pdu_projection_id[ 7 ][ 1 ]=0
pdu_orientation_index[ 7 ][ 1 ]=FPO_NULL
atdu_patch_mode[ 7 ][ 2 ]=I_END
AtduTotalNumPatches[ 7 ]=2
)");

    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front().asps_frame_width(4000).asps_frame_height(2000);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    bitCodingTest(x, 31, aspsV, afpsV, ath);
  }
}

TEST_CASE("atlas_tile_layer_rbsp", "[Atlas Tile Layer RBSP]") {
  SECTION("SKIP_TILE") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front()
        .asps_frame_width(4000)
        .asps_frame_height(2000)
        .asps_num_ref_atlas_frame_lists_in_asps(1);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_TSA_R, 0, 2};

    auto x = AtlasTileLayerRBSP{};
    x.atlas_tile_header().ath_type(AthType::SKIP_TILE).ath_ref_atlas_frame_list_asps_flag(true);
    x.atlas_tile_data_unit().skip_patch_data_unit() = {};

    REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=SKIP_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
)");
    byteCodingTest(x, 3, nuh, aspsV, afpsV);
  }

  SECTION("I_TILE") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front()
        .asps_frame_width(4000)
        .asps_frame_height(2000)
        .asps_num_ref_atlas_frame_lists_in_asps(1);

    auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);
    afpsV.front().afps_lod_mode_enabled_flag(true);

    auto pdu1 = PatchDataUnit{};
    pdu1.pdu_2d_size_x_minus1(10).pdu_2d_size_y_minus1(20).pdu_lod_enabled_flag(false);
    auto pdu2 = PatchDataUnit{};
    pdu2.pdu_2d_size_x_minus1(30).pdu_2d_size_y_minus1(40).pdu_lod_enabled_flag(false);
    auto pdu3 = PatchDataUnit{};
    pdu3.pdu_2d_size_x_minus1(50)
        .pdu_2d_size_y_minus1(60)
        .pdu_lod_enabled_flag(true)
        .pdu_lod_scale_x_minus1(1)
        .pdu_lod_scale_y_idc(3);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_RSV_ACL_35, 0, 2};

    auto x = AtlasTileLayerRBSP{};
    x.atlas_tile_header().ath_type(AthType::I_TILE).ath_ref_atlas_frame_list_asps_flag(true);
    x.atlas_tile_data_unit() = [&]() {
      auto atdu = AtlasTileDataUnit{};

      atdu.atdu_patch_mode(0, AtduPatchMode::I_INTRA);
      atdu.atdu_patch_mode(1, AtduPatchMode::I_INTRA);
      atdu.atdu_patch_mode(2, AtduPatchMode::I_INTRA);
      atdu.atdu_patch_mode(3, AtduPatchMode::I_END);

      atdu.patch_information_data(0).patch_data_unit() = pdu1;
      atdu.patch_information_data(1).patch_data_unit() = pdu2;
      atdu.patch_information_data(2).patch_data_unit() = pdu3;

      return atdu;
    }();

    REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=I_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
atdu_patch_mode[ 0 ][ 0 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 0 ]=0
pdu_2d_pos_y[ 0 ][ 0 ]=0
pdu_2d_size_x_minus1[ 0 ][ 0 ]=10
pdu_2d_size_y_minus1[ 0 ][ 0 ]=20
pdu_3d_offset_u[ 0 ][ 0 ]=0
pdu_3d_offset_v[ 0 ][ 0 ]=0
pdu_3d_offset_d[ 0 ][ 0 ]=0
pdu_projection_id[ 0 ][ 0 ]=0
pdu_orientation_index[ 0 ][ 0 ]=FPO_NULL
pdu_lod_enabled_flag[ 0 ][ 0 ]=false
atdu_patch_mode[ 0 ][ 1 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 1 ]=0
pdu_2d_pos_y[ 0 ][ 1 ]=0
pdu_2d_size_x_minus1[ 0 ][ 1 ]=30
pdu_2d_size_y_minus1[ 0 ][ 1 ]=40
pdu_3d_offset_u[ 0 ][ 1 ]=0
pdu_3d_offset_v[ 0 ][ 1 ]=0
pdu_3d_offset_d[ 0 ][ 1 ]=0
pdu_projection_id[ 0 ][ 1 ]=0
pdu_orientation_index[ 0 ][ 1 ]=FPO_NULL
pdu_lod_enabled_flag[ 0 ][ 1 ]=false
atdu_patch_mode[ 0 ][ 2 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 2 ]=0
pdu_2d_pos_y[ 0 ][ 2 ]=0
pdu_2d_size_x_minus1[ 0 ][ 2 ]=50
pdu_2d_size_y_minus1[ 0 ][ 2 ]=60
pdu_3d_offset_u[ 0 ][ 2 ]=0
pdu_3d_offset_v[ 0 ][ 2 ]=0
pdu_3d_offset_d[ 0 ][ 2 ]=0
pdu_projection_id[ 0 ][ 2 ]=0
pdu_orientation_index[ 0 ][ 2 ]=FPO_NULL
pdu_lod_enabled_flag[ 0 ][ 2 ]=true
pdu_lod_scale_x_minus1[ 0 ][ 2 ]=1
pdu_lod_scale_y_idc[ 0 ][ 2 ]=3
atdu_patch_mode[ 0 ][ 3 ]=I_END
AtduTotalNumPatches[ 0 ]=3
)");
    byteCodingTest(x, 16, nuh, aspsV, afpsV);
  }

  SECTION("I_TILE with quantizers") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front()
        .asps_geometry_2d_bit_depth_minus1(11)
        .asps_geometry_3d_bit_depth_minus1(11)
        .asps_num_ref_atlas_frame_lists_in_asps(1)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    auto pdu1 = PatchDataUnit{};
    pdu1.pdu_2d_size_x_minus1(10).pdu_2d_size_y_minus1(20).pdu_3d_offset_d(31).pdu_3d_range_d(127);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_BLA_W_LP, 0, 1};

    auto x = AtlasTileLayerRBSP{};
    x.atlas_tile_header()
        .ath_no_output_of_prior_atlas_frames_flag(false)
        .ath_type(AthType::I_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true)
        .ath_pos_min_d_quantizer(7)
        .ath_pos_delta_max_d_quantizer(5);

    x.atlas_tile_data_unit() = [&pdu1]() {
      auto atdu = AtlasTileDataUnit{};

      atdu.atdu_patch_mode(0, AtduPatchMode::I_INTRA);
      atdu.atdu_patch_mode(1, AtduPatchMode::I_END);

      atdu.patch_information_data(0).patch_data_unit() = pdu1;

      return atdu;
    }();

    REQUIRE(toString(x) == R"(ath_no_output_of_prior_atlas_frames_flag=false
ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=I_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
ath_pos_min_d_quantizer=7
ath_pos_delta_max_d_quantizer=5
atdu_patch_mode[ 0 ][ 0 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 0 ]=0
pdu_2d_pos_y[ 0 ][ 0 ]=0
pdu_2d_size_x_minus1[ 0 ][ 0 ]=10
pdu_2d_size_y_minus1[ 0 ][ 0 ]=20
pdu_3d_offset_u[ 0 ][ 0 ]=0
pdu_3d_offset_v[ 0 ][ 0 ]=0
pdu_3d_offset_d[ 0 ][ 0 ]=31
pdu_3d_range_d[ 0 ][ 0 ]=127
pdu_projection_id[ 0 ][ 0 ]=0
pdu_orientation_index[ 0 ][ 0 ]=FPO_NULL
atdu_patch_mode[ 0 ][ 1 ]=I_END
AtduTotalNumPatches[ 0 ]=1
)");
    byteCodingTest(x, 12, nuh, aspsV, afpsV);
  }
}
} // namespace TMIV::MivBitstream
