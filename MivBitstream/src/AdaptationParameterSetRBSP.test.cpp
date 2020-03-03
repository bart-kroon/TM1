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

#include <TMIV/MivBitstream/AdaptationParameterSetRBSP.h>

using namespace TMIV::MivBitstream;

TEST_CASE("camera_intrinsics", "[Adaptation Parameter Set RBSP]") {
  auto x = CameraIntrinsics{};

  SECTION("equirectangular") {
    x.ci_cam_type(CiCamType::equirectangular)
        .ci_erp_phi_min(-2.F)
        .ci_erp_phi_max(2.F)
        .ci_erp_theta_min(-1.F)
        .ci_erp_theta_max(1.F);

    REQUIRE(toString(x, 1) == R"(ci_cam_type[ 1 ]=equirectangular
ci_projection_plane_width_minus1[ 1 ]=0
ci_projection_plane_height_minus1[ 1 ]=0
ci_erp_phi_min[ 1 ]=-2
ci_erp_phi_max[ 1 ]=2
ci_erp_theta_min[ 1 ]=-1
ci_erp_theta_max[ 1 ]=1
)");

    REQUIRE(bitCodingTest(x, 168));
  }

  SECTION("perspective") {
    x.ci_cam_type(CiCamType::perspective)
        .ci_projection_plane_width_minus1(19)
        .ci_projection_plane_height_minus1(9)
        .ci_perspective_focal_hor(50.F)
        .ci_perspective_focal_ver(25.F)
        .ci_perspective_center_hor(10.F)
        .ci_perspective_center_ver(5.F);

    REQUIRE(toString(x, 1) == R"(ci_cam_type[ 1 ]=perspective
ci_projection_plane_width_minus1[ 1 ]=19
ci_projection_plane_height_minus1[ 1 ]=9
ci_perspective_focal_hor[ 1 ]=50
ci_perspective_focal_ver[ 1 ]=25
ci_perspective_center_hor[ 1 ]=10
ci_perspective_center_ver[ 1 ]=5
)");

    REQUIRE(bitCodingTest(x, 168));
  }

  SECTION("orthographic") {
    x.ci_cam_type(CiCamType::orthographic)
        .ci_projection_plane_width_minus1(1023)
        .ci_projection_plane_height_minus1(767)
        .ci_ortho_width(100.F)
        .ci_ortho_height(50.F);

    REQUIRE(toString(x, 1) == R"(ci_cam_type[ 1 ]=orthographic
ci_projection_plane_width_minus1[ 1 ]=1023
ci_projection_plane_height_minus1[ 1 ]=767
ci_ortho_width[ 1 ]=100
ci_ortho_height[ 1 ]=50
)");

    REQUIRE(bitCodingTest(x, 104));
  }
}

TEST_CASE("camera_extrinsics", "[Adaptation Parameter Set RBSP]") {
  auto x = CameraExtrinsics{};

  REQUIRE(toString(x, 1) == R"(ce_view_pos_x[ 1 ]=0
ce_view_pos_y[ 1 ]=0
ce_view_pos_z[ 1 ]=0
ce_view_quat_x[ 1 ]=0
ce_view_quat_y[ 1 ]=0
ce_view_quat_z[ 1 ]=0
)");

  REQUIRE(bitCodingTest(x, 192));

  SECTION("Example") {
    x.ce_view_pos_x(3.F)
        .ce_view_pos_y(1.F)
        .ce_view_pos_z(4.F)
        .ce_view_quat_x(5.F)
        .ce_view_quat_y(9.F)
        .ce_view_quat_z(14.F);

    REQUIRE(toString(x, 1) == R"(ce_view_pos_x[ 1 ]=3
ce_view_pos_y[ 1 ]=1
ce_view_pos_z[ 1 ]=4
ce_view_quat_x[ 1 ]=5
ce_view_quat_y[ 1 ]=9
ce_view_quat_z[ 1 ]=14
)");

    REQUIRE(bitCodingTest(x, 192));
  }
}

TEST_CASE("depth_quantization", "[Adaptation Parameter Set RBSP]") {
  auto x = DepthQuantization{};

  REQUIRE(toString(x, 7) == R"(dq_quantization_law[ 7 ]=0
dq_norm_disp_low[ 7 ]=0
dq_norm_disp_high[ 7 ]=0
dq_depth_occ_map_threshold_default[ 7 ]=0
)");

  REQUIRE(bitCodingTest(x, 73));

  SECTION("Example 2") {
    x.dq_norm_disp_low(0.02F);
    x.dq_norm_disp_high(2.F);
    x.dq_depth_occ_map_threshold_default(200);

    REQUIRE(toString(x, 2) == R"(dq_quantization_law[ 2 ]=0
dq_norm_disp_low[ 2 ]=0.02
dq_norm_disp_high[ 2 ]=2
dq_depth_occ_map_threshold_default[ 2 ]=200
)");

    REQUIRE(bitCodingTest(x, 87));
  }
}

TEST_CASE("pruning_children", "[Adaptation Parameter Set RBSP]") {
  SECTION("Example 1") {
    const auto x = PruningChildren{};
    REQUIRE(toString(x, 3) == R"(pc_is_leaf_flag[ 3 ]=true
)");

    const uint16_t mvp_num_views_minus1 = 10;
    REQUIRE(bitCodingTest(x, 1, mvp_num_views_minus1));
  }

  SECTION("Example 2") {
    const auto x = PruningChildren{{2, 3, 5, 8}};
    REQUIRE(toString(x, 5) == R"(pc_is_leaf_flag[ 5 ]=false
pc_num_children_minus1[ 5 ]=3
pc_child_id[ 5 ][ 0 ]=2
pc_child_id[ 5 ][ 1 ]=3
pc_child_id[ 5 ][ 2 ]=5
pc_child_id[ 5 ][ 3 ]=8
)");

    const uint16_t mvp_num_views_minus1 = 10;
    REQUIRE(bitCodingTest(x, 21, mvp_num_views_minus1));
  }
}

TEST_CASE("miv_view_params_list", "[Adaptation Parameter Set RBSP]") {
  auto x = MivViewParamsList{};

  SECTION("Example 1") {
    x.mvp_num_views_minus1(0)
        .mvp_intrinsic_params_equal_flag(false)
        .mvp_depth_quantization_params_equal_flag(false)
        .mvp_pruning_graph_params_present_flag(false);
    x.camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);

    REQUIRE(toString(x) == R"(mvp_num_views_minus1=0
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_intrinsic_params_equal_flag=false
ci_cam_type[ 0 ]=orthographic
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_ortho_width[ 0 ]=4
ci_ortho_height[ 0 ]=3
mvp_depth_quantization_params_equal_flag=false
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=0
dq_norm_disp_high[ 0 ]=0
dq_depth_occ_map_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=false
)");

    REQUIRE(bitCodingTest(x, 388));
  }

  SECTION("Example 2") {
    x.mvp_num_views_minus1(2)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true);
    x.camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);

    REQUIRE(toString(x) == R"(mvp_num_views_minus1=2
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
ce_view_pos_x[ 1 ]=0
ce_view_pos_y[ 1 ]=0
ce_view_pos_z[ 1 ]=0
ce_view_quat_x[ 1 ]=0
ce_view_quat_y[ 1 ]=0
ce_view_quat_z[ 1 ]=0
ce_view_pos_x[ 2 ]=0
ce_view_pos_y[ 2 ]=0
ce_view_pos_z[ 2 ]=0
ce_view_quat_x[ 2 ]=0
ce_view_quat_y[ 2 ]=0
ce_view_quat_z[ 2 ]=0
mvp_intrinsic_params_equal_flag=true
ci_cam_type[ 0 ]=orthographic
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_ortho_width[ 0 ]=4
ci_ortho_height[ 0 ]=3
mvp_depth_quantization_params_equal_flag=true
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=0
dq_norm_disp_high[ 0 ]=0
dq_depth_occ_map_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=true
pc_is_leaf_flag[ 0 ]=true
pc_is_leaf_flag[ 1 ]=true
pc_is_leaf_flag[ 2 ]=true
)");

    REQUIRE(bitCodingTest(x, 775));
  }
}

TEST_CASE("adaptation_parameter_set_rbsp", "[Adaptation Parameter Set RBSP]") {
  auto x = AdaptationParameterSetRBSP{};

  REQUIRE(toString(x) == R"(aps_adaptation_parameter_set_id=0
aps_camera_params_present_flag=false
aps_miv_view_params_list_present_flag=false
aps_extension2_flag=false
)");

  REQUIRE(byteCodingTest(x, 1));

  SECTION("Example 1") {
    x.aps_adaptation_parameter_set_id(63);
    x.aps_miv_view_params_list_present_flag(true);
    x.aps_miv_view_params_list_update_mode(MvpUpdateMode::VPL_INITLIST);
    x.miv_view_params_list() = MivViewParamsList{};
    x.miv_view_params_list()
        .mvp_num_views_minus1(2)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);

    REQUIRE(toString(x) == R"(aps_adaptation_parameter_set_id=63
aps_camera_params_present_flag=false
aps_miv_view_params_list_present_flag=true
aps_miv_view_params_list_update_mode=VPL_INITLIST
mvp_num_views_minus1=2
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
ce_view_pos_x[ 1 ]=0
ce_view_pos_y[ 1 ]=0
ce_view_pos_z[ 1 ]=0
ce_view_quat_x[ 1 ]=0
ce_view_quat_y[ 1 ]=0
ce_view_quat_z[ 1 ]=0
ce_view_pos_x[ 2 ]=0
ce_view_pos_y[ 2 ]=0
ce_view_pos_z[ 2 ]=0
ce_view_quat_x[ 2 ]=0
ce_view_quat_y[ 2 ]=0
ce_view_quat_z[ 2 ]=0
mvp_intrinsic_params_equal_flag=true
ci_cam_type[ 0 ]=orthographic
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_ortho_width[ 0 ]=4
ci_ortho_height[ 0 ]=3
mvp_depth_quantization_params_equal_flag=true
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=0
dq_norm_disp_high[ 0 ]=0
dq_depth_occ_map_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=true
pc_is_leaf_flag[ 0 ]=true
pc_is_leaf_flag[ 1 ]=true
pc_is_leaf_flag[ 2 ]=true
aps_extension2_flag=false
)");

    REQUIRE(byteCodingTest(x, 100));
  }
}
