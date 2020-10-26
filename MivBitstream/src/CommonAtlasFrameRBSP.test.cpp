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

#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("common_atlas_frame_rbsp", "[Common Atlas Frame RBSP]") {
  auto x = CommonAtlasFrameRBSP{};
  auto vps = V3cParameterSet{};
  vps.vps_extension_present_flag(true);
  vps.vps_miv_extension_present_flag(true);
  vps.vps_miv_extension() = {};

  const auto maxCommonAtlasFrmOrderCntLsb = 16;

  SECTION("Initialize view parameters") {
    x.caf_atlas_adaptation_parameter_set_id(63)
        .caf_frm_order_cnt_lsb(15)
        .caf_extension_present_flag(true)
        .caf_extension_8bits(255)
        .cafExtensionData({true})
        .miv_view_params_list()
        .mvp_num_views_minus1(2)
        .mvp_view_enabled_in_atlas_flag(0, 0, false)
        .mvp_view_enabled_in_atlas_flag(0, 1, false)
        .mvp_view_enabled_in_atlas_flag(0, 2, false)
        .mvp_explicit_view_id_flag(false)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);

    REQUIRE(toString(x) == R"(caf_atlas_adaptation_parameter_set_id=63
caf_frm_order_cnt_lsb=15
caf_irap_flag=true
mvp_num_views_minus1=2
mvp_view_enabled_present_flag=true
mvp_view_enabled_in_atlas_flag[ 0 ][ 0 ]=false
mvp_view_enabled_in_atlas_flag[ 0 ][ 1 ]=false
mvp_view_enabled_in_atlas_flag[ 0 ][ 2 ]=false
mvp_explicit_view_id_flag=false
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
pp_is_root_flag[ 0 ]=true
pp_is_root_flag[ 1 ]=true
pp_is_root_flag[ 2 ]=true
caf_extension_present_flag=true
caf_extension_8bits=255
caf_extension_data_flag=true
)");

    REQUIRE(byteCodingTest(x, 102, vps, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("Update extrinsics") {
    x.caf_atlas_adaptation_parameter_set_id(63)
        .caf_frm_order_cnt_lsb(15)
        .caf_irap_flag(false)
        .caf_update_extrinsics_flag(true)
        .miv_view_params_update_extrinsics()
        .mvpue_num_view_updates_minus1(0)
        .mvpue_view_idx(0, 3)
        .camera_extrinsics(0)
        .ce_view_pos_x(1.F)
        .ce_view_pos_y(2.F)
        .ce_view_pos_z(3.F)
        .ce_view_quat_x(4.F)
        .ce_view_quat_y(5.F)
        .ce_view_quat_z(6.F);

    REQUIRE(toString(x) == R"(caf_atlas_adaptation_parameter_set_id=63
caf_frm_order_cnt_lsb=15
caf_irap_flag=false
caf_update_extrinsics_flag=true
caf_update_intrinsics_flag=false
caf_update_depth_quantization_flag=false
mvpue_num_view_updates_minus1=0
mvpue_view_idx[ 0 ]=3
ce_view_pos_x[ 0 ]=1
ce_view_pos_y[ 0 ]=2
ce_view_pos_z[ 0 ]=3
ce_view_quat_x[ 0 ]=4
ce_view_quat_y[ 0 ]=5
ce_view_quat_z[ 0 ]=6
caf_extension_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 31, vps, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("Update camera intrinsics") {
    x.caf_atlas_adaptation_parameter_set_id(4)
        .caf_frm_order_cnt_lsb(4)
        .caf_irap_flag(false)
        .caf_update_intrinsics_flag(true)
        .miv_view_params_update_intrinsics()
        .mvpui_num_view_updates_minus1(0)
        .mvpui_view_idx(0, 6)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::equirectangular)
        .ci_erp_phi_min(-2.F)
        .ci_erp_phi_max(2.F)
        .ci_erp_theta_min(-1.F)
        .ci_erp_theta_max(1.F);

    REQUIRE(toString(x) == R"(caf_atlas_adaptation_parameter_set_id=4
caf_frm_order_cnt_lsb=4
caf_irap_flag=false
caf_update_extrinsics_flag=false
caf_update_intrinsics_flag=true
caf_update_depth_quantization_flag=false
mvpui_num_view_updates_minus1=0
mvpui_view_idx[ 0 ]=6
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=-2
ci_erp_phi_max[ 0 ]=2
ci_erp_theta_min[ 0 ]=-1
ci_erp_theta_max[ 0 ]=1
caf_extension_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 27, vps, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("Update depth quantization") {
    x.caf_atlas_adaptation_parameter_set_id(5)
        .caf_frm_order_cnt_lsb(9)
        .caf_irap_flag(false)
        .caf_update_depth_quantization_flag(true)
        .miv_view_params_update_depth_quantization()
        .mvpudq_num_view_updates_minus1(0)
        .mvpudq_view_idx(0, 6)
        .depth_quantization(0)
        .dq_depth_occ_map_threshold_default(64)
        .dq_norm_disp_low(1.F)
        .dq_norm_disp_high(100.F);

    REQUIRE(toString(x) == R"(caf_atlas_adaptation_parameter_set_id=5
caf_frm_order_cnt_lsb=9
caf_irap_flag=false
caf_update_extrinsics_flag=false
caf_update_intrinsics_flag=false
caf_update_depth_quantization_flag=true
mvpudq_num_view_updates_minus1=0
mvpudq_view_idx[ 0 ]=6
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=1
dq_norm_disp_high[ 0 ]=100
dq_depth_occ_map_threshold_default[ 0 ]=64
caf_extension_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 17, vps, maxCommonAtlasFrmOrderCntLsb));
  }
}
} // namespace TMIV::MivBitstream
