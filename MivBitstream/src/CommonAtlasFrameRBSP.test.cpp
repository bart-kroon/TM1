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
  const auto maxCommonAtlasFrmOrderCntLsb = 32;

  auto vps = V3cParameterSet{};
  vps.vps_extension_present_flag(true);
  vps.vps_miv_extension_present_flag(true);
  vps.vps_miv_extension() = {};

  SECTION("Default constructor") {
    REQUIRE(toString(x) == R"(caf_common_atlas_sequence_parameter_set_id=0
caf_common_atlas_frm_order_cnt_lsb=0
caf_extension_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 2, vps, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("Extension present, but no MIV extension") {
    x.caf_common_atlas_sequence_parameter_set_id(15)
        .caf_common_atlas_frm_order_cnt_lsb(31)
        .caf_extension_present_flag(true)
        .caf_miv_extension_present_flag(false)
        .caf_extension_7bits(127)
        .cafExtensionData({true, false});

    REQUIRE(toString(x) == R"(caf_common_atlas_sequence_parameter_set_id=15
caf_common_atlas_frm_order_cnt_lsb=31
caf_extension_present_flag=true
caf_miv_extension_present_flag=false
caf_extension_7bits=127
caf_extension_data_flag=true
caf_extension_data_flag=false
)");

    REQUIRE(byteCodingTest(x, 3, vps, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("MIV extension present") {
    x.caf_common_atlas_sequence_parameter_set_id(14)
        .caf_common_atlas_frm_order_cnt_lsb(30)
        .caf_extension_present_flag(true)
        .caf_miv_extension_present_flag(true)
        .caf_miv_extension() = {};

    REQUIRE(toString(x) == R"(caf_common_atlas_sequence_parameter_set_id=14
caf_common_atlas_frm_order_cnt_lsb=30
caf_extension_present_flag=true
caf_miv_extension_present_flag=true
caf_extension_7bits=0
came_irap_flag=true
mvp_num_views_minus1=0
mvp_view_enabled_present_flag=false
mvp_explicit_view_id_flag=false
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_intrinsic_params_equal_flag=false
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=0
ci_erp_phi_max[ 0 ]=0
ci_erp_theta_min[ 0 ]=0
ci_erp_theta_max[ 0 ]=0
mvp_depth_quantization_params_equal_flag=false
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=0
dq_norm_disp_high[ 0 ]=0
dq_depth_occ_map_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 60, vps, maxCommonAtlasFrmOrderCntLsb));
  }
}
} // namespace TMIV::MivBitstream
