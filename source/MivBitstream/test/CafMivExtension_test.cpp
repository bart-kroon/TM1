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

#include <TMIV/MivBitstream/CafMivExtension.h>

#include <limits>
#include <random>

namespace TMIV::MivBitstream {
TEST_CASE("camera_intrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = CameraIntrinsics{};

  SECTION("Default constructor: equirectangular") {
    REQUIRE(toString(unit, uint16_t{1}) == R"(ci_cam_type[ 1 ]=equirectangular
ci_projection_plane_width_minus1[ 1 ]=0
ci_projection_plane_height_minus1[ 1 ]=0
ci_erp_phi_min[ 1 ]=0
ci_erp_phi_max[ 1 ]=0
ci_erp_theta_min[ 1 ]=0
ci_erp_theta_max[ 1 ]=0
)");

    bitCodingTest(unit, 168);
  }

  SECTION("equirectangular custom") {
    unit.ci_cam_type(CiCamType::equirectangular)
        .ci_erp_phi_min(-2.F)
        .ci_erp_phi_max(2.F)
        .ci_erp_theta_min(-1.F)
        .ci_erp_theta_max(1.F);

    REQUIRE(toString(unit, uint16_t{1}) == R"(ci_cam_type[ 1 ]=equirectangular
ci_projection_plane_width_minus1[ 1 ]=0
ci_projection_plane_height_minus1[ 1 ]=0
ci_erp_phi_min[ 1 ]=-2
ci_erp_phi_max[ 1 ]=2
ci_erp_theta_min[ 1 ]=-1
ci_erp_theta_max[ 1 ]=1
)");

    bitCodingTest(unit, 168);
  }

  SECTION("perspective") {
    unit.ci_cam_type(CiCamType::perspective)
        .ci_projection_plane_width_minus1(19)
        .ci_projection_plane_height_minus1(9)
        .ci_perspective_focal_hor(50.F)
        .ci_perspective_focal_ver(25.F)
        .ci_perspective_center_hor(10.F)
        .ci_perspective_center_ver(5.F);

    REQUIRE(toString(unit, uint16_t{1}) == R"(ci_cam_type[ 1 ]=perspective
ci_projection_plane_width_minus1[ 1 ]=19
ci_projection_plane_height_minus1[ 1 ]=9
ci_perspective_focal_hor[ 1 ]=50
ci_perspective_focal_ver[ 1 ]=25
ci_perspective_center_hor[ 1 ]=10
ci_perspective_center_ver[ 1 ]=5
)");

    bitCodingTest(unit, 168);
  }

  SECTION("orthographic") {
    unit.ci_cam_type(CiCamType::orthographic)
        .ci_projection_plane_width_minus1(1023)
        .ci_projection_plane_height_minus1(767)
        .ci_ortho_width(100.F)
        .ci_ortho_height(50.F);

    REQUIRE(toString(unit, uint16_t{1}) == R"(ci_cam_type[ 1 ]=orthographic
ci_projection_plane_width_minus1[ 1 ]=1023
ci_projection_plane_height_minus1[ 1 ]=767
ci_ortho_width[ 1 ]=100
ci_ortho_height[ 1 ]=50
)");

    bitCodingTest(unit, 104);
  }
}

TEST_CASE("camera_extrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = CameraExtrinsics{};

  REQUIRE(toString(unit, uint16_t{1}) == R"(ce_view_pos_x[ 1 ]=0
ce_view_pos_y[ 1 ]=0
ce_view_pos_z[ 1 ]=0
ce_view_quat_x[ 1 ]=0
ce_view_quat_y[ 1 ]=0
ce_view_quat_z[ 1 ]=0
)");

  bitCodingTest(unit, 192);

  SECTION("Example") {
    unit.ce_view_pos_x(3.F)
        .ce_view_pos_y(1.F)
        .ce_view_pos_z(4.F)
        .ce_view_quat_x(153)
        .ce_view_quat_y(-1239)
        .ce_view_quat_z(0);

    REQUIRE(toString(unit, uint16_t{1}) == R"(ce_view_pos_x[ 1 ]=3
ce_view_pos_y[ 1 ]=1
ce_view_pos_z[ 1 ]=4
ce_view_quat_x[ 1 ]=153
ce_view_quat_y[ 1 ]=-1239
ce_view_quat_z[ 1 ]=0
)");

    bitCodingTest(unit, 192);
  }
}

TEST_CASE("depth_quantization", "[Common Atlas Frame MIV Extension]") {
  auto unit = DepthQuantization{};

  REQUIRE(toString(unit, uint16_t{7}) == R"(dq_quantization_law[ 7 ]=0
dq_norm_disp_low[ 7 ]=0
dq_norm_disp_high[ 7 ]=0
dq_depth_occ_threshold_default[ 7 ]=0
)");

  bitCodingTest(unit, 66);

  SECTION("Example 2") {
    unit.dq_norm_disp_low(0.02F);
    unit.dq_norm_disp_high(2.F);
    unit.dq_depth_occ_threshold_default(200);

    REQUIRE(toString(unit, uint16_t{2}) == R"(dq_quantization_law[ 2 ]=0
dq_norm_disp_low[ 2 ]=0.02
dq_norm_disp_high[ 2 ]=2
dq_depth_occ_threshold_default[ 2 ]=200
)");

    bitCodingTest(unit, 80);
  }
}

TEST_CASE("depth_quantization linear", "[Common Atlas Frame MIV Extension]") {
  auto unit = DepthQuantization{};

  SECTION("Example 2") {
    unit.dq_quantization_law(4);
    unit.dq_linear_near(0.02F);
    unit.dq_linear_far(2.F);
    unit.dq_depth_occ_threshold_default(200);

    REQUIRE(toString(unit, uint16_t{2}) == R"(dq_quantization_law[ 2 ]=4
dq_linear_near[ 2 ]=0.02
dq_linear_far[ 2 ]=2
dq_depth_occ_threshold_default[ 2 ]=200
)");

    bitCodingTest(unit, 84);
  }
}

TEST_CASE("chroma_scaling", "[Common Atlas Frame MIV Extension]") {
  auto unit = ChromaScaling{};

  const auto mvpl = CaspsMiv2Extension{}.casme_chroma_scaling_bit_depth_minus1(9);

  REQUIRE(toString(unit, uint16_t{7}) == R"(cs_u_min[ 7 ]=0
cs_u_max[ 7 ]=0
cs_v_min[ 7 ]=0
cs_v_max[ 7 ]=0
)");

  bitCodingTest(unit, 40, mvpl);

  SECTION("Example 2") {
    unit.cs_u_min(100);
    unit.cs_u_max(905);
    unit.cs_v_min(440);
    unit.cs_v_max(640);

    REQUIRE(toString(unit, uint16_t{2}) == R"(cs_u_min[ 2 ]=100
cs_u_max[ 2 ]=905
cs_v_min[ 2 ]=440
cs_v_max[ 2 ]=640
)");

    bitCodingTest(unit, 40, mvpl);
  }
}

TEST_CASE("pruning_parent", "[Common Atlas Frame MIV Extension]") {
  SECTION("Example 1") {
    const auto unit = PruningParents{};
    REQUIRE(toString(unit, uint16_t{3}) == R"(pp_is_root_flag[ 3 ]=true
)");

    const uint16_t mvp_num_views_minus1 = 10;
    bitCodingTest(unit, 1, mvp_num_views_minus1);
  }

  SECTION("Example 2") {
    const auto unit = PruningParents{{2, 3, 5, 8}};
    REQUIRE(toString(unit, uint16_t{5}) == R"(pp_is_root_flag[ 5 ]=false
pp_num_parent_minus1[ 5 ]=3
pp_parent_idx[ 5 ][ 0 ]=2
pp_parent_idx[ 5 ][ 1 ]=3
pp_parent_idx[ 5 ][ 2 ]=5
pp_parent_idx[ 5 ][ 3 ]=8
)");

    const uint16_t mvp_num_views_minus1 = 10;
    bitCodingTest(unit, 21, mvp_num_views_minus1);
  }
}

TEST_CASE("miv_view_params_list", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsList{};
  auto casps = CommonAtlasSequenceParameterSetRBSP{};
  casps.casps_miv_extension().casme_depth_quantization_params_present_flag(false);

  SECTION("Default constructor") {
    REQUIRE(toString(unit, casps) == R"(mvp_num_views_minus1=0
mvp_explicit_view_id_flag=false
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_inpaint_flag[ 0 ]=false
mvp_intrinsic_params_equal_flag=false
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=0
ci_erp_phi_max[ 0 ]=0
ci_erp_theta_min[ 0 ]=0
ci_erp_theta_max[ 0 ]=0
mvp_pruning_graph_params_present_flag=false
cs_u_min[ 0 ]=0
cs_u_max[ 0 ]=0
cs_v_min[ 0 ]=0
cs_v_max[ 0 ]=0
mvp_view_background_flag[ 0 ]=false
)");

    bitCodingTest(unit, 380, casps);
    REQUIRE(unit.mvp_view_id(0) == ViewId{});
  }

  SECTION("Example 1") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(true);
    casps.casps_miv_2_extension().casme_chroma_scaling_bit_depth_minus1(9);
    unit.mvp_num_views_minus1(0)
        .mvp_explicit_view_id_flag(false)
        .mvp_intrinsic_params_equal_flag(false)
        .mvp_depth_quantization_params_equal_flag(false)
        .mvp_pruning_graph_params_present_flag(false);
    unit.camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);
    unit.chroma_scaling(0).cs_u_min(10).cs_u_max(300).cs_v_min(111).cs_v_max(400);

    REQUIRE(toString(unit, casps) == R"(mvp_num_views_minus1=0
mvp_explicit_view_id_flag=false
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_inpaint_flag[ 0 ]=false
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
dq_depth_occ_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=false
cs_u_min[ 0 ]=10
cs_u_max[ 0 ]=300
cs_v_min[ 0 ]=111
cs_v_max[ 0 ]=400
mvp_view_background_flag[ 0 ]=false
)");

    bitCodingTest(unit, 423, casps);
  }

  SECTION("Example 2") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(true);
    casps.casps_miv_2_extension() = {};
    unit.mvp_num_views_minus1(2)
        .mvp_view_id(0, ViewId{})
        .mvp_view_id(1, ViewId{2})
        .mvp_view_id(2, ViewId{1})
        .mvp_inpaint_flag(1, true)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true);
    unit.camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);

    REQUIRE(toString(unit, casps) == R"(mvp_num_views_minus1=2
mvp_explicit_view_id_flag=true
mvp_view_id[ 0 ]=0
mvp_view_id[ 1 ]=2
mvp_view_id[ 2 ]=1
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_inpaint_flag[ 0 ]=false
ce_view_pos_x[ 1 ]=0
ce_view_pos_y[ 1 ]=0
ce_view_pos_z[ 1 ]=0
ce_view_quat_x[ 1 ]=0
ce_view_quat_y[ 1 ]=0
ce_view_quat_z[ 1 ]=0
mvp_inpaint_flag[ 1 ]=true
ce_view_pos_x[ 2 ]=0
ce_view_pos_y[ 2 ]=0
ce_view_pos_z[ 2 ]=0
ce_view_quat_x[ 2 ]=0
ce_view_quat_y[ 2 ]=0
ce_view_quat_z[ 2 ]=0
mvp_inpaint_flag[ 2 ]=false
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
dq_depth_occ_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=true
pp_is_root_flag[ 0 ]=true
pp_is_root_flag[ 1 ]=true
pp_is_root_flag[ 2 ]=true
cs_u_min[ 0 ]=0
cs_u_max[ 0 ]=0
cs_v_min[ 0 ]=0
cs_v_max[ 0 ]=0
cs_u_min[ 1 ]=0
cs_u_max[ 1 ]=0
cs_v_min[ 1 ]=0
cs_v_max[ 1 ]=0
cs_u_min[ 2 ]=0
cs_u_max[ 2 ]=0
cs_v_min[ 2 ]=0
cs_v_max[ 2 ]=0
mvp_view_background_flag[ 0 ]=false
mvp_view_background_flag[ 1 ]=false
mvp_view_background_flag[ 2 ]=false
)");

    bitCodingTest(unit, 820, casps);
    REQUIRE(unit.mvp_view_id(0) == ViewId{0});
    REQUIRE(unit.mvp_view_id(2) == ViewId{1});
    REQUIRE(unit.mvp_view_id(1) == ViewId{2});
  }

  SECTION("mvp when casme_depth_quantization_params_present_flag=0") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(false);

    REQUIRE(toString(unit, casps) == R"(mvp_num_views_minus1=0
mvp_explicit_view_id_flag=false
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_inpaint_flag[ 0 ]=false
mvp_intrinsic_params_equal_flag=false
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=0
ci_erp_phi_max[ 0 ]=0
ci_erp_theta_min[ 0 ]=0
ci_erp_theta_max[ 0 ]=0
mvp_pruning_graph_params_present_flag=false
cs_u_min[ 0 ]=0
cs_u_max[ 0 ]=0
cs_v_min[ 0 ]=0
cs_v_max[ 0 ]=0
mvp_view_background_flag[ 0 ]=false
)");

    bitCodingTest(unit, 380, casps);
  }

  SECTION("miv2 when casme_capture_device_information_present_flag=true") {
    static const uint8_t count = 63;
    auto &cdi = casps.casps_miv_2_extension().capture_device_information();
    cdi.cdi_device_model_count_minus1(count);
    for (uint16_t m = 0; m <= cdi.cdi_device_model_count_minus1(); m++) {
      cdi.cdi_device_model_id(m, static_cast<uint8_t>(m));
      cdi.cdi_device_class_id(m, static_cast<uint8_t>(m));
      if (cdi.cdi_device_class_id(m) != 0) {
        cdi.cdi_sensor_count_minus1(m, m);
        for (uint16_t s = 0; s <= cdi.cdi_sensor_count_minus1(m); s++) {
          cdi.cdi_sensor_component_id(m, s, static_cast<uint8_t>(std::min(uint16_t{31}, s)));
        }
        cdi.cdi_intra_sensor_parallax_flag(m, m % 2 == 0);
        cdi.cdi_light_source_count(m, m);
        cdi.cdi_infrared_image_present_flag(m, m % 2 == 0);
        cdi.cdi_depth_confidence_present_flag(m, m % 2 == 0);
        if (cdi.cdi_depth_confidence_present_flag(m)) {
          for (uint16_t s = 0; s <= cdi.cdi_sensor_count_minus1(m); s++) {
            if (cdi.cdi_sensor_component_id(m, s) == 0) {
              cdi.cdi_depth_confidence_flag(m, s, (m % 2 == 0));
            }
          }
        }
      }
    }

    CaptureDeviceInformation::Semantics semantics;
    cdi.applySemantics(semantics);
    unit.mvp_num_views_minus1(count);
    unit.mvp_intrinsic_params_equal_flag(true);
    for (uint16_t v = 0; v <= count; v++) {
      unit.mvp_device_model_id(v, static_cast<uint8_t>(v));
      auto i = unit.mvp_device_model_id(v);
      for (uint16_t s = 0; s < semantics.sensorCount[i]; s++) {
        if (semantics.intraSensorParallaxFlag[i]) {
          unit.sensor_extrinsics(v, s).se_sensor_quat_z(int32_t{s});
        }
        unit.distortion_parameters(v, s).dp_model_id(s % 4);
      }
      for (uint16_t s = 0; s < semantics.lightSourceCount[i]; s++) {
        unit.light_source_extrinsics(v, s).lse_light_source_quat_z(int32_t{s});
      }
    }

    auto resultstring = toString(unit, casps);
    REQUIRE(resultstring.find(R"(mvp_device_model_id[ 63 ]=63
)") != std::string::npos);
    REQUIRE(resultstring.find(R"(se_sensor_quat_z[ 62 ][ 61 ]=61
)") != std::string::npos);
    REQUIRE(resultstring.find(R"(dp_model_id[ 63 ][ 63 ]=3
)") != std::string::npos);
    REQUIRE(resultstring.find(R"(lse_light_source_quat_z[ 63 ][ 62 ]=62
)") != std::string::npos);

    bitCodingTest(unit, 879034, casps);
  }
}

TEST_CASE("caf_miv_extension", "[Common Atlas Frame MIV Extension]") {
  auto unit = CafMivExtension{};
  auto casps = CommonAtlasSequenceParameterSetRBSP{};
  casps.casps_miv_extension() = {};
  const auto nalCaf = NalUnitHeader{NalUnitType::NAL_CAF_TRIAL, 0, 1};
  const auto nalIdrCaf = NalUnitHeader{NalUnitType::NAL_CAF_IDR, 0, 1};

  SECTION("Initialize view parameters") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(true);
    casps.casps_miv_2_extension()
        .casme_chroma_scaling_bit_depth_minus1(9)
        .casme_background_separation_enable_flag(true);
    unit.miv_view_params_list()
        .mvp_num_views_minus1(2)
        .mvp_explicit_view_id_flag(false)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true)
        .mvp_view_background_flag(1, true)
        .mvp_view_background_flag(2, true)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::orthographic)
        .ci_ortho_width(4.F)
        .ci_ortho_height(3.F);

    REQUIRE(toString(unit, casps) == R"(miv_view_params_list=mvp_num_views_minus1=2
mvp_explicit_view_id_flag=false
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_inpaint_flag[ 0 ]=false
ce_view_pos_x[ 1 ]=0
ce_view_pos_y[ 1 ]=0
ce_view_pos_z[ 1 ]=0
ce_view_quat_x[ 1 ]=0
ce_view_quat_y[ 1 ]=0
ce_view_quat_z[ 1 ]=0
mvp_inpaint_flag[ 1 ]=false
ce_view_pos_x[ 2 ]=0
ce_view_pos_y[ 2 ]=0
ce_view_pos_z[ 2 ]=0
ce_view_quat_x[ 2 ]=0
ce_view_quat_y[ 2 ]=0
ce_view_quat_z[ 2 ]=0
mvp_inpaint_flag[ 2 ]=false
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
dq_depth_occ_threshold_default[ 0 ]=0
mvp_pruning_graph_params_present_flag=true
pp_is_root_flag[ 0 ]=true
pp_is_root_flag[ 1 ]=true
pp_is_root_flag[ 2 ]=true
cs_u_min[ 0 ]=0
cs_u_max[ 0 ]=0
cs_v_min[ 0 ]=0
cs_v_max[ 0 ]=0
cs_u_min[ 1 ]=0
cs_u_max[ 1 ]=0
cs_v_min[ 1 ]=0
cs_v_max[ 1 ]=0
cs_u_min[ 2 ]=0
cs_u_max[ 2 ]=0
cs_v_min[ 2 ]=0
cs_v_max[ 2 ]=0
mvp_view_background_flag[ 0 ]=false
mvp_view_background_flag[ 1 ]=true
mvp_view_background_flag[ 2 ]=true
)");

    bitCodingTest(unit, 895, nalIdrCaf, casps);
  }

  SECTION("Update extrinsics") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(true);
    casps.casps_miv_2_extension().casme_chroma_scaling_bit_depth_minus1(9);
    unit.came_update_depth_quantization_flag(false)
        .came_update_intrinsics_flag(false)
        .came_update_extrinsics_flag(true)
        .came_update_chroma_scaling_flag(false)
        .miv_view_params_update_extrinsics()
        .mvpue_num_view_updates_minus1(0)
        .mvpue_view_idx(0, 3)
        .camera_extrinsics(0)
        .ce_view_pos_x(1.F)
        .ce_view_pos_y(2.F)
        .ce_view_pos_z(3.F)
        .ce_view_quat_x(std::numeric_limits<int16_t>::max())
        .ce_view_quat_y(std::numeric_limits<int16_t>::min())
        .ce_view_quat_z(1);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=true
came_update_intrinsics_flag=false
came_update_depth_quantization_flag=false
came_update_chroma_scaling_flag=false
mvpue_num_view_updates_minus1=0
mvpue_view_idx[ 0 ]=3
ce_view_pos_x[ 0 ]=1
ce_view_pos_y[ 0 ]=2
ce_view_pos_z[ 0 ]=3
ce_view_quat_x[ 0 ]=32767
ce_view_quat_y[ 0 ]=-32768
ce_view_quat_z[ 0 ]=1
)");

    bitCodingTest(unit, 228, nalCaf, casps);
  }

  SECTION("Update camera intrinsics") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(true);
    casps.casps_miv_2_extension().casme_chroma_scaling_bit_depth_minus1(5);
    unit.came_update_depth_quantization_flag(false)
        .came_update_intrinsics_flag(true)
        .came_update_extrinsics_flag(false)
        .came_update_chroma_scaling_flag(false)
        .miv_view_params_update_intrinsics()
        .mvpui_num_view_updates_minus1(0)
        .mvpui_view_idx(0, 6)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::equirectangular)
        .ci_erp_phi_min(-2.F)
        .ci_erp_phi_max(2.F)
        .ci_erp_theta_min(-1.F)
        .ci_erp_theta_max(1.F);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=false
came_update_intrinsics_flag=true
came_update_depth_quantization_flag=false
came_update_chroma_scaling_flag=false
mvpui_num_view_updates_minus1=0
mvpui_view_idx[ 0 ]=6
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=-2
ci_erp_phi_max[ 0 ]=2
ci_erp_theta_min[ 0 ]=-1
ci_erp_theta_max[ 0 ]=1
)");

    bitCodingTest(unit, 204, nalCaf, casps);
  }

  SECTION("Update depth quantization") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(true);
    casps.casps_miv_2_extension().casme_chroma_scaling_bit_depth_minus1(9);
    unit.came_update_depth_quantization_flag(true)
        .came_update_extrinsics_flag(false)
        .came_update_intrinsics_flag(false)
        .came_update_chroma_scaling_flag(false)
        .miv_view_params_update_depth_quantization()
        .mvpudq_num_view_updates_minus1(0)
        .mvpudq_view_idx(0, 6)
        .depth_quantization(0)
        .dq_depth_occ_threshold_default(64)
        .dq_norm_disp_low(1.F)
        .dq_norm_disp_high(100.F);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=false
came_update_intrinsics_flag=false
came_update_depth_quantization_flag=true
came_update_chroma_scaling_flag=false
mvpudq_num_view_updates_minus1=0
mvpudq_view_idx[ 0 ]=6
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=1
dq_norm_disp_high[ 0 ]=100
dq_depth_occ_threshold_default[ 0 ]=64
)");

    bitCodingTest(unit, 114, nalCaf, casps);
  }

  // TODO SECTION("Update chroma scaling")

  SECTION("came when casme_depth_quantization_params_present_flag=0") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(false);
    unit.came_update_intrinsics_flag(false).came_update_extrinsics_flag(false);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=false
came_update_intrinsics_flag=false
)");

    bitCodingTest(unit, 2, nalCaf, casps);
  }

  SECTION("Update sensor extrinsics") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(false);
    casps.casps_miv_2_extension().casme_capture_device_information_present_flag(true);
    unit.came_update_extrinsics_flag(false)
        .came_update_intrinsics_flag(false)
        .came_update_sensor_extrinsics_flag(true)
        .came_update_light_source_extrinsics_flag(false)
        .miv_view_params_update_sensor_extrinsics()
        .mvpuse_num_updates_minus1(0)
        .mvpuse_view_idx(0, 0)
        .mvpuse_sensor_idx(0, 0)
        .sensor_extrinsics(0, 0)
        .se_sensor_pos_x(1.0F)
        .se_sensor_pos_y(1.0F)
        .se_sensor_pos_z(1.0F)
        .se_sensor_quat_x(1)
        .se_sensor_quat_y(2)
        .se_sensor_quat_z(3);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=false
came_update_intrinsics_flag=false
came_update_sensor_extrinsics_flag=true
came_update_distortion_parameters_flag=false
came_update_light_source_extrinsics_flag=false
mvpuse_num_updates_minus1=0
mvpuse_view_idx[ 0 ]=0
mvpuse_sensor_idx[ 0 ]=0
se_sensor_pos_x[ 0 ][ 0 ]=1
se_sensor_pos_y[ 0 ][ 0 ]=1
se_sensor_pos_z[ 0 ][ 0 ]=1
se_sensor_quat_x[ 0 ][ 0 ]=1
se_sensor_quat_y[ 0 ][ 0 ]=2
se_sensor_quat_z[ 0 ][ 0 ]=3
)");

    bitCodingTest(unit, 245, nalCaf, casps);
  }

  SECTION("Update Distortion Parameters") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(false);
    casps.casps_miv_2_extension().casme_capture_device_information_present_flag(true);
    unit.came_update_extrinsics_flag(false)
        .came_update_intrinsics_flag(false)
        .miv_view_params_update_distortion_parameters()
        .mvpudp_num_updates_minus1(0)
        .mvpudp_view_idx(0, 0)
        .mvpudp_sensor_idx(0, 0)
        .distortion_parameters(0, 0)
        .dp_model_id(3)
        .dp_coefficient(0, 1.0F);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=false
came_update_intrinsics_flag=false
came_update_sensor_extrinsics_flag=false
came_update_distortion_parameters_flag=true
came_update_light_source_extrinsics_flag=false
mvpudp_num_updates_minus1=0
mvpudp_view_idx[ 0 ]=0
mvpudp_sensor_idx[ 0 ]=0
dp_model_id[ 0 ][ 0 ]=3
dp_coefficient[ 0 ][ 0 ][ 0 ]=1
dp_coefficient[ 0 ][ 0 ][ 1 ]=0
dp_coefficient[ 0 ][ 0 ][ 2 ]=0
dp_coefficient[ 0 ][ 0 ][ 3 ]=0
dp_coefficient[ 0 ][ 0 ][ 4 ]=0
dp_coefficient[ 0 ][ 0 ][ 5 ]=0
dp_coefficient[ 0 ][ 0 ][ 6 ]=0
dp_coefficient[ 0 ][ 0 ][ 7 ]=0
)");

    bitCodingTest(unit, 314, nalCaf, casps);
  }

  SECTION("Update light source extrinsics") {
    casps.casps_miv_extension().casme_depth_quantization_params_present_flag(false);
    casps.casps_miv_2_extension().casme_capture_device_information_present_flag(true);
    unit.came_update_extrinsics_flag(false)
        .came_update_intrinsics_flag(false)
        .miv_view_params_update_light_source_extrinsics()
        .mvpulse_num_updates_minus1(0)
        .mvpulse_view_idx(0, 0)
        .mvpulse_sensor_idx(0, 0)
        .light_source_extrinsics(0, 0)
        .lse_light_source_pos_x(1.0F)
        .lse_light_source_pos_y(1.0F)
        .lse_light_source_pos_z(1.0F)
        .lse_light_source_quat_x(1)
        .lse_light_source_quat_y(2)
        .lse_light_source_quat_z(3);

    REQUIRE(toString(unit, casps) == R"(came_update_extrinsics_flag=false
came_update_intrinsics_flag=false
came_update_sensor_extrinsics_flag=false
came_update_distortion_parameters_flag=false
came_update_light_source_extrinsics_flag=true
mvpulse_num_updates_minus1=0
mvpulse_view_idx[ 0 ]=0
mvpulse_sensor_idx[ 0 ]=0
lse_light_source_pos_x[ 0 ][ 0 ]=1
lse_light_source_pos_y[ 0 ][ 0 ]=1
lse_light_source_pos_z[ 0 ][ 0 ]=1
lse_light_source_quat_x[ 0 ][ 0 ]=1
lse_light_source_quat_y[ 0 ][ 0 ]=2
lse_light_source_quat_z[ 0 ][ 0 ]=3
)");

    bitCodingTest(unit, 245, nalCaf, casps);
  }
}

TEST_CASE("miv_view_params_update_extrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsUpdateExtrinsics{};

  SECTION("Example 1: Test with 1 update.") {
    unit.mvpue_num_view_updates_minus1(0);
    unit.mvpue_view_idx(0, 6)
        .camera_extrinsics(0)
        .ce_view_pos_x(3.F)
        .ce_view_pos_y(1.F)
        .ce_view_pos_z(4.F)
        .ce_view_quat_x(-2)
        .ce_view_quat_y(-1)
        .ce_view_quat_z(0);

    REQUIRE(toString(unit) == R"(mvpue_num_view_updates_minus1=0
mvpue_view_idx[ 0 ]=6
ce_view_pos_x[ 0 ]=3
ce_view_pos_y[ 0 ]=1
ce_view_pos_z[ 0 ]=4
ce_view_quat_x[ 0 ]=-2
ce_view_quat_y[ 0 ]=-1
ce_view_quat_z[ 0 ]=0
)");

    bitCodingTest(unit, 224);
  }
  SECTION("Example 1: Test with 2 update.") {
    unit.mvpue_num_view_updates_minus1(1);
    unit.mvpue_view_idx(0, 6)
        .camera_extrinsics(0)
        .ce_view_pos_x(3.F)
        .ce_view_pos_y(1.F)
        .ce_view_pos_z(4.F)
        .ce_view_quat_x(-1)
        .ce_view_quat_y(0)
        .ce_view_quat_z(-10);
    unit.mvpue_view_idx(1, 3)
        .camera_extrinsics(1)
        .ce_view_pos_x(7.F)
        .ce_view_pos_y(8.F)
        .ce_view_pos_z(3.F)
        .ce_view_quat_x(3)
        .ce_view_quat_y(6)
        .ce_view_quat_z(9);

    REQUIRE(toString(unit) == R"(mvpue_num_view_updates_minus1=1
mvpue_view_idx[ 0 ]=6
ce_view_pos_x[ 0 ]=3
ce_view_pos_y[ 0 ]=1
ce_view_pos_z[ 0 ]=4
ce_view_quat_x[ 0 ]=-1
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=-10
mvpue_view_idx[ 1 ]=3
ce_view_pos_x[ 1 ]=7
ce_view_pos_y[ 1 ]=8
ce_view_pos_z[ 1 ]=3
ce_view_quat_x[ 1 ]=3
ce_view_quat_y[ 1 ]=6
ce_view_quat_z[ 1 ]=9
)");

    bitCodingTest(unit, 432);
  }
}

TEST_CASE("miv_view_params_update_intrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsUpdateIntrinsics{};

  SECTION("Default constructor") {
    REQUIRE(toString(unit) == R"(mvpui_num_view_updates_minus1=0
mvpui_view_idx[ 0 ]=0
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=0
ci_erp_phi_max[ 0 ]=0
ci_erp_theta_min[ 0 ]=0
ci_erp_theta_max[ 0 ]=0
)");
    bitCodingTest(unit, 200);
  }

  SECTION("Example 1: Test with 1 update.") {
    unit.mvpui_num_view_updates_minus1(0);
    unit.mvpui_view_idx(0, 6)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::equirectangular)
        .ci_erp_phi_min(-2.F)
        .ci_erp_phi_max(2.F)
        .ci_erp_theta_min(-1.F)
        .ci_erp_theta_max(1.F);

    REQUIRE(toString(unit) == R"(mvpui_num_view_updates_minus1=0
mvpui_view_idx[ 0 ]=6
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=-2
ci_erp_phi_max[ 0 ]=2
ci_erp_theta_min[ 0 ]=-1
ci_erp_theta_max[ 0 ]=1
)");
    bitCodingTest(unit, 200);
  }

  SECTION("Example 1: Test with 2 updates.") {
    unit.mvpui_num_view_updates_minus1(1);
    unit.mvpui_view_idx(0, 3)
        .camera_intrinsics(0)
        .ci_cam_type(CiCamType::equirectangular)
        .ci_erp_phi_min(-90.F)
        .ci_erp_phi_max(90.F)
        .ci_erp_theta_min(-180.F)
        .ci_erp_theta_max(90.F);
    unit.mvpui_view_idx(1, 12)
        .camera_intrinsics(1)
        .ci_cam_type(CiCamType::orthographic)
        .ci_projection_plane_width_minus1(1023)
        .ci_projection_plane_height_minus1(767)
        .ci_ortho_width(100.F)
        .ci_ortho_height(50.F);

    REQUIRE(toString(unit) == R"(mvpui_num_view_updates_minus1=1
mvpui_view_idx[ 0 ]=3
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=0
ci_projection_plane_height_minus1[ 0 ]=0
ci_erp_phi_min[ 0 ]=-90
ci_erp_phi_max[ 0 ]=90
ci_erp_theta_min[ 0 ]=-180
ci_erp_theta_max[ 0 ]=90
mvpui_view_idx[ 1 ]=12
ci_cam_type[ 1 ]=orthographic
ci_projection_plane_width_minus1[ 1 ]=1023
ci_projection_plane_height_minus1[ 1 ]=767
ci_ortho_width[ 1 ]=100
ci_ortho_height[ 1 ]=50
)");
    bitCodingTest(unit, 320);
  }
}

TEST_CASE("miv_view_params_update_depth_quantization", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsUpdateDepthQuantization{};

  SECTION("Example 1: Test with 1 update.") {
    unit.mvpudq_num_view_updates_minus1(0);
    unit.mvpudq_view_idx(0, 6)
        .depth_quantization(0)
        .dq_depth_occ_threshold_default(64)
        .dq_norm_disp_low(1.F)
        .dq_norm_disp_high(100.F);

    REQUIRE(toString(unit) == R"(mvpudq_num_view_updates_minus1=0
mvpudq_view_idx[ 0 ]=6
dq_quantization_law[ 0 ]=0
dq_norm_disp_low[ 0 ]=1
dq_norm_disp_high[ 0 ]=100
dq_depth_occ_threshold_default[ 0 ]=64
)");
    bitCodingTest(unit, 110);
  }
}

TEST_CASE("sensor_extrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = SensorExtrinsics{};
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0F, 10.0F);

  unit.se_sensor_pos_x(dis(gen))
      .se_sensor_pos_y(dis(gen))
      .se_sensor_pos_z(dis(gen))
      .se_sensor_quat_x(int32_t{3})
      .se_sensor_quat_y(int32_t{-100})
      .se_sensor_quat_z(int32_t{10});

  SECTION("Copy and compare") {
    auto other = SensorExtrinsics{};

    other.se_sensor_pos_x(unit.se_sensor_pos_x())
        .se_sensor_pos_y(unit.se_sensor_pos_y())
        .se_sensor_pos_z(unit.se_sensor_pos_z())
        .se_sensor_quat_x(unit.se_sensor_quat_x())
        .se_sensor_quat_y(unit.se_sensor_quat_y())
        .se_sensor_quat_z(unit.se_sensor_quat_z());
    REQUIRE(unit == other);

    other.se_sensor_quat_z(unit.se_sensor_quat_z() + 1);
    REQUIRE(unit != other);
  }

  SECTION("To string") { toString(unit, uint16_t{0}, uint16_t{0}); }

  SECTION("Encoding random sensor extrinsics") { bitCodingTest(unit, 192); }
}

TEST_CASE("miv_view_params_update_sensor_extrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsUpdateSensorExtrinsics{};

  SECTION("Example 1: Test with updates.") {
    unit.mvpuse_num_updates_minus1(1);
    unit.mvpuse_view_idx(0, 0).mvpuse_view_idx(1, 1).mvpuse_sensor_idx(0, 0).mvpuse_sensor_idx(1,
                                                                                               2);
    unit.sensor_extrinsics(0, 1) = SensorExtrinsics{};
    unit.sensor_extrinsics(1, 2) = SensorExtrinsics{};
    REQUIRE(toString(unit) == R"(mvpuse_num_updates_minus1=1
mvpuse_view_idx[ 0 ]=0
mvpuse_sensor_idx[ 0 ]=0
se_sensor_pos_x[ 0 ][ 0 ]=0
se_sensor_pos_y[ 0 ][ 0 ]=0
se_sensor_pos_z[ 0 ][ 0 ]=0
se_sensor_quat_x[ 0 ][ 0 ]=0
se_sensor_quat_y[ 0 ][ 0 ]=0
se_sensor_quat_z[ 0 ][ 0 ]=0
mvpuse_view_idx[ 1 ]=1
mvpuse_sensor_idx[ 1 ]=2
se_sensor_pos_x[ 1 ][ 2 ]=0
se_sensor_pos_y[ 1 ][ 2 ]=0
se_sensor_pos_z[ 1 ][ 2 ]=0
se_sensor_quat_x[ 1 ][ 2 ]=0
se_sensor_quat_y[ 1 ][ 2 ]=0
se_sensor_quat_z[ 1 ][ 2 ]=0
)");
    bitCodingTest(unit, 464);
  }
}

TEST_CASE("distortion_parameters", "[Common Atlas Frame MIV Extension]") {
  auto unit = DistortionParameters{};
  auto other = DistortionParameters{};
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0F, 10.0F);

  SECTION("All model id") {
    std::vector<uint8_t> distortion_coefs = {0, 4, 5, 8};
    std::vector<int32_t> bits = {1, 131, 163, 261};
    for (uint8_t i = 0; i < 4; i++) {
      unit.dp_model_id(i);
      for (uint8_t j = 0; j < distortion_coefs[i]; j++) {
        unit.dp_coefficient(j, dis(gen));
      }

      other.dp_model_id(unit.dp_model_id());
      for (uint8_t j = 0; j < distortion_coefs[other.dp_model_id()]; j++) {
        other.dp_coefficient(j, unit.dp_coefficient(j));
      }
      REQUIRE(unit == other);

      if (i > 0) {
        other.dp_coefficient(0, unit.dp_coefficient(0) + 1.0F);
        REQUIRE(unit != other);
      }

      toString(unit, uint16_t{0}, uint16_t{0});
      bitCodingTest(unit, bits[i]);
    }
  }
}

TEST_CASE("miv_view_params_update_distortion_parameters", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsUpdateDistortionParameters{};

  SECTION("Example 1: Test with updates.") {
    unit.mvpudp_num_updates_minus1(1);
    unit.mvpudp_view_idx(0, 0).mvpudp_view_idx(1, 1).mvpudp_sensor_idx(0, 0).mvpudp_sensor_idx(1,
                                                                                               2);

    unit.distortion_parameters(0, 1) = DistortionParameters{};
    unit.distortion_parameters(1, 2) = DistortionParameters{}.dp_model_id(2);
    REQUIRE(toString(unit) == R"(mvpudp_num_updates_minus1=1
mvpudp_view_idx[ 0 ]=0
mvpudp_sensor_idx[ 0 ]=0
dp_model_id[ 0 ][ 0 ]=0
mvpudp_view_idx[ 1 ]=1
mvpudp_sensor_idx[ 1 ]=2
dp_model_id[ 1 ][ 2 ]=2
dp_coefficient[ 1 ][ 2 ][ 0 ]=0
dp_coefficient[ 1 ][ 2 ][ 1 ]=0
dp_coefficient[ 1 ][ 2 ][ 2 ]=0
dp_coefficient[ 1 ][ 2 ][ 3 ]=0
dp_coefficient[ 1 ][ 2 ][ 4 ]=0
)");
    bitCodingTest(unit, 244);
  }
}

TEST_CASE("light_source_extrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = LightSourceExtrinsics{};
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0F, 10.0F);

  unit.lse_light_source_pos_x(dis(gen))
      .lse_light_source_pos_y(dis(gen))
      .lse_light_source_pos_z(dis(gen))
      .lse_light_source_quat_x(int32_t{3})
      .lse_light_source_quat_y(int32_t{-100})
      .lse_light_source_quat_z(int32_t{10});

  SECTION("Copy and compare") {
    auto other = LightSourceExtrinsics{};

    other.lse_light_source_pos_x(unit.lse_light_source_pos_x())
        .lse_light_source_pos_y(unit.lse_light_source_pos_y())
        .lse_light_source_pos_z(unit.lse_light_source_pos_z())
        .lse_light_source_quat_x(unit.lse_light_source_quat_x())
        .lse_light_source_quat_y(unit.lse_light_source_quat_y())
        .lse_light_source_quat_z(unit.lse_light_source_quat_z());
    REQUIRE(unit == other);

    other.lse_light_source_quat_z(unit.lse_light_source_quat_z() + 1);
    REQUIRE(unit != other);
  }

  SECTION("To string") { toString(unit, uint16_t{0}, uint16_t{0}); }

  SECTION("Encoding random light source extrinsics") { bitCodingTest(unit, 192); }
}

TEST_CASE("miv_view_params_update_light_source_extrinsics", "[Common Atlas Frame MIV Extension]") {
  auto unit = MivViewParamsUpdateLightSourceExtrinsics{};

  SECTION("Example 1: Test with updates.") {
    unit.mvpulse_num_updates_minus1(1);
    unit.mvpulse_view_idx(0, 0).mvpulse_view_idx(1, 1).mvpulse_sensor_idx(0, 0).mvpulse_sensor_idx(
        1, 2);

    unit.light_source_extrinsics(0, 1) = LightSourceExtrinsics{};
    unit.light_source_extrinsics(1, 2) = LightSourceExtrinsics{};
    REQUIRE(toString(unit) == R"(mvpulse_num_updates_minus1=1
mvpulse_view_idx[ 0 ]=0
mvpulse_sensor_idx[ 0 ]=0
lse_light_source_pos_x[ 0 ][ 0 ]=0
lse_light_source_pos_y[ 0 ][ 0 ]=0
lse_light_source_pos_z[ 0 ][ 0 ]=0
lse_light_source_quat_x[ 0 ][ 0 ]=0
lse_light_source_quat_y[ 0 ][ 0 ]=0
lse_light_source_quat_z[ 0 ][ 0 ]=0
mvpulse_view_idx[ 1 ]=1
mvpulse_sensor_idx[ 1 ]=2
lse_light_source_pos_x[ 1 ][ 2 ]=0
lse_light_source_pos_y[ 1 ][ 2 ]=0
lse_light_source_pos_z[ 1 ][ 2 ]=0
lse_light_source_quat_x[ 1 ][ 2 ]=0
lse_light_source_quat_y[ 1 ][ 2 ]=0
lse_light_source_quat_z[ 1 ][ 2 ]=0
)");
    bitCodingTest(unit, 464);
  }
}

} // namespace TMIV::MivBitstream
