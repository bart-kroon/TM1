/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("casps_miv_extension") {
  SECTION("Default constructor") {
    const CaspsMivExtension unit{};

    REQUIRE(toString(unit) == R"(casme_depth_low_quality_flag=false
casme_depth_quantization_params_present_flag=true
casme_vui_params_present_flag=false
)");

    bitCodingTest(unit, 3);
  }

  SECTION("Include default VUI Parameters") {
    CaspsMivExtension unit{};
    unit.casme_depth_low_quality_flag(true)
        .casme_depth_quantization_params_present_flag(false)
        .vui_parameters({});

    REQUIRE(toString(unit) == R"(casme_depth_low_quality_flag=true
casme_depth_quantization_params_present_flag=false
casme_vui_params_present_flag=true
vui_timing_info_present_flag=false
vui_tiles_restriction_present_flag=false
vui_max_coded_video_resolution_present_flag=false
vui_coordinate_system_parameters_present_flag=false
vui_unit_in_metres_flag=false
vui_display_box_info_present_flag=false
vui_anchor_point_present_flag=false
)");

    bitCodingTest(unit, 10);
  }
}

TEST_CASE("casps_miv_2_extension") {
  SECTION("Default constructor") {
    const CaspsMiv2Extension unit{};

    REQUIRE(toString(unit) == R"(casme_decoder_side_depth_estimation_flag=false
casme_chroma_scaling_present_flag=false
casme_capture_device_information_present_flag=false
casme_background_separation_enable_flag=false
)");

    bitCodingTest(unit, 12);
  }

  SECTION("Example 1") {
    CaspsMiv2Extension unit{};
    unit.casme_decoder_side_depth_estimation_flag(true);

    REQUIRE(toString(unit) == R"(casme_decoder_side_depth_estimation_flag=true
casme_chroma_scaling_present_flag=false
casme_capture_device_information_present_flag=false
casme_background_separation_enable_flag=false
)");

    bitCodingTest(unit, 12);
  }

  SECTION("Example 2") {
    CaspsMiv2Extension unit{};
    unit.casme_chroma_scaling_bit_depth_minus1(17);
    unit.casme_capture_device_information_present_flag(true)
        .capture_device_information()
        .cdi_device_model_count_minus1(0);

    REQUIRE(toString(unit) == R"(casme_decoder_side_depth_estimation_flag=false
casme_chroma_scaling_present_flag=true
casme_chroma_scaling_bit_depth_minus1=17
casme_capture_device_information_present_flag=true
cdi_device_model_count_minus1=0
cdi_device_model_id[0]=0
cdi_device_class_id[0]=0
casme_background_separation_enable_flag=false
)");

    bitCodingTest(unit, 25);
  }
}

TEST_CASE("common_atlas_sequence_parameter_set_rbsp",
          "[Common Atlas Sequence Parameter Set RBSP]") {
  SECTION("Default Constructor") {
    const CommonAtlasSequenceParameterSetRBSP unit{};

    REQUIRE(toString(unit) == R"(casps_common_atlas_sequence_parameter_set_id=0
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=0
casps_extension_present_flag=false
)");
    byteCodingTest(unit, 1);
  }

  CommonAtlasSequenceParameterSetRBSP unit{};
  SECTION("Extension present, MIV extension flag unset") {
    unit.casps_miv_extension_present_flag(false).casps_extension_6bits(0);

    REQUIRE(toString(unit) == R"(casps_common_atlas_sequence_parameter_set_id=0
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=0
casps_extension_present_flag=true
casps_miv_extension_present_flag=false
casps_miv_2_extension_present_flag=false
casps_extension_6bits=0
)");
    byteCodingTest(unit, 2);
  }

  SECTION("Extension present, MIV extension flag true") {
    unit.casps_common_atlas_sequence_parameter_set_id(5)
        .casps_extension_6bits(0)
        .casps_miv_extension() = CaspsMivExtension{};

    REQUIRE(toString(unit) == R"(casps_common_atlas_sequence_parameter_set_id=5
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=0
casps_extension_present_flag=true
casps_miv_extension_present_flag=true
casps_miv_2_extension_present_flag=false
casps_extension_6bits=0
casme_depth_low_quality_flag=false
casme_depth_quantization_params_present_flag=true
casme_vui_params_present_flag=false
)");
    byteCodingTest(unit, 3);

    SECTION("Add MIV 2 extension") {
      unit.casps_miv_2_extension().casme_decoder_side_depth_estimation_flag(true);

      REQUIRE(toString(unit) == R"(casps_common_atlas_sequence_parameter_set_id=5
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=0
casps_extension_present_flag=true
casps_miv_extension_present_flag=true
casps_miv_2_extension_present_flag=true
casps_extension_6bits=0
casme_depth_low_quality_flag=false
casme_depth_quantization_params_present_flag=true
casme_vui_params_present_flag=false
casme_decoder_side_depth_estimation_flag=true
casme_chroma_scaling_present_flag=false
casme_capture_device_information_present_flag=false
casme_background_separation_enable_flag=false
)");
      byteCodingTest(unit, 4);
    }
  }

  SECTION("Extension present, casps_extension_6bits nonzero") {
    unit.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(3)
        .casps_miv_extension_present_flag(false)
        .casps_extension_6bits(63)
        .caspsExtensionData({true, true, false});

    REQUIRE(toString(unit) == R"(casps_common_atlas_sequence_parameter_set_id=0
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=3
casps_extension_present_flag=true
casps_miv_extension_present_flag=false
casps_miv_2_extension_present_flag=false
casps_extension_6bits=63
casps_extension_data_flag=true
casps_extension_data_flag=true
casps_extension_data_flag=false
)");
    byteCodingTest(unit, 3);
  }
}
} // namespace TMIV::MivBitstream
