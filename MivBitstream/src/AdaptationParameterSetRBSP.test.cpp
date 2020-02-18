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

TEST_CASE("miv_view_params_list", "[Adaptation Parameter Set RBSP]") {
  auto x = MivViewParamsList{};

  SECTION("Example 1") {
    x.mvp_num_views_minus1(0)
        .mvp_intrinsic_params_equal_flag(false)
        .mvp_depth_quantization_params_equal_flag(false)
        .mvp_pruning_graph_params_present_flag(false);

    REQUIRE(toString(x) == R"(mvp_num_views_minus1=0
mvp_intrinsic_params_equal_flag=false
mvp_depth_quantization_params_equal_flag=false
mvp_pruning_graph_params_present_flag=false
)");

    REQUIRE(bitCodingTest(x, 19));
  }

  SECTION("Example 2") {
    x.mvp_num_views_minus1(2)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true);

    REQUIRE(toString(x) == R"(mvp_num_views_minus1=2
mvp_intrinsic_params_equal_flag=true
mvp_depth_quantization_params_equal_flag=true
mvp_pruning_graph_params_present_flag=true
)");

    REQUIRE(bitCodingTest(x, 19));
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
    x.aps_miv_view_params_list_update_mode(MvplUpdateMode::VPL_INITLIST);
    x.miv_view_params_list() = MivViewParamsList{};
    x.miv_view_params_list()
        .mvp_num_views_minus1(2)
        .mvp_intrinsic_params_equal_flag(true)
        .mvp_depth_quantization_params_equal_flag(true)
        .mvp_pruning_graph_params_present_flag(true);

    REQUIRE(toString(x) == R"(aps_adaptation_parameter_set_id=63
aps_camera_params_present_flag=false
aps_miv_view_params_list_present_flag=true
aps_miv_view_params_list_update_mode=VPL_INITLIST
mvp_num_views_minus1=2
mvp_intrinsic_params_equal_flag=true
mvp_depth_quantization_params_equal_flag=true
mvp_pruning_graph_params_present_flag=true
aps_extension2_flag=false
)");

    REQUIRE(byteCodingTest(x, 5));
  }
}
