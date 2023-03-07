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

#include <TMIV/MivBitstream/GeometryAssistance.h>
#include <TMIV/MivBitstream/ViewParamsList.h>

namespace TMIV::MivBitstream {
TEST_CASE("geometry_assistance", "[Geometry assistance SEI payload syntax]") {
  SECTION("SINGLEVIEW") {
    const auto json_ga = TMIV::Common::Json::parse(R"(
    {"gas_qs":1,"gas_log2_bw_minus2":8
    ,"gas_num_views_minus1":0
    ,"view_idx_0":{
    "gas_projection_plane_height_minus1":1079
    ,"gas_projection_plane_width_minus1":1919
    ,"blocks":[[
     {"gas_split_flag":1 ,"gas_quad_split_flag":1, "subblks": [{ "gas_skip_flag": 0 ,"gas_zmin_delta":88 ,"gas_zmax_delta":297},{ "gas_skip_flag": 0 ,"gas_zmin_delta":0 ,"gas_zmax_delta":-145},{ "gas_skip_flag": 0 ,"gas_zmin_delta":-20 ,"gas_zmax_delta":393},{ "gas_skip_flag": 0 ,"gas_ltmin_flag":0 ,"gas_zmin_delta":0 ,"gas_ltmax_flag":0 ,"gas_zmax_delta":24}]}
     , {"gas_split_flag":1 ,"gas_quad_split_flag":1, "subblks": [{ "gas_skip_flag": 0 ,"gas_zmin_delta":-76 ,"gas_zmax_delta":4},{ "gas_skip_flag": 0 ,"gas_zmin_delta":76 ,"gas_zmax_delta":165},{ "gas_skip_flag": 0 ,"gas_ltmin_flag":1 ,"gas_zmin_delta":-12 ,"gas_ltmax_flag":0 ,"gas_zmax_delta":-28},{ "gas_skip_flag": 0 ,"gas_ltmin_flag":1 ,"gas_zmin_delta":-20 ,"gas_ltmax_flag":1 ,"gas_zmax_delta":-4}]}
     ]
     ,[
      {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 0 ,"gas_zmin_delta":88 ,"gas_zmax_delta":213}]}
      , {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 0 ,"gas_ltmin_flag":0 ,"gas_zmin_delta":-4 ,"gas_ltmax_flag":0 ,"gas_zmax_delta":-221}]}
      ]
      ]}
      })");
    auto ga = TMIV::MivBitstream::GeometryAssistance::readFrom(json_ga);
    std::ostringstream text;
    ga.writeTo(text);

    REQUIRE(text.str() ==
            R"(gas_qs=1
gas_num_views_minus1=0
gas_log2_bw_minus2=8
# VIEWIDX 0
gas_projection_plane_height_minus1[0]=1079
gas_projection_plane_width_minus1[0]=1919
block y=0 x=0 gas_split_flag=1 gas_quad_split_flag=1 [gas_skip_flag=0,gas_zmin_delta=88,gas_zmax_delta=297] [gas_skip_flag=0,gas_zmin_delta=0,gas_zmax_delta=-145] [gas_skip_flag=0,gas_zmin_delta=-20,gas_zmax_delta=393] [gas_skip_flag=0,gas_ltmin_flag=0,gas_ltmax_flag=0,gas_zmin_delta=0,gas_zmax_delta=24]
block y=0 x=1 gas_split_flag=1 gas_quad_split_flag=1 [gas_skip_flag=0,gas_zmin_delta=-76,gas_zmax_delta=4] [gas_skip_flag=0,gas_zmin_delta=76,gas_zmax_delta=165] [gas_skip_flag=0,gas_ltmin_flag=1,gas_ltmax_flag=0,gas_zmin_delta=-12,gas_zmax_delta=-28] [gas_skip_flag=0,gas_ltmin_flag=1,gas_ltmax_flag=1,gas_zmin_delta=-20,gas_zmax_delta=-4]
block y=1 x=0 gas_split_flag=0 [gas_skip_flag=0,gas_zmin_delta=88,gas_zmax_delta=213]
block y=1 x=1 gas_split_flag=0 [gas_skip_flag=0,gas_ltmin_flag=0,gas_ltmax_flag=0,gas_zmin_delta=-4,gas_zmax_delta=-221]
)");
    REQUIRE(bitCodingTest(ga, 319));
  }
}
} // namespace TMIV::MivBitstream
