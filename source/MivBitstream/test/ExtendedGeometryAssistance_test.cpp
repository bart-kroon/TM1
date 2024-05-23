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

#include <TMIV/MivBitstream/ExtendedGeometryAssistance.h>
#include <TMIV/MivBitstream/ViewParamsList.h>

namespace TMIV::MivBitstream {
TEST_CASE("extended_geometry_assistance",
          "[EGA SEI payload syntax - block-based geometry features]") {
  SECTION("SINGLEVIEW") {
    const auto json_ega = TMIV::Common::Json::parse(R"({
	"ega_num_views_minus1":0,
	"ega_num_available_assistance_types_minus1":0,
	"view_idx_0":{
		"ega_assistance_present_flag":1,
		"ega_type_idx_0":{
			"ega_assistance_type_present_flag":1,
			"bbgf_qs":1,
			"bbgf_log2_bw_minus2":8,
			"bbgf_max_number_of_splits":2,
			"bbgf_projection_plane_height_minus1":1079,
			"bbgf_projection_plane_width_minus1":1919,
			"blocks":[
				[
					{
						"bbgf_split_flag":1,
						"bbgf_quad_split_flag":1,
						"subblks":[
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_zmin_delta":88,
								"bbgf_zmax_delta":297
							},
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_zmin_delta":0,
								"bbgf_zmax_delta":-145
							},
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_zmin_delta":-20,
								"bbgf_zmax_delta":393
							},
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_ltmin_flag":0,
								"bbgf_zmin_delta":0,
								"bbgf_ltmax_flag":0,
								"bbgf_zmax_delta":24
							}
						]
					},
					{
						"bbgf_split_flag":1,
						"bbgf_quad_split_flag":1,
						"subblks":[
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_zmin_delta":-76,
								"bbgf_zmax_delta":4
							},
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_zmin_delta":76,
								"bbgf_zmax_delta":165
							},
							{
								"bbgf_split_flag":0,
								"bbgf_skip_flag":0,
								"bbgf_ltmin_flag":1,
								"bbgf_zmin_delta":-12,
								"bbgf_ltmax_flag":0,
								"bbgf_zmax_delta":-28
							},
							{
								"bbgf_split_flag":1,
								"bbgf_quad_split_flag":0,
								"bbgf_split_orientation_flag":1,
								"bbgf_split_symmetry_flag":0,
								"bbgf_split_first_block_bigger":1,
								"subblks":[
									{
										"bbgf_skip_flag":0,
										"bbgf_ltmin_flag":0,
										"bbgf_zmin_delta":-53,
										"bbgf_ltmax_flag":0,
										"bbgf_zmax_delta":-76
									},
									{
										"bbgf_skip_flag":0,
										"bbgf_ltmin_flag":0,
										"bbgf_zmin_delta":-28,
										"bbgf_ltmax_flag":0,
										"bbgf_zmax_delta":-53
									}
								]
							}
						]
					}
				],
				[
					{
						"bbgf_split_flag":0,
						"bbgf_skip_flag":0,
						"bbgf_zmin_delta":88,
						"bbgf_zmax_delta":213
					},
					{
						"bbgf_split_flag":0,
						"bbgf_skip_flag":0,
						"bbgf_ltmin_flag":0,
						"bbgf_zmin_delta":-4,
						"bbgf_ltmax_flag":0,
						"bbgf_zmax_delta":-221
					}
				]
			]
		}
	}
})");

    auto ega = TMIV::MivBitstream::ExtendedGeometryAssistance::readFrom(json_ega);
    std::ostringstream text;
    ega.writeTo(text);

    REQUIRE(text.str() ==
            R"(ega_num_views_minus1=0
ega_num_available_assistance_types_minus1=0
# VIEWIDX 0
ega_assistance_present_flag[0]=1
# EGATYPEIDX 0
ega_assistance_type_present_flag[0][0]=1
bbgf_qs[0]=1
bbgf_log2_bw_minus2[0]=8
bbgf_max_number_of_splits[0]=2
bbgf_projection_plane_height_minus1[0]=1079
bbgf_projection_plane_width_minus1[0]=1919
block y=0 x=0 bbgf_split_flag=1 bbgf_quad_split_flag=1 [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_zmin_delta=88 bbgf_zmax_delta=297 ] [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_zmin_delta=0 bbgf_zmax_delta=-145 ] [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_zmin_delta=-20 bbgf_zmax_delta=393 ] [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_ltmin_flag=0 bbgf_ltmax_flag=0 bbgf_zmin_delta=0 bbgf_zmax_delta=24 ]
block y=0 x=1 bbgf_split_flag=1 bbgf_quad_split_flag=1 [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_zmin_delta=-76 bbgf_zmax_delta=4 ] [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_zmin_delta=76 bbgf_zmax_delta=165 ] [ bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_ltmin_flag=1 bbgf_ltmax_flag=0 bbgf_zmin_delta=-12 bbgf_zmax_delta=-28 ] [ bbgf_split_flag=1 bbgf_quad_split_flag=0 bbgf_split_orientation_flag=1 bbgf_split_symmetry_flag=0 bbgf_split_first_block_bigger=1 [ bbgf_skip_flag=0 bbgf_ltmin_flag=0 bbgf_ltmax_flag=0 bbgf_zmin_delta=-53 bbgf_zmax_delta=-76 ] [ bbgf_skip_flag=0 bbgf_ltmin_flag=0 bbgf_ltmax_flag=0 bbgf_zmin_delta=-28 bbgf_zmax_delta=-53 ] ]
block y=1 x=0 bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_zmin_delta=88 bbgf_zmax_delta=213
block y=1 x=1 bbgf_split_flag=0 bbgf_skip_flag=0 bbgf_ltmin_flag=0 bbgf_ltmax_flag=0 bbgf_zmin_delta=-4 bbgf_zmax_delta=-221
)");
    bitCodingTest(ega, 374);
  }
}
} // namespace TMIV::MivBitstream
