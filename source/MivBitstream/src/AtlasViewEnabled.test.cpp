/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/MivBitstream/AtlasViewEnabled.h>

namespace TMIV::MivBitstream {
TEST_CASE("atlas_view_enabled", "[Atlas view enabled SEI payload syntax]") {
  SECTION("Null example") {
    const auto x = AtlasViewEnabled{};
    REQUIRE(toString(x) == R"(ave_cancel_flag=true
)");
    REQUIRE(bitCodingTest(x, 1));
  }

  SECTION("1 Atlas - 5 Views where only 4 enabled and 2 of them are complete") {
    auto x = AtlasViewEnabled{};
    x.ave_cancel_flag(false)
        .ave_persistence_flag(false)
        .ave_atlas_count_minus1(0)
        .ave_num_views_minus1(4)
        .ave_atlas_id(0, 0)
        .ave_view_enabled_in_atlas_flag(0, 0, true)
        .ave_view_enabled_in_atlas_flag(0, 1, true)
        .ave_view_enabled_in_atlas_flag(0, 2, false)
        .ave_view_enabled_in_atlas_flag(0, 3, true)
        .ave_view_enabled_in_atlas_flag(0, 4, true)
        .ave_view_complete_in_atlas_flag(0, 0, false)
        .ave_view_complete_in_atlas_flag(0, 1, true)
        .ave_view_complete_in_atlas_flag(0, 3, true)
        .ave_view_complete_in_atlas_flag(0, 4, false);

    REQUIRE(toString(x) == R"(ave_cancel_flag=false
ave_persistence_flag=false
ave_atlas_count_minus1=0
ave_num_views_minus1=4
ave_atlas_id[ 0 ]=0
ave_view_enabled_in_atlas_flag[ 0 ][ 0 ]=true
ave_view_complete_in_atlas_flag[ 0 ][ 0 ]=false
ave_view_enabled_in_atlas_flag[ 0 ][ 1 ]=true
ave_view_complete_in_atlas_flag[ 0 ][ 1 ]=true
ave_view_enabled_in_atlas_flag[ 0 ][ 2 ]=false
ave_view_enabled_in_atlas_flag[ 0 ][ 3 ]=true
ave_view_complete_in_atlas_flag[ 0 ][ 3 ]=true
ave_view_enabled_in_atlas_flag[ 0 ][ 4 ]=true
ave_view_complete_in_atlas_flag[ 0 ][ 4 ]=false
)");
    REQUIRE(bitCodingTest(x, 33));
  }

  SECTION("3 Atlases - 2 Views each where only views in 2nd atlas are complete") {
    auto x = AtlasViewEnabled{};
    x.ave_cancel_flag(false)
        .ave_persistence_flag(false)
        .ave_atlas_count_minus1(2)
        .ave_num_views_minus1(1)
        .ave_atlas_id(0, 0)
        .ave_atlas_id(1, 1)
        .ave_atlas_id(2, 2)
        .ave_view_enabled_in_atlas_flag(0, 0, true)
        .ave_view_enabled_in_atlas_flag(0, 1, true)
        .ave_view_enabled_in_atlas_flag(1, 0, true)
        .ave_view_enabled_in_atlas_flag(1, 1, true)
        .ave_view_enabled_in_atlas_flag(2, 0, false)
        .ave_view_enabled_in_atlas_flag(2, 1, false)
        .ave_view_complete_in_atlas_flag(0, 0, false)
        .ave_view_complete_in_atlas_flag(0, 1, false)
        .ave_view_complete_in_atlas_flag(1, 0, true)
        .ave_view_complete_in_atlas_flag(1, 1, true);

    REQUIRE(toString(x) == R"(ave_cancel_flag=false
ave_persistence_flag=false
ave_atlas_count_minus1=2
ave_num_views_minus1=1
ave_atlas_id[ 0 ]=0
ave_view_enabled_in_atlas_flag[ 0 ][ 0 ]=true
ave_view_complete_in_atlas_flag[ 0 ][ 0 ]=false
ave_view_enabled_in_atlas_flag[ 0 ][ 1 ]=true
ave_view_complete_in_atlas_flag[ 0 ][ 1 ]=false
ave_atlas_id[ 1 ]=1
ave_view_enabled_in_atlas_flag[ 1 ][ 0 ]=true
ave_view_complete_in_atlas_flag[ 1 ][ 0 ]=true
ave_view_enabled_in_atlas_flag[ 1 ][ 1 ]=true
ave_view_complete_in_atlas_flag[ 1 ][ 1 ]=true
ave_atlas_id[ 2 ]=2
ave_view_enabled_in_atlas_flag[ 2 ][ 0 ]=false
ave_view_enabled_in_atlas_flag[ 2 ][ 1 ]=false
)");
    REQUIRE(bitCodingTest(x, 40));
  }
}
} // namespace TMIV::MivBitstream
