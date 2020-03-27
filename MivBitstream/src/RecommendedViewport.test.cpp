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

#include <TMIV/MivBitstream/RecommendedViewport.h>

using namespace TMIV::MivBitstream;

TEST_CASE("rec_viewport", "[Recommended Viewport SEI payload syntax]") {

  SECTION("Example 1") {
    const auto x = RecViewport{{0, 1}};
    REQUIRE(toString(x) == R"(rec_viewport_id=0
rec_viewport_cancel_flag( 0 )=1
)");
    REQUIRE(bitCodingTest(x, 11));
  }

  SECTION("Example 2") {
    const auto x =
        RecViewport{{1, 0, 0, 1, {}, 0.2f, 1.45f, -0.79f, -0.91f, 0.0f, 1.2f, 90.0f, 60.4f}};
    REQUIRE(toString(x) == R"(rec_viewport_id=1
rec_viewport_cancel_flag( 1 )=0
rec_viewport_persistence_flag( 1 )=0
rec_viewport_center_view_flag( 1 )=1
rec_viewport_pos_x( 1 )=0.2
rec_viewport_pos_y( 1 )=1.45
rec_viewport_pos_z( 1 )=-0.79
rec_viewport_quat_x( 1 )=-0.91
rec_viewport_quat_y( 1 )=0
rec_viewport_quat_z( 1 )=1.2
rec_viewport_hor_range( 1 )=90
rec_viewport_ver_range( 1 )=60.4
)");
    REQUIRE(bitCodingTest(x, 269));
  }

  SECTION("Example 3") {
    const auto x =
        RecViewport{{2, 0, 0, 0, 1, 0.2f, 1.45f, -0.79f, -0.91f, 0.0f, 1.2f, 90.0f, 60.4f}};
    REQUIRE(toString(x) == R"(rec_viewport_id=2
rec_viewport_cancel_flag( 2 )=0
rec_viewport_persistence_flag( 2 )=0
rec_viewport_center_view_flag( 2 )=0
rec_viewport_left_view_flag( 2 )=1
rec_viewport_pos_x( 2 )=0.2
rec_viewport_pos_y( 2 )=1.45
rec_viewport_pos_z( 2 )=-0.79
rec_viewport_quat_x( 2 )=-0.91
rec_viewport_quat_y( 2 )=0
rec_viewport_quat_z( 2 )=1.2
rec_viewport_hor_range( 2 )=90
rec_viewport_ver_range( 2 )=60.4
)");
    REQUIRE(bitCodingTest(x, 270));
  }
}
