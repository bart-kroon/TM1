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

#include <TMIV/MivBitstream/MivPatchUnit.h>

using namespace TMIV::MivBitstream;

TEST_CASE("miv_patch_unit", "[MIV Patch Unit]") {
  auto pdu = PatchDataUnit{};
  auto x = MivPatchUnit{pdu};
  REQUIRE(toString(x, 856) == R"(pdu_2d_pos_x( 856 )=0
pdu_2d_pos_y( 856 )=0
pdu_2d_size_x( 856 )=0
pdu_2d_size_y( 856 )=0
mpu_view_pos_x( 856 )=0
mpu_view_pos_y( 856 )=0
mpu_view_id( 856 )=0
pdu_orientation_index( 856 )=FPO_NULL
)");

  SECTION("Compatibility check") {
    const auto pdu = x.patch_data_unit();
    const auto y = MivPatchUnit{pdu};
    REQUIRE(x == y);
  }

  SECTION("Example") {
    x.pdu_2d_pos_x(1)
        .pdu_2d_pos_y(2)
        .pdu_2d_size_x(3)
        .pdu_2d_size_y(5)
        .mpu_view_pos_x(8)
        .mpu_view_pos_y(13)
        .mpu_view_id(21)
        .pdu_orientation_index(FlexiblePatchOrientation::FPO_ROT270);

    REQUIRE(toString(x, 856) == R"(pdu_2d_pos_x( 856 )=1
pdu_2d_pos_y( 856 )=2
pdu_2d_size_x( 856 )=3
pdu_2d_size_y( 856 )=5
mpu_view_pos_x( 856 )=8
mpu_view_pos_y( 856 )=13
mpu_view_id( 856 )=21
pdu_orientation_index( 856 )=FPO_ROT270
)");

    SECTION("Compatibility check") {
      const auto pdu = x.patch_data_unit();
      const auto y = MivPatchUnit{pdu};
      REQUIRE(x == y);
    }
  }
}
