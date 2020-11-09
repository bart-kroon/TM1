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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <TMIV/Decoder/AccessUnit.h>

TEST_CASE("Test if patches are within the projection plane bounds") {
  auto vpl = TMIV::MivBitstream::ViewParamsList{};
  auto ppl = TMIV::MivBitstream::PatchParamsList{};

  SECTION("Empty lists") { TMIV::Decoder::requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl); }

  SECTION("Valid lists") {
    vpl.emplace_back().ci.ci_projection_plane_width_minus1(7).ci_projection_plane_height_minus1(5);
    vpl.emplace_back().ci.ci_projection_plane_width_minus1(9).ci_projection_plane_height_minus1(7);

    ppl.emplace_back()
        .atlasPatchOrientationIndex(TMIV::MivBitstream::FlexiblePatchOrientation::FPO_NULL)
        .atlasPatch3dOffsetU(3)
        .atlasPatch3dOffsetV(2)
        .atlasPatch3dSizeU(4)
        .atlasPatch3dSizeV(4)
        .atlasPatchProjectionId(0);
    ppl.emplace_back()
        .atlasPatchOrientationIndex(TMIV::MivBitstream::FlexiblePatchOrientation::FPO_SWAP)
        .atlasPatch3dOffsetU(4)
        .atlasPatch3dOffsetV(0)
        .atlasPatch3dSizeU(6)
        .atlasPatch3dSizeV(8)
        .atlasPatchProjectionId(1);

    REQUIRE(ppl.size() == 2);
    CHECK(ppl[0].atlasPatchProjectionId() == 0);
    CHECK(ppl[1].atlasPatchProjectionId() == 1);

    TMIV::Decoder::requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl);

    SECTION("Invalid list: projection ID out of bounds") {
      ppl[1].atlasPatchProjectionId(2);

      REQUIRE_THROWS(TMIV::Decoder::requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }

    SECTION("Invalid list: projection ID swap") {
      ppl[0].atlasPatchProjectionId(1);
      ppl[1].atlasPatchProjectionId(0);

      REQUIRE_THROWS(TMIV::Decoder::requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }

    SECTION("Invalid list: offset change") {
      ppl[1].atlasPatch3dOffsetU(5);

      REQUIRE_THROWS(TMIV::Decoder::requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }

    SECTION("Invalid list: size change") {
      ppl[0].atlasPatch3dSizeV(7);

      REQUIRE_THROWS(TMIV::Decoder::requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }
  }
}
