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

#include <catch2/catch.hpp>

#include <TMIV/Common/verify.h>

using std::is_base_of_v;
using std::runtime_error;
using TMIV::Common::BitstreamError;
using TMIV::Common::bitstreamError;
using TMIV::Common::MivBitstreamError;
using TMIV::Common::mivBitstreamError;
using TMIV::Common::PtlError;
using TMIV::Common::runtimeError;
using TMIV::Common::V3cBitstreamError;
using TMIV::Common::v3cBitstreamError;

// NOTE(BK): We know that there are non-returning function calls in below test case, but we want to
// test if exceptions of the right type are thrown using REQUIRE_THROWS_AS. Disabling C4702 for the
// remainder of this translation unit.
#if _MSC_VER
#pragma warning(disable : 4702)
#endif

TEST_CASE("Verify Macros") {
  SECTION("Pass on true conditions") {
    REQUIRE_NOTHROW(VERIFY(true));
    REQUIRE_NOTHROW(VERIFY_V3CBITSTREAM(true));
    REQUIRE_NOTHROW(VERIFY_MIVBITSTREAM(true));
    REQUIRE_NOTHROW(VERIFY_BITSTREAM(true));

    REQUIRE_NOTHROW(LIMITATION(true));
    REQUIRE_NOTHROW(PRECONDITION(true));
    REQUIRE_NOTHROW(POSTCONDITION(true));
  }

  SECTION("Raise an exception on false conditions") {
    SECTION("Exception hierarchy") {
      static_assert(is_base_of_v<BitstreamError, V3cBitstreamError>);
      static_assert(is_base_of_v<BitstreamError, MivBitstreamError>);
      static_assert(is_base_of_v<runtime_error, BitstreamError>);
      static_assert(is_base_of_v<runtime_error, PtlError>);
    }

    SECTION("Runtime error") {
      REQUIRE_THROWS_AS(runtimeError("", "", 0), runtime_error);
      REQUIRE_THROWS_AS(VERIFY(false), runtime_error);
      REQUIRE_THROWS_AS(RUNTIME_ERROR(""), runtime_error);
    }

    SECTION("V3C bitstream error") {
      REQUIRE_THROWS_AS(v3cBitstreamError("", "", 0), V3cBitstreamError);
      REQUIRE_THROWS_AS(VERIFY_V3CBITSTREAM(false), V3cBitstreamError);
      REQUIRE_THROWS_AS(V3CBITSTREAM_ERROR(""), V3cBitstreamError);
    }

    SECTION("MIV bitstream error") {
      REQUIRE_THROWS_AS(mivBitstreamError("", "", 0), MivBitstreamError);
      REQUIRE_THROWS_AS(VERIFY_MIVBITSTREAM(false), MivBitstreamError);
      REQUIRE_THROWS_AS(MIVBITSTREAM_ERROR(""), MivBitstreamError);
    }

    SECTION("Bitstream error") {
      REQUIRE_THROWS_AS(bitstreamError("", "", 0), BitstreamError);
      REQUIRE_THROWS_AS(VERIFY_BITSTREAM(false), BitstreamError);
      REQUIRE_THROWS_AS(BITSTREAM_ERROR(""), BitstreamError);
    }

    // NOTE(BK): Because Catch2 does not support dead testing, LIMITATION, PRECONDITION and
    // POSTCONDITION cannot be fully tested.
  }
}
