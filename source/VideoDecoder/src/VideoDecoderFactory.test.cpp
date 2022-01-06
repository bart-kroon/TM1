/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/VideoDecoder/VideoDecoderFactory.h>

#if HAVE_HM
#include <TMIV/VideoDecoder/HmVideoDecoder.h>
#endif

#if HAVE_VVDEC
#include <TMIV/VideoDecoder/VVdeCVideoDecoder.h>
#endif

using namespace std::string_literals;

TEST_CASE("VideoDecoder::VideoDecoderFactory") {
  using Catch::Contains;
  using TMIV::VideoDecoder::create;
  using TMIV::VideoDecoder::DecoderId;

  const auto nalUnitSource = []() { return ""s; };

  const auto checkNoSupport = [=](const auto codecGroupIdc) {
    REQUIRE_THROWS_WITH(create(nalUnitSource, codecGroupIdc), Contains("no built-in support"));
  };

  SECTION("HEVC Main10 has optional built-in support based on the HEVC Test Model (HM)") {
#if HAVE_HM
    const auto decoder = create(nalUnitSource, DecoderId::HEVC_Main10);
    REQUIRE(dynamic_cast<TMIV::VideoDecoder::HmVideoDecoder *>(decoder.get()) != nullptr);
#else
    checkNoSupport(DecoderId::HEVC_Main10);
#endif
  }

  SECTION("VVC Main10 has optional built-in support based on the VVdeC decoder") {
#if HAVE_VVDEC
    const auto decoder = create(nalUnitSource, DecoderId::VVC_Main10);
    REQUIRE(dynamic_cast<TMIV::VideoDecoder::VVdeCVideoDecoder *>(decoder.get()) != nullptr);
#else
    checkNoSupport(DecoderId::VVC_Main10);
#endif
  }

  SECTION("All other codec group IDC's are not (yet) supported") {
    const auto decoderId =
        GENERATE(DecoderId::AVC_Progressive_High, DecoderId::HEVC444, DecoderId{33});

    checkNoSupport(decoderId);

    SECTION("Check the full error message because there has been a formatting problem") {
      REQUIRE_THROWS_WITH(create(nalUnitSource, DecoderId::AVC_Progressive_High),
                          "There is no built-in support for the AVC Progressive High codec");
    }
  }
}
