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

#include <TMIV/VideoDecoder/VideoDecoderBase.h>

using namespace std::string_literals;

namespace {
using TMIV::VideoDecoder::NalUnitSource;
using TMIV::VideoDecoder::VideoDecoderBase;

auto decodedFrame() {
  return TMIV::Common::Frame<>{{20, 10}, 7, TMIV::Common::ColorFormat::YUV444};
}

struct FakeVideoDecoder : public VideoDecoderBase {
  FakeVideoDecoder(NalUnitSource source) : VideoDecoderBase{std::move(source)} {}

  int32_t decodeSomeCallCount{};

  auto decodeSome() -> bool final {
    switch (decodeSomeCallCount++) {
    case 0:
      takeNalUnit();
      return true;
    case 1:
      takeNalUnit();
      outputFrame(decodedFrame());
      outputFrame(decodedFrame());
      return true;
    case 2:
      outputFrame(decodedFrame());
      return true;
    case 3:
      takeNalUnit();
      return true;
    case 4:
      outputFrame(decodedFrame());
      return false;
    default:
      FAIL("It is not allowed to call decodeSome() after it returned false");
      return false;
    }
  }
};
} // namespace

TEST_CASE("VideoDecoder::VideoDecoderBase") {
  const auto nalUnitSource = [i = 0]() mutable {
    switch (i++) {
    case 0:
      return "[ NAL unit 1 ]"s;
    case 1:
      return "[ NAL unit 2 ]"s;
    case 2:
      return ""s;
    default:
      FAIL("It is not allowed to call the NAL unit source after it returned an empty string");
      return ""s;
    }
  };

  auto unit = FakeVideoDecoder{nalUnitSource};

  REQUIRE(unit.decodeSomeCallCount == 0);

  REQUIRE_FALSE(unit.getFrame().empty());
  REQUIRE(unit.decodeSomeCallCount == 2);

  REQUIRE_FALSE(unit.getFrame().empty());
  REQUIRE(unit.decodeSomeCallCount == 2);

  REQUIRE_FALSE(unit.getFrame().empty());
  REQUIRE(unit.decodeSomeCallCount == 3);

  REQUIRE_FALSE(unit.getFrame().empty());
  REQUIRE(unit.decodeSomeCallCount == 5);

  REQUIRE(unit.getFrame().empty());
  REQUIRE(unit.decodeSomeCallCount == 5);
}
