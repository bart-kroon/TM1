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

#include <catch2/catch_test_macros.hpp>

#include <TMIV/Decoder/DecodeNalUnitStream.h>

using namespace std::string_literals;

namespace test {
namespace {
template <typename Unit> auto exampleSource() {
  return [n = 0]() mutable -> std::optional<Unit> {
    switch (n++) {
    case 0:
      // Bitstream with one NAL unit "Hello"
      return Unit{"\0\0\0\5Hello"s};
    case 1:
      // Bitstream with two NAL units ", " and "world!"
      return Unit{"\0\0\0\2, \0\0\0\6world!"s};
    case 2:
      return std::nullopt;
    default:
      UNREACHABLE;
    }
  };
}
} // namespace
} // namespace test

TEST_CASE("TMIV::Decoder::decodeNalUnitStream") {
  using TMIV::Decoder::decodeNalUnitStream;

  SECTION("Empty source") {
    auto source = decodeNalUnitStream(TMIV::Common::emptySource<std::string>());
    REQUIRE_FALSE(source());
  }

  SECTION("Some NAL units") {
    auto source = decodeNalUnitStream(test::exampleSource<std::string>());
    auto message = std::string{};

    while (auto unit = source()) {
      message += *unit;
    }

    CHECK(message == "Hello, world!"s);
  }
}
