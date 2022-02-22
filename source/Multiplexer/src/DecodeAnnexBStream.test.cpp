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

#include <catch2/catch.hpp>

#include <TMIV/Multiplexer/DecodeAnnexBStream.h>

#include <sstream>

using namespace std::string_view_literals;
using namespace std::string_literals;

namespace {
auto exampleBitstream(int32_t zeroCount, int32_t nalUnitCount, const std::string &payload)
    -> std::string {
  std::ostringstream stream;

  while (nalUnitCount-- > 0) {
    stream << (zeroCount == 2 ? "\0\0\1"sv : "\0\0\0\1"sv) << payload;
  }

  return stream.str();
}
} // namespace

TEST_CASE("decodeAnnexBStream") {
  using TMIV::Multiplexer::decodeAnnexBStream;

  SECTION("Valid sources") {
    using TMIV::Common::sourceFromIteratorPair;

    const int32_t nalUnitCount = GENERATE(0, 1, 2, 8);
    const auto payload = GENERATE(""s, "[ NAL unit ]"s, "\0\0\0 Not a start code"s);
    const auto zeroCount = GENERATE(2, 3);

    CAPTURE(nalUnitCount, payload, zeroCount);

    auto stream =
        std::make_shared<std::istringstream>(exampleBitstream(zeroCount, nalUnitCount, payload));

    auto unitAtTest = decodeAnnexBStream(stream);

    for (int32_t i = 0; i < nalUnitCount; ++i) {
      auto nalUnit = unitAtTest();
      REQUIRE(nalUnit);
      CHECK(*nalUnit == payload);
    }

    CHECK_FALSE(unitAtTest());
  }

  SECTION("When the start code could not be parsed, an exception is thrown") {
    using TMIV::Common::BitstreamError;

    const auto bytestream = GENERATE("Not a start code"s, "\0\1 is also not a start code"s);
    CAPTURE(bytestream);

    auto stream = std::make_shared<std::istringstream>(bytestream);

    auto unitAtTest = decodeAnnexBStream(stream);

    REQUIRE_THROWS_AS(unitAtTest(), BitstreamError);
  }
}
