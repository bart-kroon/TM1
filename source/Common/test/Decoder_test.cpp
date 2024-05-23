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

#include <catch2/catch_test_macros.hpp>

#include <TMIV/Common/Decoder.h>

namespace test {
struct Apple {};
struct Orange {};

constexpr auto fakeSource() noexcept {
  return [i = 0]() mutable -> std::optional<Apple> {
    switch (i++) {
    case 0:
    case 1:
      return Apple{};
    case 2:
      return std::nullopt;
    default:
      UNREACHABLE;
    }
  };
}

struct FakeDecoder final : public TMIV::Common::Decoder<Apple, Orange> {
  FakeDecoder(TMIV::Common::Source<Apple> source)
      : TMIV::Common::Decoder<Apple, Orange>{std::move(source)} {}

  int32_t decodeSomeCallCount{};

  auto decodeSome() -> bool final {
    switch (decodeSomeCallCount++) {
    case 0:
      REQUIRE(pull());
      return true;
    case 1:
      REQUIRE(pull());
      push(Orange{});
      push(Orange{});
      return true;
    case 2:
      push(Orange{});
      return true;
    case 3:
      REQUIRE_FALSE(pull());
      return true;
    case 4:
      push(Orange{});
      return false;
    default:
      FAIL("It is not allowed to call decodeSome() after it returned false");
      return false;
    }
  }
};
} // namespace test

TEST_CASE("TMIV::Common::Decoder") {
  auto unit = test::FakeDecoder{test::fakeSource()};

  REQUIRE(unit.decodeSomeCallCount == 0);

  REQUIRE(unit());
  REQUIRE(unit.decodeSomeCallCount == 2);

  REQUIRE(unit());
  REQUIRE(unit.decodeSomeCallCount == 2);

  REQUIRE(unit());
  REQUIRE(unit.decodeSomeCallCount == 3);

  REQUIRE(unit());
  REQUIRE(unit.decodeSomeCallCount == 5);

  REQUIRE_FALSE(unit());
  REQUIRE(unit.decodeSomeCallCount == 5);
}
