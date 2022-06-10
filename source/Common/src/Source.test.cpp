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

#include <TMIV/Common/Source.h>

#include <array>

namespace test {
struct DefaultInitMovable {
  DefaultInitMovable() = default;
  DefaultInitMovable(const DefaultInitMovable &) = delete;
  DefaultInitMovable(DefaultInitMovable &&) = default;
  auto operator=(const DefaultInitMovable &) -> DefaultInitMovable & = delete;
  auto operator=(DefaultInitMovable &&) -> DefaultInitMovable & = default;
  ~DefaultInitMovable() = default;
};

auto exampleStream() -> TMIV::Common::Stream<int32_t> {
  return [n = 0]() mutable -> int32_t { return n++; };
}

auto exampleSource(int32_t n) -> TMIV::Common::Source<DefaultInitMovable> {
  return [=]() mutable -> std::optional<DefaultInitMovable> {
    if (n == 0) {
      return std::nullopt;
    }
    --n;
    return DefaultInitMovable{};
  };
}
} // namespace test

TEST_CASE("TMIV::Common::emptySource()") {
  using TMIV::Common::emptySource;

  auto source = emptySource<test::DefaultInitMovable>();
  REQUIRE_FALSE(source());
  REQUIRE_FALSE(source());
}

TEST_CASE("TMIV::Common::uniformSource(n, x)") {
  using TMIV::Common::uniformSource;

  const auto n = GENERATE(0, 1, 4);
  const auto x = GENERATE(-1, 5);

  auto source = uniformSource<int32_t>(n, x);
  for (auto i = 0; i < n; ++i) {
    REQUIRE(source() == x);
  }
  REQUIRE_FALSE(source().has_value());
}

TEST_CASE("TMIV::Common::sourceFromIteratorPair") {
  using TMIV::Common::sourceFromIteratorPair;

  const auto coll = std::array{1, 2, 3, 4};

  auto source = sourceFromIteratorPair(coll.cbegin(), coll.cend());

  for (auto i : coll) {
    REQUIRE(source() == i);
  }
  REQUIRE_FALSE(source().has_value());
}

TEST_CASE("TMIV::Common::ReadAhead") {
  using TMIV::Common::ReadAhead;

  const auto n = 4;
  auto source = test::exampleStream();

  for (int32_t i = 0; i < n; ++i) {
    REQUIRE(source() == i);
  }

  auto unit = ReadAhead{std::move(source)};

  for (int32_t i = n; i < 2 * n; ++i) {
    REQUIRE(unit.head() == i);
    REQUIRE(unit() == i);
    REQUIRE(unit.head() == i + 1);
  }
}

TEST_CASE("TMIV::Common::test(Source<T>)") {
  using TMIV::Common::test;

  SECTION("Empty source") {
    using TMIV::Common::emptySource;

    auto source = emptySource<test::DefaultInitMovable>();
    auto actual = test(source);
    REQUIRE(actual == nullptr);
  }

  SECTION("Finite source") {
    const auto n = 4;
    auto source = test::exampleSource(n);
    auto actual = test(source);
    REQUIRE(actual != nullptr);

    for (int32_t i = 0; i < n; ++i) {
      REQUIRE(actual());
    }
    REQUIRE_FALSE(actual());
  }
}
