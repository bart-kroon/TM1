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

#include <TMIV/Aggregator/Aggregator.h>

#include <algorithm>

using namespace std::string_view_literals;

namespace test {
auto mask(int32_t seed) {
  auto result = TMIV::Common::Frame<uint8_t>::lumaOnly({5, 7});
  std::iota(result.getPlane(0).begin(), result.getPlane(0).end(), static_cast<uint8_t>(seed));
  return result;
}

auto maskList(int32_t seed) {
  auto result = std::vector<TMIV::Common::Frame<uint8_t>>{};
  result.push_back(mask(seed));
  result.push_back(mask(seed + 250));
  return result;
}

auto sum(const TMIV::Common::Frame<uint8_t> &mask) {
  return std::accumulate(mask.getPlane(0).cbegin(), mask.getPlane(0).cend(), size_t{});
}
} // namespace test

TEST_CASE("TMIV::Aggregator::Aggregator") {
  using TMIV::Aggregator::Aggregator;
  using TMIV::Common::Json;

  const auto config1 = Json::parse("{}"sv);
  const auto config2 = Json::parse("{}"sv);

  auto unit = Aggregator{config1, config2};

  unit.prepareAccessUnit();

  CHECK(unit.getAggregatedMask().empty());

  unit.pushMask(test::maskList(1));

  const auto &result = unit.getAggregatedMask();

  unit.completeAccessUnit();

  REQUIRE(result.size() == 2);
  REQUIRE_FALSE(result[0].empty());
  REQUIRE_FALSE(result[1].empty());
  CHECK(test::sum(result[0]) == 630);
  CHECK(test::sum(result[1]) == 1700);

  unit.pushMask(test::maskList(2));

  unit.completeAccessUnit();

  REQUIRE(result.size() == 2);
  REQUIRE_FALSE(result[0].empty());
  REQUIRE_FALSE(result[1].empty());
  CHECK(test::sum(result[0]) == 665);
  CHECK(test::sum(result[1]) == 1734);

  unit.prepareAccessUnit();

  CHECK(unit.getAggregatedMask().empty());
}
