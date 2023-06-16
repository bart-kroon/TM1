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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>

#include <TMIV/Common/Distribution.h>

#include <cmath>

TEST_CASE("Distribution") {
  using TMIV::Common::Distribution;

  SECTION("No samples") {
    const auto unit = Distribution{};

    CHECK_FALSE(unit);
    CHECK(unit.size() == 0);
    CHECK(unit.count() == 0);
  }

  SECTION("Unique value") {
    const auto value = GENERATE(0.01F, 1.F, 456.F);
    const auto count = GENERATE(1U, 2U, 100U, 101U);

    auto unit = Distribution{};

    for (auto i = 0U; i < count; ++i) {
      unit.sample(value);
    }

    CHECK(unit);
    CHECK(unit.count() == count);
    CHECK(min(unit) == value);
    CHECK(max(unit) == value);
    CHECK(median(unit) == value);
  }

  SECTION("Odd number of unique values") {
    auto unit = Distribution{};
    const auto count = GENERATE(1U, 2U, 3U);

    CAPTURE(count);

    for (const auto value : {0.4F, 0.1F, 100.F, 0.2F, 0.7F}) {
      for (auto i = 0U; i < count; ++i) {
        unit.sample(value);
      }
    }

    CHECK(unit);
    CHECK(unit.size() == 5);
    CHECK(unit.count() == 5 * count);
    CHECK(min(unit) == 0.1F);
    CHECK(max(unit) == 100.F);
    CHECK(median(unit) == 0.4F);
  }

  SECTION("Even number of unique values") {
    auto unit = Distribution{};
    const auto count = GENERATE(1U, 2U, 3U);

    CAPTURE(count);

    for (const auto value : {0.4F, 0.1F, 100.F, 0.2F, 0.7F, 0.3F}) {
      for (auto i = 0U; i < count; ++i) {
        unit.sample(value);
      }
    }

    CHECK(unit);
    CHECK(unit.size() == 6);
    CHECK(unit.count() == 6 * count);
    CHECK(min(unit) == 0.1F);
    CHECK(max(unit) == 100.F);
    CHECK(median(unit) == Catch::Approx(0.35F));
  }
}
