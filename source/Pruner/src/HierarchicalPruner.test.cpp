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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <TMIV/Pruner/HierarchicalPruner.h>

#include <fmt/format.h>

using Catch::Matchers::ContainsSubstring;
using TMIV::Common::Json;
using TMIV::Pruner::HierarchicalPruner;
using TMIV::Pruner::PrunerParams;

using namespace std::string_view_literals;

TEST_CASE("TMIV::Pruner::HierarchicalPruner") {
  SECTION("HierarchicalPruner(rootNode, componentNode)") {
    auto unit = HierarchicalPruner{Json::parse("{}"sv), Json::parse(R"(
{
    "maxDepthError": 0.0,
    "maxLumaError": 0.0,
    "maxStretching": 0.0,
    "erode": 0,
    "dilate": 0,
    "maxBasicViewsPerGraph": 0,
    "enable2ndPassPruner": false,
    "sampleSize": 0,
    "maxColorError": 0.0,
    "rayAngleParameter": 0.0,
    "depthParameter": 0.0,
    "stretchingParameter": 0.0
})"sv)};
  }

  SECTION("prepareSequence") {
    auto unit = HierarchicalPruner{Json::parse("{}"sv), Json::parse(R"(
{
    "depthParameter": 50,
    "dilate": 5,
    "enable2ndPassPruner": true,
    "erode": 2,
    "maxBasicViewsPerGraph": 3,
    "maxColorError": 0.1,
    "maxDepthError": 0.1,
    "maxLumaError": 0.04,
    "maxStretching": 5,
    "rayAngleParameter": 10,
    "sampleSize": 32,
    "stretchingParameter": 3
})"sv)};

    GIVEN("default-constructed PrunerParams") {
      auto params = PrunerParams{};

      THEN("preparing the sequence fails because there are no view parameters") {
        REQUIRE_THROWS_WITH(unit.prepareSequence(params),
                            ContainsSubstring("!viewParamsList.empty()"));
      }

      GIVEN("default-initialized view parameters") {
        const auto viewCount = GENERATE(size_t{1}, size_t{3});

        for (size_t i = 0; i < viewCount; ++i) {
          params.viewParamsList.emplace_back().name = fmt::format("v{:02}", i);
        }

        WHEN("preparing the sequence") {
          const auto pruningParents = unit.prepareSequence(params);

          THEN("the result has as many elements and they are all roots") {
            REQUIRE(pruningParents.size() == viewCount);

            for (const auto &pruningParent : pruningParents) {
              CHECK(pruningParent.pp_is_root_flag());
            }
          }
        }
      }
    }
  }
}
