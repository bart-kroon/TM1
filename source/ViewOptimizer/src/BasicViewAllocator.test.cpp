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

#include <TMIV/ViewOptimizer/BasicViewAllocator.h>

#include <fmt/format.h>

#include <random>

using TMIV::Common::Json;
using TMIV::ViewOptimizer::BasicViewAllocator;

using namespace std::string_view_literals;

namespace test {
const auto sourceParams = TMIV::ViewOptimizer::SourceParams{
    []() {
      const int32_t numOfCams = 10;
      const int32_t W = 10;
      const int32_t H = 4;
      auto vpl = TMIV::MivBitstream::ViewParamsList{};

      std::mt19937 rnd{1};

      for (int32_t c = 0; c < numOfCams; c++) {
        auto &vp = vpl.emplace_back();
        vp.viewId = TMIV::MivBitstream::ViewId{c};
        vp.pose.position = TMIV::Common::Vec3f{0.0F, static_cast<float>(rnd()), 0.0F};
        vp.pose.orientation = TMIV::Common::neutralOrientationF;
        vp.ci.ci_cam_type(TMIV::MivBitstream::CiCamType::perspective);
        vp.ci.ci_perspective_center_hor(static_cast<float>(W) / 2.0F);
        vp.ci.ci_perspective_center_ver(static_cast<float>(H) / 2.0F);
        vp.ci.ci_perspective_focal_hor(1.0F);
        vp.ci.ci_perspective_focal_ver(1.0F);
        vp.ci.ci_projection_plane_height_minus1(H - 1);
        vp.ci.ci_projection_plane_width_minus1(W - 1);
        vp.dq.dq_norm_disp_high(100.0F);
        vp.dq.dq_norm_disp_low(1.0F);
        vp.name = fmt::format("v{:02}", c);
      }
      vpl.constructViewIdIndex();
      return vpl;
    }(),
    true};
}

TEST_CASE("TMIV::ViewOptimizer::BasicViewAllocator") {
  SECTION("With additional views") {
    const auto config1 = Json::parse(R"({
    "numGroups": 3,
    "maxLumaPictureSize": 1000,
    "maxAtlases": 7
})"sv);
    const auto config2 = Json::parse(R"(
{
    "outputAdditionalViews": true,
    "minNonCodedViews": 3,
    "maxBasicViewFraction": 0.6
})"sv);
    auto unit = BasicViewAllocator{config1, config2};

    SECTION("optimizeParams") {
      const auto viewOptimizerParams = unit.optimizeParams(test::sourceParams);
      REQUIRE(viewOptimizerParams.viewParamsList.size() ==
              test::sourceParams.viewParamsList.size());

      const auto expected =
          std::array{true, false, true, true, true, false, true, true, true, false};

      REQUIRE(viewOptimizerParams.viewParamsList.size() == expected.size());

      for (size_t i = 0; i < expected.size(); ++i) {
        CAPTURE(i);
        CHECK(viewOptimizerParams.viewParamsList[i].isBasicView == TMIV::Common::at(expected, i));
      }
    }
  }

  SECTION("Only basic views") {
    const auto config1 = Json::parse(R"({
    "numGroups": 2,
    "maxLumaPictureSize": 1000,
    "maxAtlases": 2
})"sv);
    const auto config2 = Json::parse(R"(
{
    "outputAdditionalViews": false,
    "minNonCodedViews": 0,
    "maxBasicViewFraction": 0.1
})"sv);
    auto unit = BasicViewAllocator{config1, config2};

    SECTION("optimizeParams") {
      const auto viewOptimizerParams = unit.optimizeParams(test::sourceParams);

      const auto expected = std::array{"v04"sv, "v07"sv};

      REQUIRE(viewOptimizerParams.viewParamsList.size() == expected.size());

      for (size_t i = 0; i < expected.size(); ++i) {
        CHECK(viewOptimizerParams.viewParamsList[i].name == TMIV::Common::at(expected, i));
      }
    }
  }
}
