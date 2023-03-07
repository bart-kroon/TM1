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
#include <catch2/matchers/catch_matchers_string.hpp>

#include <TMIV/Renderer/ViewWeightingSynthesizer.h>

using Catch::Matchers::ContainsSubstring;
using TMIV::Common::ColorFormat;
using TMIV::Common::Json;
using TMIV::MivBitstream::AccessUnit;
using TMIV::MivBitstream::CameraConfig;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::Renderer::ViewWeightingSynthesizer;

using namespace std::string_view_literals;

TEST_CASE("TMIV::Renderer::ViewWeightingSynthesizer") {
  SECTION("ViewWeightingSynthesizer(-, componentNode)") {
    [[maybe_unused]] auto unit = ViewWeightingSynthesizer{Json::parse("{}"sv), Json::parse(R"(
{
    "angularScaling": 0.0,
    "minimalWeight": 0.0,
    "stretchFactor": 0.0,
    "blendingFactor": 0.0,
    "overloadFactor": 0.0,
    "filteringPass": 0
})"sv)};
  }

  SECTION("renderFrame(frame, cameraConfig)") {
    auto unit = ViewWeightingSynthesizer{Json::parse("{}"sv), Json::parse(R"(
{
    "angularScaling": 1.5,
    "blendingFactor": 0.03,
    "filteringPass": 1,
    "minimalWeight": 2.5,
    "overloadFactor": 2.0,
    "stretchFactor": 100.0
})"sv)};

    auto frame = AccessUnit{};
    auto cameraConfig = CameraConfig{};

    SECTION("Default-initialized input (runtime error)") {
      REQUIRE_THROWS_WITH(unit.renderFrame(frame, cameraConfig), ContainsSubstring("frame.casps"));
    }

    SECTION("Initialise only CASPS (runtime error)") {
      frame.casps = CommonAtlasSequenceParameterSetRBSP{};
      REQUIRE_THROWS_WITH(unit.renderFrame(frame, cameraConfig),
                          ContainsSubstring("casps_miv_extension_present_flag"));
    }

    SECTION("Initialise only CASME (runtime error)") {
      frame.casps = CommonAtlasSequenceParameterSetRBSP{};
      frame.casps->casps_miv_extension() = {};
      REQUIRE_THROWS_WITH(unit.renderFrame(frame, cameraConfig),
                          ContainsSubstring("Invalid normalized disparity range"));
    }

    SECTION("Initialize only CASME + disparity range (empty output)") {
      frame.casps = CommonAtlasSequenceParameterSetRBSP{};
      frame.casps->casps_miv_extension() = {};
      cameraConfig.viewParams.dq.dq_norm_disp_low(1.F).dq_norm_disp_high(2.F);
      const auto actual = unit.renderFrame(frame, cameraConfig);

      // Geometry: 4:0:0 image of size 1 x 1 filled with a zero
      REQUIRE(actual.geometry.getNumberOfPlanes() == 1);
      CHECK(actual.geometry.getWidth() == 1);
      CHECK(actual.geometry.getHeight() == 1);
      CHECK(actual.geometry.getBitDepth() == 0);
      CHECK(actual.geometry.getPlane(0)(0, 0) == 0);

      // Texture: 4:4:4 image of size 1 x 1 filled with the neutral value
      REQUIRE(actual.texture.getNumberOfPlanes() == 3);
      REQUIRE(actual.texture.getColorFormat() == ColorFormat::YUV444);
      CHECK(actual.texture.getWidth() == 1);
      CHECK(actual.texture.getHeight() == 1);
      CHECK(actual.texture.getBitDepth() == 0);
      CHECK(actual.texture.getPlane(0)(0, 0) == actual.texture.neutralValue());
      CHECK(actual.texture.getPlane(1)(0, 0) == actual.texture.neutralValue());
      CHECK(actual.texture.getPlane(2)(0, 0) == actual.texture.neutralValue());
    }

    SECTION("Minimal initialization with one input view") {
      frame.casps = CommonAtlasSequenceParameterSetRBSP{};
      frame.casps->casps_miv_extension() = {};

      cameraConfig.viewParams.dq.dq_norm_disp_low(1.F).dq_norm_disp_high(2.F);

      [[maybe_unused]] auto &view = frame.viewParamsList.emplace_back();

      REQUIRE_THROWS_WITH(unit.renderFrame(frame, cameraConfig),
                          ContainsSubstring("!prunedViews[sourceIdx].texture.empty()"));
    }
  }
}
