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

#include <TMIV/Quantizer/GeometryQuantizer.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <array>

namespace TMIV::Quantizer::test {
using namespace std::string_view_literals;

using Common::at;

static constexpr auto patchViewIdx = std::array{size_t{}, size_t{1}, size_t{1}};
static constexpr auto patchAtlasIdx = std::array{size_t{2}, size_t{}, size_t{}};
static constexpr auto patchCount = patchViewIdx.size();
static constexpr auto viewCount = size_t{3};
static constexpr auto atlasCount = size_t{3};
static constexpr auto patchNear = std::array{5.F, 1.F, 3.F};
static constexpr auto patchFar = std::array{2.F, 0.5F, 2.F};
static constexpr auto viewNear = std::array{10.F, 5.F, 3.F};
static constexpr auto viewFar = std::array{1.F, 0.5F, 0.3F};
static constexpr auto viewHasOccupancy = std::array{false, true, true};
static constexpr auto geo2dBitDepth = 9U;

[[nodiscard]] auto distributions() -> GeometryDistributions {
  auto result = GeometryDistributions{};

  result.views.resize(viewCount);
  result.patches.resize(patchCount);

  for (size_t patchIdx = 0; patchIdx < patchCount; ++patchIdx) {
    const auto viewIdx = at(patchViewIdx, patchIdx);

    result.patches[patchIdx].sample(at(patchNear, patchIdx));
    result.patches[patchIdx].sample(at(patchFar, patchIdx));

    result.views[viewIdx].sample(at(patchNear, patchIdx));
    result.views[viewIdx].sample(at(patchFar, patchIdx));
  }

  return result;
}

[[nodiscard]] auto viewId(size_t viewIdx) {
  static constexpr auto offset = size_t{100};

  return MivBitstream::ViewId{offset + viewIdx};
}

[[nodiscard]] auto atlasId(size_t atlasIdx) { return MivBitstream::AtlasId{atlasIdx}; }

[[nodiscard]] auto params(bool vme_embedded_occupancy_enabled_flag,
                          bool casme_depth_low_quality_flag) -> EncoderParams {
  auto result = EncoderParams{};

  result.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag(
      vme_embedded_occupancy_enabled_flag);
  result.casps.casps_miv_extension().casme_depth_low_quality_flag(casme_depth_low_quality_flag);

  result.viewParamsList.resize(viewCount);

  for (size_t viewIdx = 0; viewIdx < viewCount; ++viewIdx) {
    auto &vp = result.viewParamsList[viewIdx];

    vp.dq.dq_norm_disp_low(at(viewFar, viewIdx)).dq_norm_disp_high(at(viewNear, viewIdx));
    vp.hasOccupancy = at(viewHasOccupancy, viewIdx);
    vp.viewId = viewId(viewIdx);
  }

  result.viewParamsList.constructViewIdIndex();

  result.patchParamsList.resize(patchCount);

  for (size_t patchIdx = 0; patchIdx < patchCount; ++patchIdx) {
    auto &pp = result.patchParamsList[patchIdx];

    pp.atlasId(atlasId(at(patchAtlasIdx, patchIdx)))
        .atlasPatchProjectionId(viewId(at(patchViewIdx, patchIdx)));
  }

  result.vps.vps_atlas_count_minus1(atlasCount - 1);
  result.atlas.resize(atlasCount);

  for (size_t atlasIdx = 0; atlasIdx < atlasCount; ++atlasIdx) {
    result.atlas[atlasIdx].asps.asps_geometry_2d_bit_depth_minus1(
        static_cast<uint8_t>(geo2dBitDepth - 1));

    result.vps.vps_atlas_id(atlasIdx, atlasId(atlasIdx));
  }

  return result;
}

[[nodiscard]] auto config(bool dynamicDepthRange, bool halveDepthRange,
                          double depthOccThresholdAsymmetry) -> Configuration {
  auto result = Configuration{Common::Json::parse(R"({
    "intraPeriod": 1,
    "blockSizeDepthQualityDependent": [2, 4],
    "haveTextureVideo": false,
    "haveGeometryVideo": true,
    "bitDepthGeometryVideo": 10,
    "haveOccupancyVideo": false,
    "embeddedOccupancy": true,
    "chromaScaleEnabledFlag": true,
    "framePacking": false,
    "geometryPacking": false,
    "informationPruning": true,
    "oneViewPerAtlasFlag": false,
    "dynamicDepthRange": true,
    "halveDepthRange": true,
    "rewriteParameterSets": false,
    "patchRedundancyRemoval": true,
    "viewportCameraParametersSei": false,
    "viewportPositionSei": true,
    "numGroups": 0,
    "maxEntityId": 0,
    "maxLumaSampleRate": 0,
    "maxLumaPictureSize": 0,
    "maxAtlases": 0,
    "codecGroupIdc": "HEVC Main10",
    "toolsetIdc": "MIV Main",
    "reconstructionIdc": "Rec Unconstrained",
    "levelIdc": "2.5",
    "oneV3cFrameOnly": false,
    "piecewiseDepthLinearScaling": false,
    "depthOccThresholdIfSet": [0.00390625, 0.0625],
    "depthOccThresholdAsymmetry": 1.5
})"sv)};

  result.dynamicDepthRange = dynamicDepthRange;
  result.halveDepthRange = halveDepthRange;
  result.geoBitDepth = geo2dBitDepth;
  result.depthOccThresholdAsymmetry = depthOccThresholdAsymmetry;

  return result;
}

void checkDepthQuantization(const EncoderParams &actual) {
  REQUIRE(actual.viewParamsList.size() == viewCount);
  REQUIRE(actual.patchParamsList.size() == patchCount);
  REQUIRE(actual.atlas.size() == atlasCount);

  const auto embeddedOccupancy =
      actual.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag();

  for (size_t patchIdx = 0; patchIdx < patchCount; ++patchIdx) {
    const auto &pp = actual.patchParamsList[patchIdx];
    const auto &vp = actual.viewParamsList[pp.atlasPatchProjectionId()];

    const auto dt = MivBitstream::DepthTransform{vp.dq, pp, geo2dBitDepth};
    const auto threshold = vp.dq.dq_depth_occ_threshold_default();
    LIMITATION(pp.atlasPatchDepthOccThreshold() == 0);

    CHECK(dt.expandNormDisp(threshold) <= at(patchFar, patchIdx));
    CHECK(at(patchNear, patchIdx) <= dt.expandNormDisp(pp.atlasPatch3dRangeD()));
    CHECK((0 < threshold) == (vp.hasOccupancy && embeddedOccupancy));
  }
}
} // namespace TMIV::Quantizer::test

namespace test = TMIV::Quantizer::test;

TEST_CASE("GeometryQuantizer::transformParams") {
  using TMIV::Quantizer::GeometryQuantizer;

  TMIV::Common::replaceLoggingStrategy([](auto &&...) {});

  SECTION("dq_quantization_law == 0") {
    const auto embeddedOccupancy = GENERATE(false, true);
    const auto depthLowQualityFlag = GENERATE(false, true);
    const auto dynamicDepthRange = GENERATE(false, true);
    const auto halveDepthRange = GENERATE(false, true);
    const auto depthOccThresholdAsymmetry = GENERATE(0., 1.3, 2.);

    CAPTURE(embeddedOccupancy, depthLowQualityFlag, dynamicDepthRange, halveDepthRange,
            depthOccThresholdAsymmetry);

    const auto distributions = test::distributions();
    const auto params = test::params(embeddedOccupancy, depthLowQualityFlag);
    const auto config =
        test::config(dynamicDepthRange, halveDepthRange, depthOccThresholdAsymmetry);

    const auto unit = GeometryQuantizer{config};

    const auto actual = unit.transformParams(distributions, params);

    test::checkDepthQuantization(actual);
  }
}
