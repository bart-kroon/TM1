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
#include <catch2/matchers/catch_matchers_contains.hpp>

#include "../src/Configuration.h"

using namespace std::string_view_literals;

TEST_CASE("TMIV::Encoder::Configuration") {
  using Catch::Matchers::Contains;
  using TMIV::Common::Json;
  using TMIV::Common::Vec2i;
  using TMIV::Encoder::Configuration;
  using TMIV::MivBitstream::PtlLevelIdc;
  using TMIV::MivBitstream::PtlProfileCodecGroupIdc;
  using TMIV::MivBitstream::PtlProfileReconstructionIdc;
  using TMIV::MivBitstream::PtlProfileToolsetIdc;

  SECTION("MIV Main example") {
    auto root = Json::parse(R"({
    "intraPeriod": 1,
    "blockSize": 2,
    "haveTextureVideo": false,
    "haveGeometryVideo": true,
    "bitDepthGeometryVideo": 10,
    "haveOccupancyVideo": false,
    "embeddedOccupancy": true,
    "informationPruning": true,
    "oneViewPerAtlasFlag": false,
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
    "oneV3cFrameOnly": false
})"sv);

    const auto unit = Configuration{root};

    root.checkForUnusedKeys();

    CHECK(unit.intraPeriod == 1);
    CHECK(unit.blockSize == 2);
    CHECK_FALSE(unit.haveTexture);
    CHECK(unit.haveGeometry);
    CHECK_FALSE(unit.haveOccupancy);
    CHECK_FALSE(unit.oneViewPerAtlasFlag);
    CHECK(unit.dqParamsPresentFlag);
    CHECK(unit.patchRedundancyRemoval);
    CHECK_FALSE(unit.viewportCameraParametersSei);
    CHECK(unit.viewportPositionSei);
    CHECK(unit.numGroups == 0);
    CHECK(unit.maxEntityId == 0);
    CHECK(unit.maxLumaSampleRate == 0.);
    CHECK(unit.maxLumaPictureSize == 0);
    CHECK(unit.maxAtlases == 0);
    CHECK(unit.codecGroupIdc == PtlProfileCodecGroupIdc::HEVC_Main10);
    CHECK(unit.toolsetIdc == PtlProfileToolsetIdc::MIV_Main);
    CHECK(unit.reconstructionIdc == PtlProfileReconstructionIdc::Rec_Unconstrained);
    CHECK(unit.levelIdc == PtlLevelIdc::Level_2_5);
    CHECK_FALSE(unit.oneV3cFrameOnly);
    CHECK_FALSE(unit.viewingSpace.has_value());
    CHECK(unit.overrideAtlasFrameSizes.empty());

    SECTION("Add viewing space") {
      root.update(Json::parse(R"({
    "ViewingSpace": {
        "ElementaryShapes": [{
            "ElementaryShapeOperation": "add",
            "ElementaryShape": {
                "PrimitiveShapeOperation": "add",
                "PrimitiveShapes": [{
                    "PrimitiveShapeType": "cuboid",
                    "Center": [0, 0, 0],
                    "Size": [0, 0, 0]
                }]
            }
        }]
    }
})"));

      const auto unit2 = Configuration{root};

      root.checkForUnusedKeys();

      CHECK(unit2.viewingSpace.has_value());
    }

    SECTION("Test for maxIntraPeriod") {
      root.update(Json::parse(R"({ "intraPeriod": 33 })"));
      REQUIRE_THROWS_AS((Configuration{root}), std::runtime_error);
    }

    SECTION("AVC Progressive High") {
      root.update(Json::parse(R"({ "codecGroupIdc": "AVC Progressive High" })"));
      const auto unit2 = Configuration{root};

      root.checkForUnusedKeys();

      CHECK(unit2.codecGroupIdc == PtlProfileCodecGroupIdc::AVC_Progressive_High);
    }

    SECTION("Unknown codec group IDC") {
      root.update(Json::parse(R"({ "codecGroupIdc": "Fantasy" })"));
      REQUIRE_THROWS_AS((Configuration{root}), std::runtime_error);
    }

    SECTION("Unknown toolset IDC") {
      root.update(Json::parse(R"({ "toolsetIdc": "Hammer" })"));
      REQUIRE_THROWS_AS((Configuration{root}), std::runtime_error);
    }
  }

  SECTION("MIV Geometry Absent example") {
    auto root = Json::parse(R"({
    "intraPeriod": 13,
    "blockSize": 8,
    "haveTextureVideo": true,
    "bitDepthTextureVideo": 10,
    "haveGeometryVideo": false,
    "haveOccupancyVideo": false,
    "informationPruning": false,
    "chromaScaleEnabledFlag": false,
    "oneViewPerAtlasFlag": true,
    "dqParamsPresentFlag": false,
    "textureOffsetEnabledFlag": false,
    "patchRedundancyRemoval": false,
    "viewportCameraParametersSei": true,
    "viewportPositionSei": false,
    "numGroups": 3,
    "maxEntityId": 5,
    "entityEncodeRange": [0, 4],
    "codecGroupIdc": "AVC Progressive High",
    "toolsetIdc": "MIV Geometry Absent",
    "reconstructionIdc": "Rec Unconstrained",
    "levelIdc": "2.5",
    "oneV3cFrameOnly": true
})"sv);

    const auto unit = Configuration{root};

    root.checkForUnusedKeys();

    CHECK(unit.intraPeriod == 13);
    CHECK(unit.blockSize == 8);
    CHECK(unit.haveTexture);
    CHECK_FALSE(unit.haveGeometry);
    CHECK_FALSE(unit.haveOccupancy);
    CHECK(unit.oneViewPerAtlasFlag);
    CHECK_FALSE(unit.dqParamsPresentFlag);
    CHECK_FALSE(unit.patchRedundancyRemoval);
    CHECK(unit.viewportCameraParametersSei);
    CHECK_FALSE(unit.viewportPositionSei);
    CHECK(unit.numGroups == 3);
    CHECK(unit.maxEntityId == 5);
    CHECK(unit.codecGroupIdc == PtlProfileCodecGroupIdc::AVC_Progressive_High);
    CHECK(unit.toolsetIdc == PtlProfileToolsetIdc::MIV_Geometry_Absent);
    CHECK(unit.oneV3cFrameOnly);
    CHECK_FALSE(unit.viewingSpace.has_value());
    CHECK(unit.overrideAtlasFrameSizes.empty());

    SECTION("Override atlas frame sizes") {
      root.update(Json::parse(R"({ "overrideAtlasFrameSizes": [ [4, 5], [6, 7] ] })"));

      const auto unit2 = Configuration{root};

      root.checkForUnusedKeys();

      CHECK(unit2.overrideAtlasFrameSizes == std::vector{Vec2i{4, 5}, Vec2i{6, 7}});
    }

    SECTION("Attribute offset") {
      root.update(Json::parse(R"({
    "textureOffsetEnabledFlag": true,
    "textureOffsetBitCount": 1
})"));

      const auto unit2 = Configuration{root};

      root.checkForUnusedKeys();

      CHECK(unit2.textureOffsetFlag);
      CHECK(unit2.textureOffsetBitCount == 1);
    }
  }

  SECTION("Enable Frame Packing") {
    const auto root = Json::parse(R"({
    "intraPeriod": 13,
    "blockSize": 8,
    "haveTextureVideo": false,
    "haveGeometryVideo": false,
    "haveOccupancyVideo": false,
    "informationPruning": true,
    "viewportCameraParametersSei": false,
    "viewportPositionSei": false,
    "oneViewPerAtlasFlag": true,
    "dqParamsPresentFlag": false,
    "patchRedundancyRemoval": true,
    "numGroups": 3,
    "maxEntityId": 5,
    "entityEncodeRange": [0, 4],
    "codecGroupIdc": "AVC Progressive High",
    "toolsetIdc": "MIV Geometry Absent",
    "reconstructionIdc": "Rec Unconstrained",
    "levelIdc": "2.5",
    "oneV3cFrameOnly": false
})"sv);

    const auto unit = Configuration{root};

    root.checkForUnusedKeys();

    CHECK_FALSE(unit.haveTexture);
    CHECK_FALSE(unit.haveGeometry);
  }

  SECTION("Enable Geometry Packing") {
    const auto root = Json::parse(R"({
    "intraPeriod": 13,
    "blockSize": 8,
    "haveTextureVideo": false,
    "haveGeometryVideo": false,
    "haveOccupancyVideo": false,
    "informationPruning": true,
    "viewportCameraParametersSei": false,
    "viewportPositionSei": false,
    "oneViewPerAtlasFlag": true,
    "dqParamsPresentFlag": false,
    "patchRedundancyRemoval": true,
    "numGroups": 3,
    "maxEntityId": 5,
    "entityEncodeRange": [0, 4],
    "codecGroupIdc": "AVC Progressive High",
    "toolsetIdc": "MIV Geometry Absent",
    "reconstructionIdc": "Rec Unconstrained",
    "levelIdc": "2.5",
    "oneV3cFrameOnly": false
})"sv);

    const auto unit = Configuration{root};

    root.checkForUnusedKeys();

    CHECK_FALSE(unit.haveTexture);
    CHECK_FALSE(unit.haveGeometry);
  }
}
