/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <TMIV/Metadata/Bitstream.h>
#include <TMIV/Metadata/IvAccessUnitParams.h>
#include <TMIV/Metadata/IvSequenceParams.h>

using namespace std;
using namespace TMIV::Metadata;

using namespace TMIV::Common;

TEST_CASE("TestPatchRotationAndFlipTransforms") {
  AtlasParameters patch;

  patch.atlasId = 0;
  patch.viewId = 0;
  patch.patchSizeInView = {10, 5};
  patch.posInView = {0, 0};
  patch.posInAtlas = {10, 20};
  patch.rotation = PatchRotation::none;

  auto rotations = {PatchRotation::none,   PatchRotation::rot270,  PatchRotation::rot180,
                    PatchRotation::rot90,  PatchRotation::mrot180, PatchRotation::swap,
                    PatchRotation::mirror, PatchRotation::mrot90};

  SECTION("EvaluateTransformationOf_TopLeft") {
    Vec2i posInViewEncode = {0, 0};
    std::vector<Vec2i> posInAtlasExpected = {{0, 0}, {0, 9}, {9, 4}, {4, 0},
                                             {0, 4}, {0, 0}, {9, 0}, {4, 9}};
    for (auto &pos : posInAtlasExpected) {
      pos += patch.posInAtlas;
    }

    int i = 0;
    for (auto rotation : rotations) {
      patch.rotation = rotation;
      auto posInAtlas = viewToAtlas(posInViewEncode, patch);
      REQUIRE(posInAtlas == posInAtlasExpected[i]);

      auto posInViewDecode = atlasToView(posInAtlas, patch);
      REQUIRE(posInViewDecode == posInViewEncode);

      ++i;
    }
  }

  SECTION("EvaluateTransformationOf_BottomRight") {
    Vec2i posInViewEncode = {9, 4};
    std::vector<Vec2i> posInAtlasExpected = {{9, 4}, {4, 0}, {0, 0}, {0, 9},
                                             {9, 0}, {4, 9}, {0, 4}, {0, 0}};
    for (auto &pos : posInAtlasExpected) {
      pos += patch.posInAtlas;
    }

    int i = 0;
    for (auto rotation : rotations) {
      patch.rotation = rotation;
      auto posInAtlas = viewToAtlas(posInViewEncode, patch);
      REQUIRE(posInAtlas == posInAtlasExpected[i]);

      auto posInViewDecode = atlasToView(posInAtlas, patch);
      REQUIRE(posInViewDecode == posInViewEncode);

      ++i;
    }
  }
}

TEST_CASE("Bitstream primitives") {
  stringstream stream;
  OutputBitstream obitstream{stream};
  InputBitstream ibitstream{stream};

  SECTION("u(1)") {
    obitstream.putFlag(true);
    obitstream.byteAlign();
    const auto actual = ibitstream.getFlag();
    REQUIRE(actual);
  }

  SECTION("bitstream") {
    const auto reference = array{false, true,  false, false, true, true, true,  false, true,
                                 true,  false, true,  false, true, true, false, true};
    for (const auto bit : reference) {
      obitstream.putFlag(bit);
    }
    obitstream.byteAlign();
    REQUIRE(stream.tellp() == 3);
    for (const auto bit : reference) {
      const auto actual = ibitstream.getFlag();
      REQUIRE(actual == bit);
    }
    REQUIRE(stream.tellg() == 3);
  }

  SECTION("u(8)") {
    const auto reference = uint8_t(0x12);
    obitstream.putUint8(reference);
    obitstream.byteAlign();
    const auto actual = ibitstream.getUint8();
    REQUIRE(actual == reference);
  }

  SECTION("u(16)") {
    const auto reference = uint16_t(0x1234);
    obitstream.putUint16(reference);
    obitstream.byteAlign();
    const auto actual = ibitstream.getUint16();
    REQUIRE(actual == reference);
  }

  SECTION("u(32)") {
    const auto reference = uint32_t(0x12345678);
    obitstream.putUint32(reference);
    obitstream.byteAlign();
    const auto actual = ibitstream.getUint32();
    REQUIRE(actual == reference);
  }

  SECTION("float32") {
    const auto reference = 1.F / 42.F;
    obitstream.putFloat32(reference);
    obitstream.byteAlign();
    const auto actual = ibitstream.getFloat32();
    REQUIRE(actual == reference);
  }

  SECTION("u(v)") {
    const auto referenceSequence =
        array{tuple{123, 400}, tuple{4, 10}, tuple{400, 401}, tuple{0, 0}, tuple{0, 1}};
    for (auto [reference, range] : referenceSequence) {
      obitstream.putUVar(reference, range);
    }
    obitstream.byteAlign();
    for (auto [reference, range] : referenceSequence) {
      auto actual = ibitstream.getUVar(range);
      REQUIRE(actual == reference);
    }
  }

  SECTION("ue(v)") {
    const auto referenceSequence = array<uint_least64_t, 7>{123, 4, 400, 0, 1, 3, 0x123456789ABC};
    for (auto reference : referenceSequence) {
      obitstream.putUExpGolomb(reference);
    }
    obitstream.byteAlign();
    for (auto reference : referenceSequence) {
      auto actual = ibitstream.getUExpGolomb();
      REQUIRE(actual == reference);
    }
  }
}

namespace examples {
const auto cameraParameters = array{CameraParameters{{4096, 2048},
                                                     {1.F, 2.F, 3.F},
                                                     {40.F, -60.F, 30.F},
                                                     ProjectionType::ERP,
                                                     {-60.F, 70.F},
                                                     {-50.F, 70.F},
                                                     {},
                                                     {},
                                                     {0.5F, 100.F}},
                                    CameraParameters{{1920, 1080},
                                                     {4.F, 5.F, 6.F},
                                                     {0.F, 0.F, 0.F},
                                                     ProjectionType::Perspective,
                                                     {},
                                                     {},
                                                     {1000, 1010},
                                                     {960, 540},
                                                     {1.F, 50.F}}};

const auto cameraParameterList =
    array{CameraParamsList{{cameraParameters[0]}},
          CameraParamsList{{cameraParameters[0], cameraParameters[0]}},
          CameraParamsList{{cameraParameters[0], cameraParameters[1]}}};

const auto ivsProfileTierLevel = array{IvsProfileTierLevel{}};

const auto ivsParams = array{IvsParams{ivsProfileTierLevel[0], cameraParameterList[1]}};

const auto atlasParamsList = array{
    AtlasParamsList{{AtlasParameters{0, 0, {100, 50}, {5, 4}, {34, 22}, PatchRotation::mrot90}},
                    true,
                    {{1920, 1080}}},
    AtlasParamsList{
        {AtlasParameters{0, 0, {4096, 2048}, {0, 0}, {0, 0}, PatchRotation::mrot90},
         AtlasParameters{0, 1, {100, 40}, {5, 4}, {34, 22}, PatchRotation::mrot180},
         AtlasParameters{2, 1, {100, 30}, {500, 400}, {340, 220}, PatchRotation::rot180}},
        true,
        {{2048, 4096}, {0, 0}, {2048, 1088}}}};

const auto ivAccessUnitParams =
    array{IvAccessUnitParams{}, IvAccessUnitParams{{atlasParamsList[1]}}};
} // namespace examples

namespace {
template <typename Type, typename... Args>
bool codingTest(const Type &reference, int size, Args &... args) {
  stringstream stream;
  OutputBitstream obitstream{stream};
  reference.encodeTo(obitstream, args...);
  obitstream.byteAlign();
  REQUIRE(stream.tellp() == size);

  InputBitstream ibitstream{stream};
  const auto actual = Type::decodeFrom(ibitstream, args...);
  REQUIRE(stream.tellg() == size);

  return actual == reference;
}
} // namespace

TEST_CASE("CameraParamsList") {
  SECTION("areIntrinsicParamsEqual") {
    REQUIRE(examples::cameraParameterList[0].areIntrinsicParamsEqual());
    REQUIRE(examples::cameraParameterList[1].areIntrinsicParamsEqual());
    REQUIRE(!examples::cameraParameterList[2].areIntrinsicParamsEqual());
  }
  SECTION("areDepthQuantizationParamsEqual") {
    REQUIRE(examples::cameraParameterList[0].areDepthQuantizationParamsEqual());
    REQUIRE(examples::cameraParameterList[1].areDepthQuantizationParamsEqual());
    REQUIRE(!examples::cameraParameterList[2].areDepthQuantizationParamsEqual());
  }
}

TEST_CASE("Metadata bitstreams") {
  SECTION("ivs_profile_tier_level[0]") { REQUIRE(codingTest(examples::ivsProfileTierLevel[0], 0)); }
  SECTION("camera_params_list[0]") { REQUIRE(codingTest(examples::cameraParameterList[0], 59)); }
  SECTION("camera_params_list[1]") { REQUIRE(codingTest(examples::cameraParameterList[1], 83)); }
  SECTION("camera_params_list[2]") { REQUIRE(codingTest(examples::cameraParameterList[2], 115)); }
  SECTION("ivs_params[0]") { REQUIRE(codingTest(examples::ivsParams[0], 83)); }
  SECTION("atlas_params_list[0]") {
    REQUIRE(codingTest(examples::atlasParamsList[0], 17, examples::cameraParameterList[0]));
  }
  SECTION("atlas_params_list[1]") {
    REQUIRE(codingTest(examples::atlasParamsList[1], 42, examples::cameraParameterList[1]));
  }
  SECTION("iv_access_unit_params[0]") {
    REQUIRE(codingTest(examples::ivAccessUnitParams[0], 1, examples::cameraParameterList[1]));
  }
  SECTION("iv_access_unit_params[1]") {
    REQUIRE(codingTest(examples::ivAccessUnitParams[1], 42, examples::cameraParameterList[1]));
  }
}