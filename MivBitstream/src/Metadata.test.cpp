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

#include <catch2/catch.hpp>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/MivBitstream/IvAccessUnitParams.h>
#include <TMIV/MivBitstream/IvSequenceParams.h>
#include <TMIV/MivBitstream/ViewingSpace.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

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
    SizeVector posInAtlasExpected = {{0, 0}, {0, 9}, {9, 4}, {4, 0},
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
    SizeVector posInAtlasExpected = {{9, 4}, {4, 0}, {0, 0}, {0, 9},
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
const auto cameraParameters = array{ViewParams{{4096, 2048},
                                               {1.F, 2.F, 3.F},
                                               {40.F, -60.F, 30.F},
                                               ErpParams{{-60.F, 70.F}, {-50.F, 70.F}},
                                               {0.5F, 100.F}},
                                    ViewParams{{1920, 1080},
                                               {4.F, 5.F, 6.F},
                                               {0.F, 0.F, 0.F},
                                               PerspectiveParams{{1000, 1010}, {960, 540}},
                                               {1.F, 50.F},
                                               64,
                                               {128}}};

const auto cameraParameterList = array{ViewParamsList{{cameraParameters[0]}},
                                       ViewParamsList{{cameraParameters[0], cameraParameters[0]}},
                                       ViewParamsList{{cameraParameters[0], cameraParameters[1]}}};

const auto ivsProfileTierLevel = array{IvsProfileTierLevel{}};

const auto viewingSpace = array{
    ViewingSpace{
        {{ElementaryShapeOperation::add, ElementaryShape{{PrimitiveShape{
                                                             Cuboid{{}, {}}, // primitive
                                                             {},             // guard band size
                                                             {},             // orientation
                                                             {} // viewing direction constraint
                                                         }},
                                                         {}}}}},
    ViewingSpace{{{ElementaryShapeOperation::subtract,
                   ElementaryShape{{PrimitiveShape{Spheroid{{}, {}}, {}, {}, {}}},
                                   PrimitiveShapeOperation::interpolate}},
                  {ElementaryShapeOperation::add,
                   ElementaryShape{{PrimitiveShape{Halfspace{{}, {}}, {}, {}, {}}}}}}},
    ViewingSpace{{{ElementaryShapeOperation::add, ElementaryShape{{PrimitiveShape{
                                                      Cuboid{{}, {}},
                                                      1.F,                     // guard band size
                                                      Vec3f{30.F, 60.F, 90.F}, // orientation
                                                      {} // viewing direction constraint
                                                  }}}}}},
    ViewingSpace{{{ElementaryShapeOperation::add,
                   ElementaryShape{{PrimitiveShape{Cuboid{{}, {}},
                                                   {},
                                                   {},
                                                   PrimitiveShape::ViewingDirectionConstraint{
                                                       {},
                                                       90.F, // yaw_center
                                                       30.F, // yaw_range,
                                                       45.F, // pitch_center
                                                       60.F  // pitch_range
                                                   }}}}}}},
    ViewingSpace{
        {{ElementaryShapeOperation::intersect,
          ElementaryShape{{PrimitiveShape{Cuboid{{}, {}}, 1.F, Vec3f{30.F, 45.F, 60.F},
                                          PrimitiveShape::ViewingDirectionConstraint{
                                              15.F, // guard_band_direction_size
                                              90.F, // yaw_center
                                              30.F, // yaw_range,
                                              45.F, // pitch_center
                                              60.F  // pitch_range
                                          }}}}},
         {ElementaryShapeOperation::subtract,
          ElementaryShape{{PrimitiveShape{Cuboid{{-1.F, 0.F, 1.F}, {1.F, 2.F, 3.F}}, {}, {}, {}},
                           PrimitiveShape{Spheroid{{-2.F, 2.F, 2.F}, {3.F, 2.F, 1.F}}, {}, {}, {}},
                           PrimitiveShape{Halfspace{{3.F, 3.F, 3.F}, -1.F}, {}, {}, {}}},
                          PrimitiveShapeOperation::interpolate}}}}};

const auto viewingSpaceJson = array{
    "{\"ElementaryShapes\":[{\"ElementaryShapeOperation\":\"add\",\"ElementaryShape\": "
    "{\"PrimitiveShapeOperation\": \"add\",\"PrimitiveShapes\": [{\"PrimitiveShapeType\": "
    "\"cuboid\",\"Center\":[0,0,0],\"Size\":[0,0,0]}]}}]}",
    "{\"ElementaryShapes\":[{\"ElementaryShapeOperation\":\"intersect\",\"ElementaryShape\":{"
    "\"PrimitiveShapeOperation\":\"add\",\"PrimitiveShapes\":[{\"PrimitiveShapeType\":\"Cuboid\","
    "\"GuardBandSize\":1.0,\"Rotation\":[30,45,60],\"ViewingDirectionConstraint\":{"
    "\"GuardBandDirectionSize\":15.0,\"YawCenter\":90.0,\"YawRange\":30.0,\"PitchCenter\":45.0,"
    "\"PitchRange\":60.0}}]}},{\"ElementaryShapeOperation\":\"subtract\",\"ElementaryShape\":{"
    "\"PrimitiveShapes\":[{\"PrimitiveShapeType\":\"cuboid\",\"Center\":[-1.0,0.0,1.0],\"Size\":[1."
    "0,2.0,3.0]},{\"PrimitiveShapeType\":\"spheroid\",\"Center\":[-2.0,2.0,2.0],\"Radius\":[3.0,2."
    "0,1.0]},{\"PrimitiveShapeType\":\"halfspace\",\"Normal\":[3.0,3.0,3.0],\"Distance\":-1.0}],"
    "\"PrimitiveShapeOperation\":\"interpolate\"}}]}"};

const auto ivSequenceParams =
    array{IvSequenceParams{ivsProfileTierLevel[0], cameraParameterList[0]},
          IvSequenceParams{ivsProfileTierLevel[0], cameraParameterList[1],
                           true, // low depth quality flag
                           2,    // num objects
                           2,    // max groups
                           12,   // num depth occupancy bits
                           viewingSpace[0]}};

const auto atlasParamsList = array{
    AtlasParamsList{
        {AtlasParameters{0, 0, {}, {100, 50}, {5, 4}, {34, 22}, PatchRotation::mrot90, {}, {}}},
        true,           // omaf v1 compatible flags
        {},             // no group ID's
        {{1920, 1080}}, // atlas sizes
        {false}},       // depth occ. params present flags
    AtlasParamsList{
        {AtlasParameters{0, 0, {0}, {4096, 2048}, {0, 0}, {0, 0}, PatchRotation::mrot90, {}, {}},
         AtlasParameters{0, 1, {1}, {100, 40}, {5, 4}, {34, 22}, PatchRotation::mrot180, {64}, {}},
         AtlasParameters{
             2, 1, {1}, {100, 30}, {500, 400}, {340, 220}, PatchRotation::rot180, {}, {128}}},
        true,                                 // omaf v1 compatible flag
        {{1, 0, 1}},                          // group ID's,
        {{2048, 4096}, {0, 0}, {2048, 1088}}, // atlas sizes
        {true, false, true}}};                // namespace examples

const auto ivAccessUnitParams =
    array{IvAccessUnitParams{}, IvAccessUnitParams{{atlasParamsList[1]}}};
} // namespace examples

namespace {
template <typename Type, typename... Args>
auto codingTest(const Type &reference, int size, Args &... args) -> bool {
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
template <typename Type> auto loadJson(const std::string &str) -> Type {
  istringstream stream(str);
  Json json(stream);
  return Type::loadFromJson(json);
}
} // namespace

TEST_CASE("ViewParamsList") {
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
  SECTION("ivs_profile_tier_level") { REQUIRE(codingTest(examples::ivsProfileTierLevel[0], 0)); }

  SECTION("camera_params_list") {
    const auto depthOccMapThresholdNumBits = 10U;
    REQUIRE(codingTest(examples::cameraParameterList[0], 58, depthOccMapThresholdNumBits));
    REQUIRE(codingTest(examples::cameraParameterList[1], 82, depthOccMapThresholdNumBits));
    REQUIRE(codingTest(examples::cameraParameterList[2], 115, depthOccMapThresholdNumBits));
  }

  SECTION("ivs_params") {
    REQUIRE(codingTest(examples::ivSequenceParams[0], 59));
    REQUIRE(codingTest(examples::ivSequenceParams[1], 98));
  }

  SECTION("atlas_params_list") {
    REQUIRE(codingTest(examples::atlasParamsList[0], 17, examples::ivSequenceParams[0]));
    REQUIRE(codingTest(examples::atlasParamsList[1], 47, examples::ivSequenceParams[1]));
  }

  SECTION("iv_access_unit_params") {
    REQUIRE(codingTest(examples::ivAccessUnitParams[0], 1, examples::ivSequenceParams[1]));
    REQUIRE(codingTest(examples::ivAccessUnitParams[1], 47, examples::ivSequenceParams[1]));
  }

  SECTION("viewing_space") {
    REQUIRE(codingTest(examples::viewingSpace[0], 15));
    REQUIRE(codingTest(examples::viewingSpace[1], 25));
    REQUIRE(codingTest(examples::viewingSpace[2], 23));
    REQUIRE(codingTest(examples::viewingSpace[3], 23));
    REQUIRE(codingTest(examples::viewingSpace[4], 67));
  }
}

TEST_CASE("Metadata_json") {
  SECTION("viewing_space") {
    REQUIRE(loadJson<ViewingSpace>(examples::viewingSpaceJson[0]) == examples::viewingSpace[0]);
    REQUIRE(loadJson<ViewingSpace>(examples::viewingSpaceJson[1]) == examples::viewingSpace[4]);
  }
}

TEST_CASE("OccupancyTransform") {
  SECTION("Zero threshold view") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {}, 0, 100};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, {}, {}};
    const auto transform = OccupancyTransform{viewParams, atlasParams};
    REQUIRE(transform.occupant(0));
    REQUIRE(transform.occupant(0xFFFF));
  }

  SECTION("Zero threshold patch") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {}, 100, 100};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, 0, {}};
    const auto transform = OccupancyTransform{viewParams, atlasParams};
    REQUIRE(transform.occupant(0));
    REQUIRE(transform.occupant(0xFFFF));
  }

  SECTION("Non-zero threshold view") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {}, 100, 100};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, {}, {}};
    const auto transform = OccupancyTransform{viewParams, atlasParams};
    REQUIRE(!transform.occupant(99));
    REQUIRE(transform.occupant(100));
  }

  SECTION("Non-zero threshold patch") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {}, 100, 100};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, 50, {}};
    const auto transform = OccupancyTransform{viewParams, atlasParams};
    REQUIRE(!transform.occupant(49));
    REQUIRE(transform.occupant(50));
  }
}

TEST_CASE("DepthTransform") {
  SECTION("View without depth start") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {-1.F, 4.F}, 100, {}};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, {}, {}};
    const auto transform = DepthTransform<12>{viewParams, atlasParams};

    REQUIRE(transform.expandNormDisp(0) > 0.F);
    REQUIRE(transform.expandNormDisp(1500) == -1.F + 5.F * 1500.F / 4095.F);
    REQUIRE(transform.expandNormDisp(0xFFF) == 4.F);

    REQUIRE(transform.expandDepth(0) == 1.F / transform.expandNormDisp(0));
    REQUIRE(transform.expandDepth(1500) == 1.F / transform.expandNormDisp(1500));
    REQUIRE(transform.expandDepth(0xFFF) == 1.F / transform.expandNormDisp(0xFFF));
  }

  SECTION("Quantize normalized disparity") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {-1.F, 4.F}, 100, 46};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, 33, 75};
    const auto transform = DepthTransform<13>{viewParams, atlasParams};

    REQUIRE(transform.quantizeNormDisp(0.F, 0) == 0);
    REQUIRE(transform.quantizeNormDisp(-0.1F, 0) == 0);
    REQUIRE(transform.quantizeNormDisp(4.1F, 0) == 8191);
    REQUIRE(transform.quantizeNormDisp(NaN, 0) == 0);
    REQUIRE(transform.quantizeNormDisp(-inf, 0) == 0);
    REQUIRE(transform.quantizeNormDisp(inf, 0) == 8191);

    REQUIRE(transform.quantizeNormDisp(0.1F, 0) == lround(1.1F / 5.F * 8191));
    REQUIRE(transform.quantizeNormDisp(4.F, 0) == 8191);

    REQUIRE(transform.quantizeNormDisp(0.F, 4000) == 0);
    REQUIRE(transform.quantizeNormDisp(-0.1F, 4000) == 0);
    REQUIRE(transform.quantizeNormDisp(4.1F, 4000) == 8191);
    REQUIRE(transform.quantizeNormDisp(NaN, 4000) == 0);
    REQUIRE(transform.quantizeNormDisp(-inf, 4000) == 0);
    REQUIRE(transform.quantizeNormDisp(inf, 4000) == 8191);

    REQUIRE(transform.quantizeNormDisp(0.1F, 4000) == 4000);
    REQUIRE(transform.quantizeNormDisp(4.F, 4000) == 8191);
  }

  SECTION("View with depth start") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {-1.F, 4.F}, 100, 400};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, {}, {}};
    const auto transform = DepthTransform<10>{viewParams, atlasParams};
    const auto reference = -1.F + 5.F * 400.F / 1023.F;
    REQUIRE(transform.expandNormDisp(0) == reference);
    REQUIRE(transform.expandNormDisp(399) == reference);
    REQUIRE(transform.expandNormDisp(400) == reference);
    REQUIRE(transform.expandNormDisp(401) > reference);
    REQUIRE(transform.expandNormDisp(0x3FF) == 4.F);

    REQUIRE(transform.expandDepth(0) == 1.F / transform.expandNormDisp(0));
    REQUIRE(transform.expandDepth(400) == 1.F / transform.expandNormDisp(400));
    REQUIRE(transform.expandDepth(0xFFF) == 1.F / transform.expandNormDisp(0xFFF));
  }

  SECTION("Patch that overrides depth start (1)") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {-1.F, 4.F}, 100, 400};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, {}, 50};
    const auto transform = DepthTransform<10>{viewParams, atlasParams};
    REQUIRE(transform.expandNormDisp(0) > 0);
    REQUIRE(transform.expandNormDisp(0x3FF) == 4.F);

    REQUIRE(transform.expandDepth(0) == 1.F / transform.expandNormDisp(0));
  }

  SECTION("Patch that overrides depth start (2)") {
    const auto viewParams = ViewParams{{1920, 1080}, {}, {}, ErpParams{}, {2.F, 4.F}, 100, 400};
    const auto atlasParams = AtlasParameters{3, 2, {}, {}, {}, {}, {}, {}, 50};
    const auto transform = DepthTransform<10>{viewParams, atlasParams};
    const auto reference = 2.F + 2.F * 50.F / 1023.F;
    REQUIRE(transform.expandNormDisp(0) == reference);
    REQUIRE(transform.expandNormDisp(49) == reference);
    REQUIRE(transform.expandNormDisp(50) == reference);
    REQUIRE(transform.expandNormDisp(51) > reference);
    REQUIRE(transform.expandNormDisp(0x3FF) == 4.F);

    REQUIRE(transform.expandDepth(51) == 1.F / transform.expandNormDisp(51));
  }
}
