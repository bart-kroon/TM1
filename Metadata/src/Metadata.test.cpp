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

#include <TMIV/Metadata/AtlasParametersList.h>

using namespace std;
using namespace TMIV::Metadata;

using namespace TMIV::Common;

static Vec2f imagePosition(Vec2f atlas, const AtlasParameters &patch) {
  // [FT, 7-aug-2019] : at the synthesizer level, only UR_none and
  //                    CCW_none cases are tested, since the packer
  //                    itself has not been modified
  const auto posInAtlas = Vec2f(patch.posInAtlas); // (xP, yP)
  const auto posInView = Vec2f(patch.posInView);   // (xM, yM)
  const auto patchSize = Vec2f(patch.patchSize);   // (w, h)
  float i, j;

  if (patch.flip == PatchFlip::none) {
    switch (patch.rotation) {
    case PatchRotation::upright:
      i = atlas.y() - posInAtlas.y() + posInView.y();
      j = atlas.x() - posInAtlas.x() + posInView.x();
      return Vec2f{ j, i };
    case PatchRotation::ccw:
      i = atlas.x() - posInAtlas.x() + posInView.y();
      j = -atlas.y() + posInAtlas.y() + posInView.x() + patchSize.x() - 1;
      return Vec2f{ j, i };
    case PatchRotation::ht:
      i = -atlas.y() + posInAtlas.y() + posInView.y() + patchSize.y() - 1;
      j = -atlas.x() + posInAtlas.x() + posInView.x() + patchSize.x() - 1;
      return Vec2f{ j, i };
    case PatchRotation::cw:
      i = -atlas.x() + posInAtlas.x() + posInView.y() + patchSize.y() - 1;
      j = atlas.y() - posInAtlas.y() + posInView.x();
      return Vec2f{ j, i };
    default:
      abort();
    }
  }
  else { // patch.flip == PatchFlip::vflip
    switch (patch.rotation) {
    case PatchRotation::upright:
      i = -atlas.y() + posInAtlas.y() + posInView.y() + patchSize.y() - 1;
      j = atlas.x() - posInAtlas.x() + posInView.x();
      return Vec2f{ j, i };
    case PatchRotation::ccw:
      i = atlas.x() - posInAtlas.x() + posInView.y();
      j = atlas.y() - posInAtlas.y() + posInView.x();
      return Vec2f{ j, i };
    case PatchRotation::ht:
      i = atlas.y() - posInAtlas.y() + posInView.y();
      j = -atlas.x() + posInAtlas.x() + posInView.x() + patchSize.x() - 1;
      return Vec2f{ j, i };
    case PatchRotation::cw:
      i = -atlas.x() + posInAtlas.x() + posInView.y() + patchSize.y() - 1;
      j = -atlas.y() + posInAtlas.y() + posInView.x() + patchSize.x() - 1;
      return Vec2f{ j, i };
    default:
      abort();
    }
  }
}

Vec2i atlasToViewMod(Vec2i atlasPosition, const AtlasParameters &patch)
{
  Vec2f atlasPosition2f{ float(atlasPosition[0]) , float(atlasPosition[1]) };

  auto viewPosition2f = imagePosition(atlasPosition2f, patch);

  Vec2i viewPosition{ int(viewPosition2f[0]), int(viewPosition2f[1]) };
  return viewPosition;

}

TEST_CASE("TestPatchRotationAndFlipTransforms") {
  AtlasParameters patch;

  patch.atlasId = 0;
  patch.viewId = 0;
  patch.patchSize = {10, 5};
  patch.posInView = {0, 0};
  patch.posInAtlas = {10, 20};
  patch.rotation = PatchRotation::upright;
  patch.flip = PatchFlip::none;

  auto flips = {PatchFlip::none, PatchFlip::vflip};
  auto rotations = {PatchRotation::upright, PatchRotation::ccw,
                    PatchRotation::ht, PatchRotation::cw};

  SECTION("EvaluateTransformationOf_TopLeft") {
    Vec2i posInViewEncode = {0, 0};
    std::vector<Vec2i> posInAtlasExpected = {{0, 0}, {0, 9}, {9, 4}, {4, 0},
                                             {0, 4}, {0, 0}, {9, 0}, {4, 9}};
    for (auto &pos : posInAtlasExpected)
      pos += patch.posInAtlas;

    int i = 0;
    for (auto flip : flips)
      for (auto rotation : rotations) {
        patch.flip = flip;
        patch.rotation = rotation;
        auto posInAtlas = viewToAtlas(posInViewEncode, patch);
        REQUIRE(posInAtlas == posInAtlasExpected[i]);

        //auto posInViewDecode = atlasToView(posInAtlas, patch);
        auto posInViewDecode = atlasToViewMod(posInAtlas, patch);
        REQUIRE(posInViewDecode == posInViewEncode);

        ++i;
      }
  }

  SECTION("EvaluateTransformationOf_BottomRight") {
    Vec2i posInViewEncode = {9, 4};
    std::vector<Vec2i> posInAtlasExpected = {{9, 4}, {4, 0}, {0, 0}, {0, 9},
                                             {9, 0}, {4, 9}, {0, 4}, {0, 0}};
    for (auto &pos : posInAtlasExpected)
      pos += patch.posInAtlas;

    int i = 0;
    for (auto flip : flips)
      for (auto rotation : rotations) {
        patch.flip = flip;
        patch.rotation = rotation;
        auto posInAtlas = viewToAtlas(posInViewEncode, patch);
        REQUIRE(posInAtlas == posInAtlasExpected[i]);

        auto posInViewDecode = atlasToViewMod(posInAtlas, patch);
        REQUIRE(posInViewDecode == posInViewEncode);

        ++i;
      }
  }
}


