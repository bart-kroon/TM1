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

#include <TMIV/IO/IO.h>

#include <sstream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::IO;

namespace {
auto examplePatch() -> AtlasParameters {
  return {
      uint8_t{1},        // atlasId,
      uint8_t{3},        // viewId
      Vec2i{16, 32},     // patchSize
      Vec2i{3, 4},       // posInView
      Vec2i{8, 12},      // posInPatch
      PatchRotation::ccw // rotation
  };
}

auto exampleCamera() -> CameraParameters {
  return {
      Vec2i{3000, 2000},           // size
      Vec3f{1.f, -2.f, 3.f},       // position
      Vec3f{3.f, 4.f, 5.f},        // rotation
      ProjectionType::Perspective, // type
      Vec2f{-90.f, 70.f},          // erpPhiRange
      Vec2f{-60.f, 80.f},          // erpThetaRange
      CubicMapType{},              // cubicMapType
      Vec2f{},                     // perspectiveFocal
      Vec2f{},                     // perspectiveCenter
      Vec2f{1.f, 100.f}            // depthRange
  };
}

auto exampleMetadata() -> MivMetadata {
  return {vector<Vec2i>{{4000, 3000}}, // atlasSize
          true,                        // omafV1CompatibleFlag
          {examplePatch()},            // patches
          {exampleCamera()}};          // cameras
}

auto minimalConfig() -> Json {
  auto stream = istringstream{R"(
{
	"OutputDirectory": ".",
	"AtlasMetadataPath": "IO.test.bit"
}
)"};
  return Json{stream};
}
} // namespace

TEST_CASE("save- and loadMivMetadata") {
  auto config = minimalConfig();
  auto reference = exampleMetadata();
  auto frames = {0, 32, 48};

  for (auto frame : frames) {
    saveMivMetadata(config, frame, reference);
  }

  for (auto frame : frames) {
    auto actual = loadMivMetadata(config, frame);
    REQUIRE(actual == reference);
  }
}
