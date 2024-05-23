/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include <TMIV/MivBitstream/AccessUnit.h>

using namespace std::string_literals;

TEST_CASE("Convert AccessUnit to SequenceConfig") {
  auto au = TMIV::MivBitstream::AccessUnit{};

  SECTION("viewParams") {
    au.viewParamsList.resize(2);
    au.viewParamsList.front().pose.position = {2.F, -3.F, 7.F};
    au.viewParamsList.back().pose.position = {6.F, 13.F, 5.F};

    au.viewParamsList.front().name = "front"s;
    au.viewParamsList.back().name = "back"s;

    au.atlas.emplace_back().texFrame.createYuv420({8, 4}, 10);
    au.atlas.back().geoFrame.createY({8, 4}, 10);

    const TMIV::MivBitstream::SequenceConfig sc = au.sequenceConfig();
    REQUIRE(sc.cameras.size() == 2);
    CHECK(sc.cameras.front().viewParams.name == "front"s);
    CHECK(sc.cameras.back().viewParams.name == "back"s);

    CHECK(sc.frameRate == 0.);

    SECTION("boundingBoxCenter is not in the bitstream, so the average position is used instead") {
      CHECK(sc.boundingBoxCenter.x() == 4.);
      CHECK(sc.boundingBoxCenter.y() == 5.);
      CHECK(sc.boundingBoxCenter.z() == 6.);
    }

    SECTION("numberOfFrames is unknown, put 0 to signal that") { CHECK(sc.numberOfFrames == 0); }

    SECTION("Video format fields need to be set") {
      for (const auto &camera : sc.cameras) {
        CHECK(camera.bitDepthTexture == 10);
        CHECK(camera.bitDepthTransparency == 0);
        CHECK(camera.bitDepthGeometry == 10);
        CHECK(camera.bitDepthEntities == 0);

        CHECK(camera.colorFormatTexture == TMIV::Common::ColorFormat::YUV420);
        CHECK(camera.colorFormatTransparency == TMIV::Common::ColorFormat::YUV420);
        CHECK(camera.colorFormatGeometry == TMIV::Common::ColorFormat::YUV420);
        CHECK(camera.colorFormatEntities == TMIV::Common::ColorFormat::YUV420);
      }
    }
  }

  SECTION("frameRate") {
    auto vui = TMIV::MivBitstream::VuiParameters{};
    vui.vui_time_scale(240).vui_num_units_in_tick(10);
    au.vui = vui;

    const TMIV::MivBitstream::SequenceConfig sc = au.sequenceConfig();
    CHECK(sc.frameRate == 24.);
  }

  SECTION("contentName") {
    const TMIV::MivBitstream::SequenceConfig sc = au.sequenceConfig();

    CHECK(!sc.contentName.empty());
  }
}

TEST_CASE("Test if patches are within the projection plane bounds") {
  using TMIV::MivBitstream::requireAllPatchesWithinProjectionPlaneBounds;

  auto vpl = TMIV::MivBitstream::ViewParamsList{};
  auto ppl = TMIV::MivBitstream::PatchParamsList{};

  SECTION("Empty lists") {
    REQUIRE_NOTHROW(requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
  }

  SECTION("Valid lists") {
    using TMIV::MivBitstream::ViewId;

    vpl.emplace_back().ci.ci_projection_plane_width_minus1(7).ci_projection_plane_height_minus1(5);
    vpl.back().viewId = ViewId{};
    vpl.emplace_back().ci.ci_projection_plane_width_minus1(9).ci_projection_plane_height_minus1(7);
    vpl.back().viewId = ViewId{1};
    vpl.constructViewIdIndex();

    ppl.emplace_back()
        .atlasPatchOrientationIndex(TMIV::MivBitstream::FlexiblePatchOrientation::FPO_NULL)
        .atlasPatch3dOffsetU(3)
        .atlasPatch3dOffsetV(2)
        .atlasPatch3dSizeU(4)
        .atlasPatch3dSizeV(4)
        .atlasPatchProjectionId(ViewId{});
    ppl.emplace_back()
        .atlasPatchOrientationIndex(TMIV::MivBitstream::FlexiblePatchOrientation::FPO_SWAP)
        .atlasPatch3dOffsetU(4)
        .atlasPatch3dOffsetV(0)
        .atlasPatch3dSizeU(6)
        .atlasPatch3dSizeV(8)
        .atlasPatchProjectionId(ViewId{1});

    REQUIRE(ppl.size() == 2);
    CHECK(ppl[0].atlasPatchProjectionId() == ViewId{});
    CHECK(ppl[1].atlasPatchProjectionId() == ViewId{1});

    requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl);

    SECTION("Invalid list: projection ID out of bounds") {
      ppl[1].atlasPatchProjectionId(ViewId{2});

      REQUIRE_THROWS(requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }

    SECTION("Invalid list: projection ID swap") {
      ppl[0].atlasPatchProjectionId(ViewId{1});
      ppl[1].atlasPatchProjectionId(ViewId{});

      REQUIRE_THROWS(requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }

    SECTION("Invalid list: offset change") {
      ppl[1].atlasPatch3dOffsetU(5);

      REQUIRE_THROWS(requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }

    SECTION("Invalid list: size change") {
      ppl[0].atlasPatch3dSizeV(7);

      REQUIRE_THROWS(requireAllPatchesWithinProjectionPlaneBounds(vpl, ppl));
    }
  }
}

TEST_CASE("Test if patches are within the atlas frame bounds") {
  using TMIV::MivBitstream::requireAllPatchesWithinAtlasFrameBounds;

  const auto asps_atlas_width = GENERATE(7, 64);
  const auto asps_atlas_height = GENERATE(24, 77);

  const auto asps = [=]() {
    return TMIV::MivBitstream::AtlasSequenceParameterSetRBSP{}
        .asps_frame_width(asps_atlas_width)
        .asps_frame_height(asps_atlas_height);
  }();

  auto ppl = TMIV::MivBitstream::PatchParamsList{};

  SECTION("Empty list") { REQUIRE_NOTHROW(requireAllPatchesWithinAtlasFrameBounds(ppl, asps)); }

  ppl.emplace_back().atlasPatch2dPosX(3).atlasPatch2dPosY(2).atlasPatch2dSizeY(4).atlasPatch2dSizeX(
      asps_atlas_width - 3);
  ppl.emplace_back().atlasPatch2dPosX(1).atlasPatch2dPosY(1).atlasPatch2dSizeX(6).atlasPatch2dSizeY(
      asps_atlas_height - 1);

  SECTION("Valid list") {
    REQUIRE(ppl.size() == 2);
    requireAllPatchesWithinAtlasFrameBounds(ppl, asps);
  }

  SECTION("Invalid list: atlasPatch2dPosX +1") {
    ppl[0].atlasPatch2dPosX(ppl[0].atlasPatch2dPosX() + 1);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: atlasPatch2dPosY +1") {
    ppl[1].atlasPatch2dPosY(ppl[1].atlasPatch2dPosY() + 1);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: atlasPatch2dSizeX +1") {
    ppl[0].atlasPatch2dSizeX(ppl[0].atlasPatch2dSizeX() + 1);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: atlasPatch2dSizeY +1") {
    ppl[1].atlasPatch2dSizeY(ppl[1].atlasPatch2dSizeY() + 1);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: Negative atlasPatch2dPosX") {
    const auto x = GENERATE(-64, -1);
    ppl.emplace_back().atlasPatch2dPosX(x);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: Negative atlasPatch2dPosY") {
    const auto y = GENERATE(-64, -1);
    ppl.emplace_back().atlasPatch2dPosY(y);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: Negative atlasPatch2dSizeX") {
    const auto x = GENERATE(-64, -1);
    ppl.emplace_back().atlasPatch2dSizeX(x);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }

  SECTION("Invalid list: Negative atlasPatch2dSizeY") {
    const auto y = GENERATE(-64, -1);
    ppl.emplace_back().atlasPatch2dSizeY(y);
    REQUIRE_THROWS(requireAllPatchesWithinAtlasFrameBounds(ppl, asps));
  }
}
