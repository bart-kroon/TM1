/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/Decoder/OutputLog.h>

TEST_CASE("Decoder::writeFrameToOutputLog") {
  using TMIV::Decoder::writeFrameToOutputLog;
  using TMIV::MivBitstream::AccessUnit;
  using TMIV::MivBitstream::AtlasId;
  using TMIV::MivBitstream::ViewId;

  std::ostringstream stream;

  SECTION("Minimal example") {
    auto frame = AccessUnit{};
    frame.atlas.emplace_back().asps.asps_frame_width(8).asps_frame_height(5);
    frame.atlas.back().blockToPatchMap.getPlanes().emplace_back();
    writeFrameToOutputLog(frame, stream);
    std::string reference = "-1 0 8 5 00000000 00000000 00000000 00000000\n";
    REQUIRE(stream.str() == reference);

    SECTION("Same frame twice gives two equal rows") {
      writeFrameToOutputLog(frame, stream);
      reference += reference;
      REQUIRE(stream.str() == reference);
    }

    SECTION("Frame order count") {
      frame.foc = 7;
      reference += "7 0 8 5 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(stream.str() == reference);
    }

    SECTION("Atlas ID") {
      frame.vps.vps_atlas_id(0, AtlasId{7});
      reference += "-1 7 8 5 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(stream.str() == reference);
    }

    SECTION("Video frame hash") {
      REQUIRE(TMIV::Decoder::videoDataHash(frame.atlas.front()) == 0);

      frame.atlas.front().decOccFrame.createY({8, 10}, 10);
      frame.atlas.front().decOccFrame.fillValue(23);
      reference += "-1 0 8 5 e9bf42e6 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::videoDataHash(frame.atlas.front()) == 0xE9BF42E6);
      REQUIRE(stream.str() == reference);

      frame.atlas.front().decGeoFrame.createY({10, 4}, 10);
      frame.atlas.front().decGeoFrame.fillValue(25);
      reference += "-1 0 8 5 14c31f8e 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::videoDataHash(frame.atlas.front()) == 0x14C31F8E);
      REQUIRE(stream.str() == reference);

      frame.atlas.front().decAttrFrame.emplace_back().createYuv444({12, 26}, 10);
      frame.atlas.front().decAttrFrame.back().fillValue(100);
      reference += "-1 0 8 5 7afd08cd 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::videoDataHash(frame.atlas.front()) == 0x7AFD08CD);
      REQUIRE(stream.str() == reference);

      frame.atlas.front().decAttrFrame.emplace_back().createY({10, 8});
      frame.atlas.front().decAttrFrame.back().fillValue(88);
      reference += "-1 0 8 5 4033b1a2 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::videoDataHash(frame.atlas.front()) == 0x4033B1A2);
      REQUIRE(stream.str() == reference);
    }

    SECTION("Block to patch map") {
      REQUIRE(TMIV::Decoder::blockToPatchMapHash(frame.atlas.front()) == 0);

      frame.atlas.front().blockToPatchMap.createY({8, 10});
      frame.atlas.front().blockToPatchMap.fillValue(23);
      frame.atlas.front().blockToPatchMap.getPlane(0)(3, 4) = TMIV::Common::unusedPatchIdx;
      reference += "-1 0 8 5 00000000 59fe0999 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::blockToPatchMapHash(frame.atlas.front()) == 0x59FE0999);
      REQUIRE(stream.str() == reference);
    }

    SECTION("Unused patch ID translates to 0xFFFFFFFF") {
      const auto value =
          GENERATE(uint16_t{}, uint16_t{1}, uint16_t{1000}, uint16_t{TMIV::Common::unusedPatchIdx});
      CAPTURE(value);

      // Video frames use the same hash function but do not have this logic
      frame.atlas.front().decOccFrame.createY({2, 2}, 10);
      frame.atlas.front().decOccFrame.fillValue(value);
      const auto reference2 = TMIV::Decoder::videoDataHash(frame.atlas.front());

      frame.atlas.front().blockToPatchMap.createY({2, 2});
      frame.atlas.front().blockToPatchMap.fillValue(value);
      const auto actual = TMIV::Decoder::blockToPatchMapHash(frame.atlas.front());

      if (value == TMIV::Common::unusedPatchIdx) {
        REQUIRE(actual != reference2);
      } else {
        REQUIRE(actual == reference2);
      }
    }

    SECTION("Patch params list") {
      REQUIRE(TMIV::Decoder::patchParamsListHash(frame.atlas.front().patchParamsList) == 0);

      frame.atlas.front()
          .patchParamsList.emplace_back()
          .atlasPatch3dOffsetD(3)
          .atlasPatch3dOffsetU(2)
          .atlasPatch3dOffsetV(100)
          .atlasPatch3dRangeD(1000)
          .atlasPatchAttributeOffset({6, 7, 9})
          .atlasPatchDepthOccThreshold(50)
          .atlasPatchEntityId(100)
          .atlasPatchInpaintFlag(true)
          .atlasPatchLoDScaleX(5)
          .atlasPatchLoDScaleY(4)
          .atlasPatchOrientationIndex(TMIV::MivBitstream::FlexiblePatchOrientation::FPO_MROT180)
          .atlasPatchProjectionId(ViewId{10});

      reference += "-1 0 8 5 00000000 00000000 659eb2c7 00000000\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::patchParamsListHash(frame.atlas.front().patchParamsList) ==
              0x659eb2c7);
      REQUIRE(stream.str() == reference);
    }

    SECTION("View params list") {
      using TMIV::MivBitstream::CiCamType;

      REQUIRE(TMIV::Decoder::viewParamsListHash(frame.viewParamsList) == 0);

      for (int32_t i = 0; i < 3; ++i) {
        auto &v = frame.viewParamsList.emplace_back();

        v.ci.ci_cam_type(static_cast<TMIV::MivBitstream::CiCamType>(i));
        v.ci.ci_projection_plane_width_minus1(99);
        v.ci.ci_projection_plane_height_minus1(201);

        switch (v.ci.ci_cam_type()) {
        case CiCamType::equirectangular:
          v.ci.ci_erp_phi_min(-1.);
          v.ci.ci_erp_phi_max(2.);
          v.ci.ci_erp_theta_min(0.5);
          v.ci.ci_erp_theta_max(1.);
          break;
        case CiCamType::perspective:
          v.ci.ci_perspective_focal_hor(1.);
          v.ci.ci_perspective_focal_ver(2.);
          v.ci.ci_perspective_center_hor(3.);
          v.ci.ci_perspective_center_ver(4.);
          break;
        case CiCamType::orthographic:
          v.ci.ci_ortho_width(5.);
          v.ci.ci_ortho_height(6.);
          break;
        }

        v.pose.position = {1., 2., 3.};
        v.pose.orientation = {4., 5., 6., 7.};

        v.dq.dq_norm_disp_low(7.);
        v.dq.dq_norm_disp_high(8.);
        v.dq.dq_depth_occ_threshold_default(40);
      }

      reference += "-1 0 8 5 00000000 00000000 00000000 f6b9ee21\n";
      writeFrameToOutputLog(frame, stream);
      REQUIRE(TMIV::Decoder::viewParamsListHash(frame.viewParamsList) == 0xf6b9ee21);
      REQUIRE(stream.str() == reference);
    }
  }

  SECTION("Multiple atlases") {
    auto frame = AccessUnit{};
    frame.vps.vps_atlas_count_minus1(1).vps_atlas_id(1, AtlasId{3});
    frame.atlas.emplace_back().asps.asps_frame_width(8).asps_frame_height(5);
    frame.atlas.back().blockToPatchMap.getPlanes().emplace_back();
    frame.atlas.emplace_back().asps.asps_frame_width(13).asps_frame_height(6);
    frame.atlas.back().blockToPatchMap.getPlanes().emplace_back();
    writeFrameToOutputLog(frame, stream);
    std::string reference = R"(-1 0 8 5 00000000 00000000 00000000 00000000
-1 3 13 6 00000000 00000000 00000000 00000000
)";
    REQUIRE(stream.str() == reference);
  }
}
