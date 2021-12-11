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
  using TMIV::MivBitstream::AspsMivExtension;
  using TMIV::MivBitstream::AtlasId;
  using TMIV::MivBitstream::ViewId;

  std::ostringstream stream;

  SECTION("Minimal example") {
    auto frame = AccessUnit{};
    frame.atlas.emplace_back().asps.asps_frame_width(8).asps_frame_height(5);
    frame.atlas.back().blockToPatchMap.getPlanes().emplace_back();
    writeFrameToOutputLog(frame, stream);
    std::string reference = "-1 0 8 5 00000000 00000000 00000000 00000000 00000000 00000000\n";
    CHECK(stream.str() == reference);

    SECTION("Same frame twice gives two equal rows") {
      writeFrameToOutputLog(frame, stream);
      reference += reference;
      CHECK(stream.str() == reference);
    }

    SECTION("Frame order count") {
      frame.foc = 7;
      reference += "7 0 8 5 00000000 00000000 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(stream.str() == reference);
    }

    SECTION("Atlas ID") {
      frame.vps.vps_atlas_id(0, AtlasId{7});
      reference += "-1 7 8 5 00000000 00000000 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(stream.str() == reference);
    }

    SECTION("Video frame hash") {
      frame.atlas.front().decOccFrame.createY({8, 10}, 10);
      frame.atlas.front().decOccFrame.fillValue(23);
      reference += "-1 0 8 5 O e9bf42e6 00000000 00000000 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::videoDataHash(frame.atlas.front().decOccFrame) == 0xE9BF42E6);
      CHECK(stream.str() == reference);

      frame.atlas.front().decGeoFrame.createY({10, 4}, 10);
      frame.atlas.front().decGeoFrame.fillValue(25);
      reference +=
          "-1 0 8 5 O e9bf42e6 G 5b494eb1 00000000 00000000 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::videoDataHash(frame.atlas.front().decGeoFrame) == 0x5B494EB1);
      CHECK(stream.str() == reference);

      frame.atlas.front().decAttrFrame.emplace_back().createYuv444({12, 26}, 10);
      frame.atlas.front().decAttrFrame.back().fillValue(100);
      reference += "-1 0 8 5 O e9bf42e6 G 5b494eb1 A 0 8e0c7a24 00000000 00000000 00000000 "
                   "00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::videoDataHash(frame.atlas.front().decAttrFrame.back()) == 0x8E0C7A24);
      CHECK(stream.str() == reference);

      frame.atlas.front().decAttrFrame.emplace_back().createY({10, 8});
      frame.atlas.front().decAttrFrame.back().fillValue(88);
      reference += "-1 0 8 5 O e9bf42e6 G 5b494eb1 A 0 8e0c7a24 1 df8c9c40 00000000 00000000 "
                   "00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::videoDataHash(frame.atlas.front().decAttrFrame.back()) == 0xDF8C9C40);
      CHECK(stream.str() == reference);
    }

    SECTION("Block to patch map") {
      CHECK(TMIV::Decoder::blockToPatchMapHash(frame.atlas.front()) == 0);

      frame.atlas.front().blockToPatchMap.createY({8, 10});
      frame.atlas.front().blockToPatchMap.fillValue(23);
      frame.atlas.front().blockToPatchMap.getPlane(0)(3, 4) = TMIV::Common::unusedPatchIdx;
      reference += "-1 0 8 5 59fe0999 00000000 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::blockToPatchMapHash(frame.atlas.front()) == 0x59FE0999);
      CHECK(stream.str() == reference);
    }

    SECTION("Unused patch ID translates to 0xFFFFFFFF") {
      const auto value =
          GENERATE(uint16_t{}, uint16_t{1}, uint16_t{1000}, uint16_t{TMIV::Common::unusedPatchIdx});
      CAPTURE(value);

      // Video frames use the same hash function but do not have this logic
      frame.atlas.front().decOccFrame.createY({2, 2}, 10);
      frame.atlas.front().decOccFrame.fillValue(value);
      const auto reference2 = TMIV::Decoder::videoDataHash(frame.atlas.front().decOccFrame);

      frame.atlas.front().blockToPatchMap.createY({2, 2});
      frame.atlas.front().blockToPatchMap.fillValue(value);
      const auto actual = TMIV::Decoder::blockToPatchMapHash(frame.atlas.front());

      if (value == TMIV::Common::unusedPatchIdx) {
        CHECK(actual != reference2);
      } else {
        CHECK(actual == reference2);
      }
    }

    SECTION("Patch params list") {
      CHECK(TMIV::Decoder::patchParamsListHash(frame.atlas.front().patchParamsList) == 0);

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

      reference += "-1 0 8 5 00000000 659eb2c7 00000000 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::patchParamsListHash(frame.atlas.front().patchParamsList) == 0x659eb2c7);
      CHECK(stream.str() == reference);
    }

    SECTION("View params list") {
      using TMIV::MivBitstream::CiCamType;

      CHECK(TMIV::Decoder::viewParamsListHash(frame.viewParamsList) == 0);

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

        v.viewInpaintFlag = i == 1;

        switch (i) {
        case 0:
          break;
        case 1:
          v.pp = TMIV::MivBitstream::PruningParents{};
          break;
        case 2:
          v.pp = TMIV::MivBitstream::PruningParents{{3, 4}};
          break;
        }
      }

      reference += "-1 0 8 5 00000000 00000000 d873633b 00000000 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::viewParamsListHash(frame.viewParamsList) == 0xd873633b);
      CHECK(stream.str() == reference);
    }

    SECTION("ASPS MIV extension") {
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0);

      frame.atlas.front().asps.asps_miv_extension_present_flag(true);
      frame.atlas.front().asps.asps_miv_extension() = AspsMivExtension{};
      reference += "-1 0 8 5 00000000 00000000 00000000 d3c8a549 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0xd3c8a549);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_ancillary_atlas_flag(true);
      reference += "-1 0 8 5 00000000 00000000 00000000 ad101508 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0xad101508);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_embedded_occupancy_enabled_flag(true);
      frame.atlas.front().asps.asps_miv_extension().asme_depth_occ_threshold_flag(false);
      reference += "-1 0 8 5 00000000 00000000 00000000 22483a1b 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0x22483a1b);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_geometry_scale_enabled_flag(true);
      frame.atlas.front().asps.asps_miv_extension().asme_geometry_scale_factor_x_minus1(5);
      frame.atlas.front().asps.asps_miv_extension().asme_geometry_scale_factor_y_minus1(7);
      reference += "-1 0 8 5 00000000 00000000 00000000 a42119c3 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0xa42119c3);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_embedded_occupancy_enabled_flag(false);
      frame.atlas.front().asps.asps_miv_extension().asme_occupancy_scale_enabled_flag(true);
      frame.atlas.front().asps.asps_miv_extension().asme_occupancy_scale_factor_x_minus1(10);
      frame.atlas.front().asps.asps_miv_extension().asme_occupancy_scale_factor_y_minus1(11);
      reference += "-1 0 8 5 00000000 00000000 00000000 aaffd18b 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0xaaffd18b);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_patch_constant_depth_flag(true);
      reference += "-1 0 8 5 00000000 00000000 00000000 2d591ac8 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0x2d591ac8);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_patch_attribute_offset_enabled_flag(true);
      frame.atlas.front().asps.asps_miv_extension().asme_patch_attribute_offset_bit_depth_minus1(
          12);
      reference += "-1 0 8 5 00000000 00000000 00000000 16fb3059 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0x16fb3059);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_max_entity_id(13);
      reference += "-1 0 8 5 00000000 00000000 00000000 ee6bf4e8 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0xee6bf4e8);
      CHECK(stream.str() == reference);

      frame.atlas.front().asps.asps_miv_extension().asme_inpaint_enabled_flag(true);
      reference += "-1 0 8 5 00000000 00000000 00000000 996cc47e 00000000 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::asmeHash(frame.atlas.front()) == 0x996cc47e);
      CHECK(stream.str() == reference);
    }

    SECTION("AFPS MIV extension") {
      CHECK(TMIV::Decoder::afmeHash(frame.atlas.front()) == 0);

      auto &afme = frame.atlas.front().afps.afps_miv_extension();
      reference += "-1 0 8 5 00000000 00000000 00000000 00000000 7bd5c66f 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::afmeHash(frame.atlas.front()) == 0x7bd5c66f);
      CHECK(stream.str() == reference);

      afme.afme_inpaint_lod_enabled_flag(true);
      afme.afme_inpaint_lod_scale_x_minus1(3);
      afme.afme_inpaint_lod_scale_y_idc(2);
      reference += "-1 0 8 5 00000000 00000000 00000000 00000000 c500c9d0 00000000\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::afmeHash(frame.atlas.front()) == 0xc500c9d0);
      CHECK(stream.str() == reference);
    }

    SECTION("CASPS MIV extension") {
      CHECK(TMIV::Decoder::casmeHash(frame) == 0);

      frame.casps = TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP{};
      auto &casme = frame.casps->casps_miv_extension();
      reference += "-1 0 8 5 00000000 00000000 00000000 00000000 00000000 46b5efdf\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::casmeHash(frame) == 0x46b5efdf);
      CHECK(stream.str() == reference);

      casme.casme_depth_low_quality_flag(true);
      casme.casme_depth_quantization_params_present_flag(false);
      casme.vui_parameters({});
      reference += "-1 0 8 5 00000000 00000000 00000000 00000000 00000000 1ba9e2ba\n";
      writeFrameToOutputLog(frame, stream);
      CHECK(TMIV::Decoder::casmeHash(frame) == 0x1ba9e2ba);
      CHECK(stream.str() == reference);
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
    std::string reference = R"(-1 0 8 5 00000000 00000000 00000000 00000000 00000000 00000000
-1 3 13 6 00000000 00000000 00000000 00000000 00000000 00000000
)";
    CHECK(stream.str() == reference);
  }
}
