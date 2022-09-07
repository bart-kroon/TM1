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

#include <catch2/catch.hpp>

#include <TMIV/PtlChecker/PtlChecker.h>

using CG = TMIV::MivBitstream::PtlProfileCodecGroupIdc;
using TS = TMIV::MivBitstream::PtlProfileToolsetIdc;
using RC = TMIV::MivBitstream::PtlProfileReconstructionIdc;
using LV = TMIV::MivBitstream::PtlLevelIdc;
using VUT = TMIV::MivBitstream::VuhUnitType;
using VUH = TMIV::MivBitstream::V3cUnitHeader;
using NUT = TMIV::MivBitstream::NalUnitType;
using NUH = TMIV::MivBitstream::NalUnitHeader;
using ATI = TMIV::MivBitstream::AiAttributeTypeId;
using ASPS = TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using APM = TMIV::MivBitstream::AtduPatchMode;
using VET = TMIV::MivBitstream::VpsExtensionType;

using TMIV::Common::contains;
using TMIV::Common::downCast;
using TMIV::Common::Frame;
using TMIV::Common::Vec2i;
using TMIV::MivBitstream::AccessUnit;
using TMIV::MivBitstream::AthType;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasId;
using TMIV::MivBitstream::AtlasTileDataUnit;
using TMIV::MivBitstream::AtlasTileLayerRBSP;
using TMIV::MivBitstream::AttributeInformation;
using TMIV::MivBitstream::CommonAtlasFrameRBSP;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::mivToolsetProfileComponents;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;
using TMIV::MivBitstream::PackingInformation;
using TMIV::MivBitstream::PatchInformationData;
using TMIV::MivBitstream::ProfileTierLevel;
using TMIV::MivBitstream::ProfileToolsetConstraintsInformation;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::VpsExtensionType;
using TMIV::MivBitstream::VuiParameters;
using TMIV::PtlChecker::PtlChecker;

namespace test {
namespace {
class Exception : public std::runtime_error {
public:
  using runtime_error::runtime_error;
};

void logger(const std::string &warning) { throw Exception(warning); }

auto unit() {
  auto result = PtlChecker{};
  result.replaceLogger(&logger);
  TMIV::Common::replaceLoggingStrategy([](auto &&...) {});
  return result;
}

[[nodiscard]] constexpr auto vpccToolset(TS toolsetIdc) noexcept {
  return toolsetIdc == TS::VPCC_Basic || toolsetIdc == TS::VPCC_Extended;
}

[[nodiscard]] constexpr auto mivToolset(TS toolsetIdc) noexcept {
  return contains(mivToolsetProfileComponents, toolsetIdc);
}

constexpr auto size = Vec2i{72, 24};

auto allowedAttrTypeIds(TS toolsetIdc) {
  switch (toolsetIdc) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    return std::vector{ATI::ATTR_TEXTURE, ATI::ATTR_MATERIAL_ID, ATI::ATTR_TRANSPARENCY,
                       ATI::ATTR_REFLECTANCE, ATI::ATTR_NORMAL};
  case TS::MIV_Main:
  case TS::MIV_Geometry_Absent:
    return std::vector{ATI::ATTR_TEXTURE};
  case TS::MIV_Extended:
    return std::vector{ATI::ATTR_TEXTURE, ATI::ATTR_TRANSPARENCY};
  default:
    UNREACHABLE;
  }
}

void setAttributeTypeId(V3cParameterSet &vps, AtlasId atlasId, uint8_t attrIdx, ATI attrTypeId) {
  auto &ai = vps.attribute_information(atlasId);
  ai.ai_attribute_type_id(attrIdx, attrTypeId);

  const auto toolsetIdc = vps.profile_tier_level().ptl_profile_toolset_idc();

  if (toolsetIdc == TS::MIV_Extended && attrTypeId == ATI::ATTR_TRANSPARENCY) {
    ai.ai_attribute_dimension_minus1(attrIdx, 0);
  } else {
    ai.ai_attribute_dimension_minus1(attrIdx, 2);
  }
}

void addAttributes(V3cParameterSet &vps, AtlasId atlasId, uint8_t attrCount) {
  auto &ai = vps.attribute_information(atlasId);
  ai.ai_attribute_count(attrCount);

  const auto toolsetIdc = vps.profile_tier_level().ptl_profile_toolset_idc();
  const auto allowedAti = allowedAttrTypeIds(toolsetIdc);

  for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
    const auto n = (i + vps.indexOf(atlasId)) % allowedAti.size();
    setAttributeTypeId(vps, atlasId, i, allowedAti[n]);
  }
}

// Create an example VPS that is within PTL constraints
auto vps(CG codecGroupIdc = CG::HEVC444, TS toolsetIdc = TS::MIV_Main,
         bool restrictedGeometry = false) {
  auto result = V3cParameterSet{};

  // Set profile
  auto &ptl = result.profile_tier_level();
  ptl.ptl_profile_codec_group_idc(codecGroupIdc)
      .ptl_profile_toolset_idc(toolsetIdc)
      .ptl_profile_reconstruction_idc(RC::Rec_Unconstrained)
      .ptl_level_idc(LV::Level_4_5);

  // Set MIV Extended sub-profile
  if (restrictedGeometry) {
    PRECONDITION(toolsetIdc == TS::MIV_Extended);
    auto ptc = ProfileToolsetConstraintsInformation{};
    ptc.ptc_restricted_geometry_flag(true);
    ptl.ptl_profile_toolset_constraints_information(ptc);
  }

  // When allowed, use multiple atlases to test all allowed combinations of OVD, GVD, AVD and PVD
  switch (toolsetIdc) {
  case TS::VPCC_Basic:
  case TS::VPCC_Extended:
    break;
  case TS::MIV_Main:
    result.vps_atlas_count_minus1(1);
    break;
  case TS::MIV_Extended:
    result.vps_atlas_count_minus1(restrictedGeometry ? 0 : 7);
    break;
  case TS::MIV_Geometry_Absent:
    result.vps_atlas_count_minus1(1);
    break;
  }

  for (uint8_t k = 0; k <= result.vps_atlas_count_minus1(); ++k) {
    const auto j = AtlasId{10 + k};
    result.vps_atlas_id(k, j).vps_frame_width(j, size.x()).vps_frame_height(j, size.y());

    switch (toolsetIdc) {
    case TS::VPCC_Basic:
    case TS::VPCC_Extended:
      result.vps_occupancy_video_present_flag(j, true);
      result.vps_geometry_video_present_flag(j, true);
      result.vps_attribute_video_present_flag(j, true);
      break;
    case TS::MIV_Main:
      result.vps_geometry_video_present_flag(j, true);
      result.vps_attribute_video_present_flag(j, (k & 1) == 1);
      break;
    case TS::MIV_Extended:
      if (restrictedGeometry) {
        result.vps_attribute_video_present_flag(j, true);
      } else {
        result.vps_occupancy_video_present_flag(j, (k & 1) == 1);
        result.vps_geometry_video_present_flag(j, (k & 2) == 2);
        result.vps_attribute_video_present_flag(j, (k & 4) == 4);
      }
      break;
    case TS::MIV_Geometry_Absent:
      result.vps_attribute_video_present_flag(j, true);
      break;
    }

    if (result.vps_occupancy_video_present_flag(j)) {
      result.occupancy_information(j, {});
    }

    if (result.vps_geometry_video_present_flag(j)) {
      result.geometry_information(j, {});
    }

    if (result.vps_attribute_video_present_flag(j)) {
      addAttributes(result, j, restrictedGeometry ? 2 : 1);
    }
  }

  if (mivToolset(toolsetIdc)) {
    auto &vme = result.vps_miv_extension();
    vme.vme_embedded_occupancy_enabled_flag(toolsetIdc == TS::MIV_Main);
  }

  return result;
}

// Create an example VPS that is within PTL constraints [overload]
auto vps(TS toolsetIdc, bool restrictedGeometry = false) {
  return vps(CG::HEVC444, toolsetIdc, restrictedGeometry);
}

// Create an example ASPS that is within PTL constraints
auto asps(const V3cParameterSet &vps, AtlasId atlasId) {
  auto result = ASPS{}
                    .asps_frame_width(vps.vps_frame_width(atlasId))
                    .asps_frame_height(vps.vps_frame_height(atlasId));

  const auto &ptl = vps.profile_tier_level();

  if (ptl.ptl_profile_toolset_idc() == TS::MIV_Extended &&
      !vps.vps_geometry_video_present_flag(atlasId)) {
    auto &asme = result.asps_miv_extension();
    asme.asme_patch_constant_depth_flag(true);
  }

  return result;
}
} // namespace
} // namespace test

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CHECK_THROWS_IFF(expression, condition)                                                    \
  if ((condition)) {                                                                               \
    CHECK_THROWS_AS((expression), test::Exception);                                                \
  } else {                                                                                         \
    CHECK_NOTHROW(expression);                                                                     \
  }

TEST_CASE("By default the PtlChecker logs warnings") {
  auto unit = PtlChecker{};
  const auto nuh = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 2};
  CHECK_NOTHROW(unit.checkNuh(nuh));
}

TEST_CASE("PtlChecker ISO/IEC DIS 23090-5(2E):2021 A.1") {
  auto unit = test::unit();

  SECTION("The temporal ID of an atlas sub-bitstream shall be equal to 0") {
    const auto nuh1 = NUH{NUT::NAL_IDR_N_LP, 0, 1};
    const auto nuh2 = NUH{NUT::NAL_IDR_N_LP, 0, 2};

    CHECK_NOTHROW(unit.checkNuh(nuh1));
    CHECK_THROWS_AS(unit.checkNuh(nuh2), test::Exception);
  }
}

TEST_CASE("PtlChecker ISO/IEC DIS 23090-5(2E):2021 Table A-1") {
  auto unit = test::unit();

  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(CG::AVC_Progressive_High)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(CG::HEVC_Main10)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(CG::HEVC444)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(CG::VVC_Main10)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(CG::MP4RA)));
  CHECK_THROWS_AS(unit.checkAndActivateVps(test::vps(static_cast<CG>(33))), test::Exception);
}

TEST_CASE("PtlChecker ISO/IEC DIS 23090-5(2E):2021 Table A-2") {
  auto unit = test::unit();

  const auto mono8 = Frame<>::lumaOnly(test::size, 8);
  const auto yuv420p8 = Frame<>::yuv420(test::size, 8);
  const auto yuv444p8 = Frame<>::yuv444(test::size, 8);
  const auto yuv420p10 = Frame<>::yuv420(test::size, 10);
  const auto yuv420p12 = Frame<>::yuv420(test::size, 12);

  const auto setup = [&](CG codecGroupIdc) -> ASPS {
    auto vps = test::vps(codecGroupIdc);
    unit.checkAndActivateVps(vps);
    const auto atlasId = vps.vps_atlas_id(0);
    auto asps = test::asps(vps, atlasId);
    unit.checkAsps(atlasId, asps);
    return asps;
  };

  SECTION("AVC Progressive High") {
    const auto asps = setup(CG::AVC_Progressive_High);

    const auto vuh = GENERATE(VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD);

    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, mono8), test::Exception);
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p8));
    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv444p8), test::Exception);
    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv420p10), test::Exception);
  }

  SECTION("HEVC Main10") {
    const auto asps = setup(CG::HEVC_Main10);

    const auto vuh = GENERATE(VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD);

    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, mono8), test::Exception);
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p8));
    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv444p8), test::Exception);
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p10));
    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv420p12), test::Exception);
  }

  SECTION("HEVC444") {
    const auto asps = setup(CG::HEVC444);

    SECTION("Occupancy or geometry video data") {
      const auto vuh = GENERATE(VUT::V3C_OVD, VUT::V3C_GVD);

      CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, mono8));
      CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p8));
      CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv444p8), test::Exception);
      CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p10));
      CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv420p12), test::Exception);
    }

    SECTION("Attribute video data") {
      CHECK_NOTHROW(unit.checkVideoFrame(VUT::V3C_AVD, asps, mono8));
      CHECK_NOTHROW(unit.checkVideoFrame(VUT::V3C_AVD, asps, yuv420p8));
      CHECK_NOTHROW(unit.checkVideoFrame(VUT::V3C_AVD, asps, yuv444p8));
      CHECK_NOTHROW(unit.checkVideoFrame(VUT::V3C_AVD, asps, yuv420p10));
      CHECK_THROWS_AS(unit.checkVideoFrame(VUT::V3C_AVD, asps, yuv420p12), test::Exception);
    }
  }

  SECTION("VVC Main 10") {
    const auto asps = setup(CG::VVC_Main10);

    const auto vuh = GENERATE(VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD);

    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, mono8));
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p8));
    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv444p8), test::Exception);
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p10));
    CHECK_THROWS_AS(unit.checkVideoFrame(vuh, asps, yuv420p12), test::Exception);
  }

  SECTION("MP4RA") {
    const auto asps = setup(CG::MP4RA);

    const auto vuh = GENERATE(VUT::V3C_OVD, VUT::V3C_GVD, VUT::V3C_AVD);

    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, mono8));
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p8));
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv444p8));
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p10));
    CHECK_NOTHROW(unit.checkVideoFrame(vuh, asps, yuv420p12));
  }
}

TEST_CASE("PtlChecker ISO/IEC DIS 23090-5(2E):2021 Table A-3") {
  auto unit = test::unit();

  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(TS::VPCC_Basic)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(TS::VPCC_Extended)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(TS::MIV_Main)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(TS::MIV_Extended)));
  CHECK_NOTHROW(unit.checkAndActivateVps(test::vps(TS::MIV_Geometry_Absent)));
  CHECK_THROWS_AS(unit.checkAndActivateVps(test::vps(static_cast<TS>(33))), test::Exception);
}

TEST_CASE("PtlChecker ISO/IEC 23090-12:2021 A.4.1") {
  auto unit = test::unit();

  const auto testFrameSizeCheck = [&](VUT vut, const ASPS &asps, int32_t width, int32_t height) {
    const auto frame1 = [](int32_t w, int32_t h) { return Frame<>::yuv420({w, h}, 8); };

    CAPTURE(vut, width, height);
    CHECK_NOTHROW(unit.checkVideoFrame(vut, asps, frame1(width, height)));
    CHECK_THROWS_AS(unit.checkVideoFrame(vut, asps, frame1(width + height, height)),
                    test::Exception);
    CHECK_THROWS_AS(unit.checkVideoFrame(vut, asps, frame1(width, width + height)),
                    test::Exception);
  };

  SECTION("No occupancy or geometry scaling") {
    const auto toolsetIdc = GENERATE(TS::MIV_Main, TS::MIV_Extended, TS::MIV_Geometry_Absent);
    CAPTURE(toolsetIdc);
    const auto vps = test::vps(toolsetIdc);
    unit.checkAndActivateVps(vps);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto atlasId = vps.vps_atlas_id(k);
      const auto asps = test::asps(vps, atlasId);
      unit.checkAsps(atlasId, asps);

      if (vps.vps_occupancy_video_present_flag(atlasId)) {
        testFrameSizeCheck(VUT::V3C_OVD, asps, 72, 24);
      }
      if (vps.vps_geometry_video_present_flag(atlasId)) {
        testFrameSizeCheck(VUT::V3C_GVD, asps, 72, 24);
      }
      if (vps.vps_attribute_video_present_flag(atlasId)) {
        testFrameSizeCheck(VUT::V3C_AVD, asps, 72, 24);
      }
    }
  }

  SECTION("Occupancy scaling") {
    auto vps = test::vps(CG::HEVC_Main10, TS::MIV_Extended, false);
    auto &vme = vps.vps_miv_extension();
    vme.vme_occupancy_scale_enabled_flag(true);
    unit.checkAndActivateVps(vps);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto atlasId = vps.vps_atlas_id(k);

      if (vps.vps_occupancy_video_present_flag(atlasId)) {
        auto asps = test::asps(vps, atlasId);
        asps.asps_miv_extension()
            .asme_occupancy_scale_enabled_flag(true)
            .asme_occupancy_scale_factor_x_minus1(0)
            .asme_occupancy_scale_factor_y_minus1(1);
        unit.checkAsps(atlasId, asps);

        testFrameSizeCheck(VUT::V3C_OVD, asps, 72, 12);
      }
    }
  }

  SECTION("Geometry scaling") {
    const auto toolsetIdc = GENERATE(TS::MIV_Main, TS::MIV_Extended);
    CAPTURE(toolsetIdc);
    auto vps = test::vps(CG::HEVC_Main10, toolsetIdc, false);
    auto &vme = vps.vps_miv_extension();
    vme.vme_geometry_scale_enabled_flag(true);
    unit.checkAndActivateVps(vps);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto atlasId = vps.vps_atlas_id(k);

      auto asps = test::asps(vps, atlasId);
      asps.asps_miv_extension()
          .asme_geometry_scale_enabled_flag(true)
          .asme_geometry_scale_factor_x_minus1(2)
          .asme_geometry_scale_factor_y_minus1(3);
      unit.checkAsps(atlasId, asps);

      testFrameSizeCheck(VUT::V3C_GVD, asps, 24, 6);
    }
  }
}

TEST_CASE("PtlChecker ISO/IEC 23090-12:2021 Table A-1") {
  auto unit = test::unit();

  const auto toolsetIdc = GENERATE(TS::VPCC_Basic, TS::VPCC_Extended, TS::MIV_Main,
                                   TS::MIV_Extended, TS::MIV_Geometry_Absent);
  const auto restrictedGeometry = GENERATE(false, true);

  if (toolsetIdc != TS::MIV_Extended && restrictedGeometry) {
    return;
  }

  CAPTURE(toolsetIdc, restrictedGeometry);

  auto vps = test::vps(toolsetIdc, restrictedGeometry);
  unit.checkAndActivateVps(vps);

  SECTION("vuh_unit_type") {
    CHECK_NOTHROW(unit.checkVuh(VUH::vps()));
    CHECK_NOTHROW(unit.checkVuh(VUH::ad(0, {})));

    CHECK_THROWS_IFF(unit.checkVuh(VUH::ovd(0, {})), toolsetIdc == TS::MIV_Main ||
                                                         toolsetIdc == TS::MIV_Geometry_Absent ||
                                                         restrictedGeometry);

    CHECK_THROWS_IFF(unit.checkVuh(VUH::gvd(0, {})),
                     toolsetIdc == TS::MIV_Geometry_Absent ||
                         (toolsetIdc == TS::MIV_Extended && restrictedGeometry));

    CHECK_NOTHROW(unit.checkVuh(VUH::avd(0, {}, 0)));

    CHECK_THROWS_IFF(unit.checkVuh(VUH::pvd(0, {})),
                     test::vpccToolset(toolsetIdc) || toolsetIdc == TS::MIV_Main);

    CHECK_THROWS_IFF(unit.checkVuh(VUH::cad(0)), test::vpccToolset(toolsetIdc));
  }

  SECTION("ptl_profile_reconstruction_idc") {
    const auto reconstructionIdc =
        GENERATE(RC::Rec_Unconstrained, RC::VPCC_Rec0, RC::VPCC_Rec1, RC::VPCC_Rec2);
    CAPTURE(reconstructionIdc);

    auto &ptl = vps.profile_tier_level();
    ptl.ptl_profile_reconstruction_idc(reconstructionIdc);

    CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                     reconstructionIdc != RC::Rec_Unconstrained && test::mivToolset(toolsetIdc));
  }

  SECTION("vpsMivExtensionPresentFlag") {
    const auto vmePresent = GENERATE(false, true);
    CAPTURE(vmePresent);

    if (vmePresent) {
      vps.vps_extension(VET::VPS_EXT_MIV).vps_miv_extension();
    } else {
      vps.removeVpsExtension(VET::VPS_EXT_MIV);
    }

    if (test::vpccToolset(toolsetIdc)) {
      CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), vmePresent);
    } else if (test::mivToolset(toolsetIdc)) {
      CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), !vmePresent);
    }
  }

  SECTION("vpsPackingInformationPresentFlag") {
    const auto pinPresent = GENERATE(false, true);
    CAPTURE(pinPresent);

    if (pinPresent) {
      vps.vps_extension(VpsExtensionType::VPS_EXT_PACKED).vps_packed_video_extension();
    }

    CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                     pinPresent && (toolsetIdc == TS::MIV_Main || test::vpccToolset(toolsetIdc)));
  }

  SECTION("vps_map_count_minus1") {
    const auto mapCountMinus1 = GENERATE(uint8_t{}, uint8_t{1});
    CAPTURE(mapCountMinus1);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);
      vps.vps_map_count_minus1(j, mapCountMinus1);

      CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                       mapCountMinus1 != 0 && test::mivToolset(toolsetIdc));

      vps.vps_map_count_minus1(j, 0);
      CHECK_NOTHROW(unit.checkAndActivateVps(vps));
    }
  }

  SECTION("vps_occupancy_video_present_flag") {
    const auto ovPresent = GENERATE(false, true);
    CAPTURE(ovPresent);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      const auto prevOvPresent = vps.vps_occupancy_video_present_flag(j);
      vps.vps_occupancy_video_present_flag(j, ovPresent);

      if (ovPresent) {
        CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), toolsetIdc == TS::MIV_Main ||
                                                            restrictedGeometry ||
                                                            toolsetIdc == TS::MIV_Geometry_Absent);
      } else {
        CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), test::vpccToolset(toolsetIdc));
      }

      vps.vps_occupancy_video_present_flag(j, prevOvPresent);
      CHECK_NOTHROW(unit.checkAndActivateVps(vps));
    }
  }

  SECTION("vps_geometry_video_present_flag") {
    const auto gvPresent = GENERATE(false, true);
    CAPTURE(gvPresent);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      const auto prevGvPresent = vps.vps_geometry_video_present_flag(j);
      vps.vps_geometry_video_present_flag(j, gvPresent);

      if (gvPresent) {
        vps.geometry_information(j, {});

        CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                         restrictedGeometry || toolsetIdc == TS::MIV_Geometry_Absent);
      } else {
        CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                         toolsetIdc == TS::MIV_Main || test::vpccToolset(toolsetIdc));
      }

      vps.vps_geometry_video_present_flag(j, prevGvPresent);
      CHECK_NOTHROW(unit.checkAndActivateVps(vps));
    }
  }

  SECTION("vme_embedded_occupancy_enabled_flag") {
    if (!vps.vpsMivExtensionPresentFlag()) {
      return;
    }
    const auto eoEnabled = GENERATE(false, true);
    CAPTURE(eoEnabled);

    auto &vme = vps.vps_miv_extension();
    vme.vme_embedded_occupancy_enabled_flag(eoEnabled);

    if (eoEnabled) {
      CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                       restrictedGeometry || toolsetIdc == TS::MIV_Geometry_Absent);
    } else {
      CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), toolsetIdc == TS::MIV_Main);
    }
  }

  SECTION("gi_geometry_MSB_align_flag") {
    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      if (vps.vps_geometry_video_present_flag(j)) {
        vps.geometry_information(j).gi_geometry_MSB_align_flag(true);
        CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), test::mivToolset(toolsetIdc));

        vps.geometry_information(j).gi_geometry_MSB_align_flag(false);
        CHECK_NOTHROW(unit.checkAndActivateVps(vps));
      }
    }
  }

  SECTION("ai_attribute_count") {
    const auto attrCount = GENERATE(0, 1, 2, 3, 10, 63);
    CAPTURE(attrCount);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      if (vps.vps_attribute_video_present_flag(j)) {
        const auto &ai = vps.attribute_information(j);
        const auto prevAttrCount = ai.ai_attribute_count();
        test::addAttributes(vps, j, static_cast<uint8_t>(attrCount));

        if (attrCount <= 1) {
          CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), restrictedGeometry);
        } else if (attrCount == 2) {
          CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                           toolsetIdc == TS::MIV_Main || toolsetIdc == TS::MIV_Geometry_Absent);
        } else if (attrCount < 63) {
          CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), test::mivToolset(toolsetIdc));
        } else {
          CHECK_THROWS(unit.checkAndActivateVps(vps));
        }

        test::addAttributes(vps, j, prevAttrCount);
        CHECK_NOTHROW(unit.checkAndActivateVps(vps));
      }
    }
  }

  SECTION("ai_attribute_type_id") {
    const auto attrTypeId =
        GENERATE(ATI::ATTR_TEXTURE, ATI::ATTR_TRANSPARENCY, ATI::ATTR_MATERIAL_ID, ATI::ATTR_NORMAL,
                 ATI::ATTR_REFLECTANCE, ATI::ATTR_UNSPECIFIED);
    CAPTURE(attrTypeId);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      if (vps.vps_attribute_video_present_flag(j)) {
        auto &ai = vps.attribute_information(j);

        for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
          const auto prevAttrTypeId = ai.ai_attribute_type_id(i);

          if ((restrictedGeometry &&
               (prevAttrTypeId == ATI::ATTR_TEXTURE && attrTypeId == ATI::ATTR_TRANSPARENCY)) ||
              (prevAttrTypeId == ATI::ATTR_TRANSPARENCY && attrTypeId == ATI::ATTR_TEXTURE)) {
            continue; // Avoid having two texture or two transparency attribute
          }

          test::setAttributeTypeId(vps, j, i, attrTypeId);

          switch (attrTypeId) {
          case ATI::ATTR_TEXTURE:
            CHECK_NOTHROW(unit.checkAndActivateVps(vps));
            break;
          case ATI::ATTR_TRANSPARENCY:
            CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                             toolsetIdc == TS::MIV_Main || toolsetIdc == TS::MIV_Geometry_Absent);
            break;
          case ATI::ATTR_UNSPECIFIED:
            CHECK_THROWS_AS(unit.checkAndActivateVps(vps), test::Exception);
            break;
          default:
            CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), test::mivToolset(toolsetIdc));
          }

          test::setAttributeTypeId(vps, j, i, prevAttrTypeId);
          CHECK_NOTHROW(unit.checkAndActivateVps(vps));
        }
      }
    }
  }

  SECTION("ai_attribute_dimension_minus1") {
    const auto attrDimensionMinus1 = GENERATE(0, 1, 2, 3, 63);
    CAPTURE(attrDimensionMinus1);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      if (vps.vps_attribute_video_present_flag(j)) {
        auto &ai = vps.attribute_information(j);

        for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
          const auto attrTypeId = ai.ai_attribute_type_id(i);
          ai.ai_attribute_dimension_minus1(i, static_cast<uint8_t>(attrDimensionMinus1));

          switch (attrDimensionMinus1) {
          case 0:
            CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                             (attrTypeId == ATI::ATTR_TEXTURE && test::mivToolset(toolsetIdc)) ||
                                 (attrTypeId == ATI::ATTR_NORMAL) || toolsetIdc == TS::VPCC_Basic);
            break;
          case 2:
            CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), attrTypeId == ATI::ATTR_TRANSPARENCY &&
                                                                toolsetIdc == TS::MIV_Extended);
            break;
          default:
            CHECK_THROWS_IFF(
                unit.checkAndActivateVps(vps),
                (test::mivToolset(toolsetIdc) &&
                 (attrTypeId == ATI::ATTR_TEXTURE || attrTypeId == ATI::ATTR_TRANSPARENCY)) ||
                    attrTypeId == ATI::ATTR_NORMAL || toolsetIdc == TS::VPCC_Basic);
          }

          test::setAttributeTypeId(vps, j, i, attrTypeId);
          CHECK_NOTHROW(unit.checkAndActivateVps(vps));
        }
      }
    }
  }

  // NOTE(#517): The ai_attribute_dimension_partitions_minus1 constraint cannot be checked
  // because there is no test model support for this syntax

  SECTION("ai_attribute_MSB_align_flag") {
    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      if (vps.vps_attribute_video_present_flag(j)) {
        auto &ai = vps.attribute_information(j);

        for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
          ai.ai_attribute_MSB_align_flag(i, true);
          CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), test::mivToolset(toolsetIdc));

          ai.ai_attribute_MSB_align_flag(i, false);
          CHECK_NOTHROW(unit.checkAndActivateVps(vps));
        }
      }
    }
  }

  SECTION("asps_..._flag") {
    const auto flagValue = GENERATE(false, true);
    CAPTURE(flagValue);

    const auto atlasId = vps.vps_atlas_id(0);

    const auto testFlag = [&](ASPS &(ASPS::*setter)(bool value), bool allowSet) {
      auto asps = test::asps(vps, atlasId);
      (asps.*setter)(flagValue);

      if (asps.asps_miv_extension_present_flag() || restrictedGeometry) {
        auto &asme = asps.asps_miv_extension();

        if (restrictedGeometry) {
          asme.asme_patch_constant_depth_flag(true);
        }
      }

      unit.checkAndActivateVps(vps);

      CHECK_THROWS_IFF(unit.checkAsps(atlasId, asps), flagValue && !allowSet);
    };

    testFlag(&ASPS::asps_long_term_ref_atlas_frames_flag, !test::mivToolset(toolsetIdc));
    testFlag(&ASPS::asps_pixel_deinterleaving_enabled_flag, !test::mivToolset(toolsetIdc));
    testFlag(&ASPS::asps_patch_precedence_order_flag, !test::mivToolset(toolsetIdc));
    testFlag(&ASPS::asps_raw_patch_enabled_flag, !test::mivToolset(toolsetIdc));
    testFlag(&ASPS::asps_eom_patch_enabled_flag,
             !test::mivToolset(toolsetIdc) && toolsetIdc != TS::VPCC_Basic);
    testFlag(&ASPS::asps_plr_enabled_flag,
             !test::mivToolset(toolsetIdc) && toolsetIdc != TS::VPCC_Basic);
    testFlag(&ASPS::asps_use_eight_orientations_flag, toolsetIdc != TS::VPCC_Basic);
    testFlag(&ASPS::asps_extended_projection_enabled_flag, toolsetIdc != TS::VPCC_Basic);
    testFlag(&ASPS::asps_miv_extension_present_flag, !test::vpccToolset(toolsetIdc));
    testFlag(&ASPS::asps_vpcc_extension_present_flag, !test::mivToolset(toolsetIdc));
  }

  SECTION("asps_max_dec_atlas_frame_buffering_minus1") {
    const auto value = GENERATE(uint8_t{}, uint8_t{33});
    CAPTURE(value);

    const auto atlasId = vps.vps_atlas_id(0);

    auto asps = test::asps(vps, atlasId);
    asps.asps_max_dec_atlas_frame_buffering_minus1(value);

    if (asps.asps_miv_extension_present_flag() || restrictedGeometry) {
      auto &asme = asps.asps_miv_extension();

      if (restrictedGeometry) {
        asme.asme_patch_constant_depth_flag(true);
      }
    }

    unit.checkAndActivateVps(vps);

    CHECK_THROWS_IFF(unit.checkAsps(atlasId, asps), value != 0 && test::mivToolset(toolsetIdc));
  }

  SECTION("asme_patch_constant_depth_flag") {
    if (!vps.vpsMivExtensionPresentFlag()) {
      return;
    }

    const auto constantDepth = GENERATE(false, true);
    const auto packedGeometry = GENERATE(false, true);
    CAPTURE(constantDepth, packedGeometry);

    if (constantDepth && packedGeometry) {
      return;
    }

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      if (packedGeometry && !vps.vps_geometry_video_present_flag(j)) {
        vps.packing_information(j, PackingInformation{}.pin_geometry_present_flag(true));
        vps.vps_packed_video_present_flag(j, true);
        unit.checkAndActivateVps(vps);
      }

      auto asps = test::asps(vps, j);
      auto &asme = asps.asps_miv_extension();

      const auto prevConstantDepth = asme.asme_patch_constant_depth_flag();
      asps.asps_miv_extension().asme_patch_constant_depth_flag(constantDepth);

      if (constantDepth) {
        CHECK_THROWS_IFF(unit.checkAsps(j, asps), toolsetIdc == TS::MIV_Main);
      } else {
        const auto pin_geometry_present_flag =
            vps.vps_packed_video_present_flag(j)
                ? vps.packing_information(j).pin_geometry_present_flag()
                : false;

        CAPTURE(vps.vps_geometry_video_present_flag(j), pin_geometry_present_flag);

        CHECK_THROWS_IFF(unit.checkAsps(j, asps), !vps.vps_geometry_video_present_flag(j) &&
                                                      !pin_geometry_present_flag &&
                                                      toolsetIdc == TS::MIV_Extended);
      }

      asme.asme_patch_constant_depth_flag(prevConstantDepth);
      CHECK_NOTHROW(unit.checkAsps(j, asps));
    }
  }

  SECTION("vps_packed_video_present_flag") {
    if (toolsetIdc != TS::MIV_Extended && toolsetIdc != TS::MIV_Geometry_Absent) {
      return;
    }

    vps.vps_extension(VpsExtensionType::VPS_EXT_PACKED).vps_packed_video_extension();

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);

      // This is not a restriction in view of the vps_packing_information_present_flag restriction
      vps.vps_packed_video_present_flag(j, true);
      CHECK_NOTHROW(unit.checkAndActivateVps(vps));
      vps.vps_packed_video_present_flag(j, false);
    }
  }

  SECTION("afps_lod_mode_enabled_flag") {
    auto afps = AtlasFrameParameterSetRBSP{};
    CHECK_NOTHROW(unit.checkAfps(afps));

    afps.afps_lod_mode_enabled_flag(true);
    CHECK_THROWS_AS(unit.checkAfps(afps), test::Exception);
  }

  SECTION("afps_raw_3d_offset_bit_count_explicit_mode_flag") {
    auto afps = AtlasFrameParameterSetRBSP{};

    afps.afps_raw_3d_offset_bit_count_explicit_mode_flag(true);
    CHECK_THROWS_AS(unit.checkAfps(afps), test::Exception);
  }

  // NOTE(#517): The afti_single_tile_in_atlas_frame_flag constraint cannot be checked because there
  // is no test model support for this syntax

  SECTION("ath_type") {
    const auto type = GENERATE(AthType::I_TILE, AthType::P_TILE, AthType::SKIP_TILE);

    const auto nuh = NUH{NUT::NAL_CRA, 0, 1};
    unit.checkNuh(nuh);

    auto atl = AtlasTileLayerRBSP{};
    atl.atlas_tile_header().ath_type(type);

    CHECK_THROWS_IFF(unit.checkAtl(nuh, atl),
                     type != AthType::I_TILE && test::mivToolset(toolsetIdc));
  }

  SECTION("atdu_patch_mode") {
    const auto nuh = NUH{NUT::NAL_CRA, 0, 1};
    unit.checkNuh(nuh);

    auto atl = AtlasTileLayerRBSP{};
    atl.atlas_tile_header().ath_type(AthType::I_TILE);

    const auto mode = GENERATE(APM::I_INTRA, APM::I_RAW, APM::I_EOM, APM::I_END);
    atl.atlas_tile_data_unit() = AtlasTileDataUnit{std::pair{APM::I_INTRA, PatchInformationData{}},
                                                   std::pair{mode, PatchInformationData{}}};
    CHECK_THROWS_IFF(unit.checkAtl(nuh, atl), mode != APM::I_INTRA && test::mivToolset(toolsetIdc));
  }
}

TEST_CASE("PtlChecker ISO/IEC 23090-12:2021 A.4.2") {
  SECTION("MIV Extended with restricted geometry has texture + transparency per atlas") {
    auto unit = test::unit();

    auto vps = test::vps(TS::MIV_Extended, true);
    unit.checkAndActivateVps(vps);

    for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      const auto j = vps.vps_atlas_id(k);
      CHECK_NOTHROW(unit.checkAndActivateVps(vps));

      auto &ai = vps.attribute_information(j);

      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        if (ai.ai_attribute_type_id(i) == ATI::ATTR_TEXTURE) {
          test::setAttributeTypeId(vps, j, i, ATI::ATTR_TRANSPARENCY);
          CHECK_THROWS_AS(unit.checkAndActivateVps(vps), test::Exception);
          test::setAttributeTypeId(vps, j, i, ATI::ATTR_TEXTURE);
        } else {
          test::setAttributeTypeId(vps, j, i, ATI::ATTR_TEXTURE);
          CHECK_THROWS_AS(unit.checkAndActivateVps(vps), test::Exception);
          test::setAttributeTypeId(vps, j, i, ATI::ATTR_TRANSPARENCY);
        }
      }
    }
  }
}

TEST_CASE("PtlChecker ISO/IEC 23090-12(2E) A.4.3") {
  SECTION("ptc_only_one_v3c_frame_flag") {
    const auto ptc_one_v3c_frame_only_flag = GENERATE(false, true);
    const auto ptl_level_idc = GENERATE(LV::Level_1_0, LV::Level_4_5, LV::Level_8_5);

    auto unit = test::unit();
    auto vps = test::vps(TS::MIV_Main);
    vps.profile_tier_level().ptl_level_idc(ptl_level_idc);
    auto frame = AccessUnit{};

    if (ptc_one_v3c_frame_only_flag) {
      vps.profile_tier_level().ptl_profile_toolset_constraints_information(
          ProfileToolsetConstraintsInformation{}.ptc_one_v3c_frame_only_flag(true));
    }

    CHECK_THROWS_IFF(unit.checkAndActivateVps(vps),
                     !ptc_one_v3c_frame_only_flag && ptl_level_idc == LV::Level_8_5);

    if (ptc_one_v3c_frame_only_flag || ptl_level_idc != LV::Level_8_5) {
      unit.checkV3cFrame(frame);
      CHECK_THROWS_IFF(unit.checkV3cFrame(frame), ptc_one_v3c_frame_only_flag);
    }
  }
}

// NOTE(MPEG/PCC/Specs/23090-5#497):What is the impact of ptl_tier_flag?
TEST_CASE("PtlChecker tier") {
  auto unit = test::unit();
  auto vps = test::vps(TS::MIV_Main, false);
  vps.profile_tier_level().ptl_tier_flag(true);
  CHECK_THROWS_AS(unit.checkAndActivateVps(vps), test::Exception);
}

TEST_CASE("PtlChecker ISO/IEC DIS 23090-5(2E):2021 A.6.1 level limits") {
  auto unit = test::unit();

  SECTION("Unknown level IDC values") {
    auto vps = test::vps(TS::MIV_Main, false);
    vps.profile_tier_level().ptl_level_idc(static_cast<LV>(33));
    CHECK_THROWS_AS(unit.checkAndActivateVps(vps), test::Exception);
  }

  const auto [level, maxAtlasSize, maxMapCount, maxAttrCount] = GENERATE(
      std::tuple{LV::Level_1_0, 2228224, 2, 1}, std::tuple{LV::Level_1_5, 2228224, 2, 3},
      std::tuple{LV::Level_2_0, 8912896, 4, 4}, std::tuple{LV::Level_2_5, 8912896, 4, 8},
      std::tuple{LV::Level_3_0, 35651584, 8, 16}, std::tuple{LV::Level_3_5, 35651584, 8, 24},
      std::tuple{LV::Level_4_0, 134217728, 16, 32}, std::tuple{LV::Level_4_5, 134217728, 16, 48},
      std::tuple{LV::Level_8_5, INT32_MAX / 2, 64, 63});
  const auto fail = GENERATE(false, true);
  CAPTURE(level, fail);

  SECTION("AspsFrameSize") {
    CAPTURE(maxAtlasSize);

    auto vps = test::vps(TS::MIV_Main, false);
    vps.profile_tier_level().ptl_level_idc(level);

    if (level == LV::Level_8_5) {
      vps.profile_tier_level().ptl_profile_toolset_constraints_information(
          ProfileToolsetConstraintsInformation{}.ptc_one_v3c_frame_only_flag(true));
    }

    const auto j = vps.vps_atlas_id(0);
    vps.vps_frame_width(j, 1);
    vps.vps_frame_height(j, fail ? maxAtlasSize + 1 : maxAtlasSize);
    CAPTURE(vps.vps_frame_width(j) * vps.vps_frame_height(j));
    unit.checkAndActivateVps(vps);

    const auto asps = test::asps(vps, j);
    CHECK_THROWS_IFF(unit.checkAsps(j, asps), fail && level != LV::Level_8_5);
  }

  SECTION("vps_map_count_minus1") {
    CAPTURE(maxMapCount);

    if (level == LV::Level_8_5 && fail) {
      return; // Syntactically impossible
    }

    auto vps = test::vps(TS::VPCC_Extended, false);
    vps.profile_tier_level().ptl_level_idc(level);

    if (level == LV::Level_8_5) {
      vps.profile_tier_level().ptl_profile_toolset_constraints_information(
          ProfileToolsetConstraintsInformation{}.ptc_one_v3c_frame_only_flag(true));
    }

    const auto j = vps.vps_atlas_id(0);
    vps.vps_map_count_minus1(j, static_cast<uint8_t>(maxMapCount - (fail ? 0 : 1)));

    CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), fail);
  }

  SECTION("ai_attribute_count") {
    CAPTURE(maxAttrCount);

    if (level == LV::Level_8_5 && fail) {
      return; // Syntactically impossible
    }

    auto vps = test::vps(TS::VPCC_Extended, false);
    vps.profile_tier_level().ptl_level_idc(level);

    if (level == LV::Level_8_5) {
      vps.profile_tier_level().ptl_profile_toolset_constraints_information(
          ProfileToolsetConstraintsInformation{}.ptc_one_v3c_frame_only_flag(true));
    }

    const auto j = vps.vps_atlas_id(0);
    test::addAttributes(vps, j, static_cast<uint8_t>(maxAttrCount + (fail ? 1 : 0)));

    CHECK_THROWS_IFF(unit.checkAndActivateVps(vps), fail);
  }

  // NOTE(#517): No need to check ai_attribute_dimension_minus1 due to MIV profile restrictions

  // NOTE(#517): No need to check afti_num_tiles_in_atlas_frame_minus1 due to MIV profile
  // restrictions

  // NOTE(MPEG/MIV/Specs/23090-12#435): NumProjPoints is 0 for embedded occupancy
}

TEST_CASE("PtlChecker ISO/IEC DIS 23090-5(2E):2021 A.6.1 FOC LSB") {
  auto unit = test::unit();

  SECTION("ath_atlas_frm_order_cnt_lsb") {
    auto vps = test::vps(TS::MIV_Main, false);
    unit.checkAndActivateVps(vps);

    SECTION("IDR coded atlas") {
      const auto nut = GENERATE(NUT::NAL_IDR_W_RADL, NUT::NAL_IDR_N_LP, NUT::NAL_GIDR_W_RADL,
                                NUT::NAL_GIDR_N_LP);
      CAPTURE(nut);

      const auto nuh = NUH{nut, 0, 1};
      unit.checkNuh(nuh);

      auto atl = AtlasTileLayerRBSP{};
      atl.atlas_tile_header().ath_type(AthType::I_TILE);
      CHECK_NOTHROW(unit.checkAtl(nuh, atl));

      atl.atlas_tile_header().ath_atlas_frm_order_cnt_lsb(1);
      CHECK_THROWS_AS(unit.checkAtl(nuh, atl), test::Exception);
    }

    SECTION("Non-IDR coded atlas") {
      const auto nut = GENERATE(NUT::NAL_CRA, NUT::NAL_SKIP_N);
      CAPTURE(nut);

      const auto nuh = NUH{nut, 0, 1};
      unit.checkNuh(nuh);

      auto atl = AtlasTileLayerRBSP{};
      atl.atlas_tile_header().ath_type(AthType::I_TILE);
      CHECK_NOTHROW(unit.checkAtl(nuh, atl));

      atl.atlas_tile_header().ath_atlas_frm_order_cnt_lsb(1);
      CHECK_NOTHROW(unit.checkAtl(nuh, atl));

      atl.atlas_tile_header().ath_atlas_frm_order_cnt_lsb(65535);
      CHECK_NOTHROW(unit.checkAtl(nuh, atl));
    }
  }

  SECTION("caf_common_atlas_frm_order_cnt_lsb") {
    auto vps = test::vps(TS::MIV_Main, false);
    unit.checkAndActivateVps(vps);

    // NOTE(MPEG/PCC/Specs/23090-5#498): Missing term `IRAP coded common atlas`
    SECTION("IRAP coded common atlas") {
      const auto nuh = NUH{NUT::NAL_CAF_IDR, 0, 1};
      unit.checkNuh(nuh);

      auto caf = CommonAtlasFrameRBSP{};
      CHECK_NOTHROW(unit.checkCaf(nuh, caf));

      caf.caf_common_atlas_frm_order_cnt_lsb(1);
      CHECK_THROWS_AS(unit.checkCaf(nuh, caf), test::Exception);
    }

    SECTION("Non-IRAP coded common atlas") {
      const auto nuh = NUH{NUT::NAL_CAF_TRIAL, 0, 1};
      unit.checkNuh(nuh);

      auto caf = CommonAtlasFrameRBSP{};
      CHECK_NOTHROW(unit.checkCaf(nuh, caf));

      caf.caf_common_atlas_frm_order_cnt_lsb(1);
      CHECK_NOTHROW(unit.checkCaf(nuh, caf));

      caf.caf_common_atlas_frm_order_cnt_lsb(65535);
      CHECK_NOTHROW(unit.checkCaf(nuh, caf));
    }
  }
}

TEST_CASE("PtlChecker ISO/IEC 23090-5(2E)/Amd.1 A.6.2") {
  auto unit = test::unit();

  SECTION("Frame rate for level checks") {
    SECTION("By default the frame rate is assumed to be 30 Hz") { CHECK(unit.frameRate() == 30.); }

    SECTION("The frame rate may be provided in the VUI") {
      const auto ratio = GENERATE(std::tuple{1, 15}, std::tuple{3, 40});

      unit.activateCasps([=]() {
        auto casps = CommonAtlasSequenceParameterSetRBSP{};
        casps.casps_miv_extension().vui_parameters(VuiParameters{}
                                                       .vui_num_units_in_tick(std::get<0>(ratio))
                                                       .vui_time_scale(std::get<1>(ratio)));
        return casps;
      }());

      CHECK(unit.frameRate() * std::get<0>(ratio) == std::get<1>(ratio));
    }
  }

  SECTION("MaxNumProjPatches") {
    const auto limit =
        GENERATE(std::tuple{LV::Level_1_5, 4'096}, std::tuple{LV::Level_4_0, 262'140});
    const auto invalid = GENERATE(false, true);
    const auto atlasCount = GENERATE(1, 5);

    auto vps = test::vps(TS::MIV_Main);
    vps.profile_tier_level().ptl_level_idc(std::get<0>(limit));
    unit.checkAndActivateVps(vps);

    auto frame = AccessUnit{};

    for (int32_t atlasIdx = 0; atlasIdx < atlasCount; ++atlasIdx) {
      frame.atlas.emplace_back().patchParamsList.resize(static_cast<size_t>(invalid) +
                                                        std::get<1>(limit));
    }

    CHECK_THROWS_IFF(unit.checkV3cFrame(frame), invalid);
  }

  SECTION("MaxNumProjPatchesPerSec") {
    const auto limit =
        GENERATE(std::tuple{LV::Level_1_0, 65'536}, std::tuple{LV::Level_3_5, 4'194'304});
    const auto invalid = GENERATE(false, true);
    const auto atlasCount = GENERATE(1, 5);

    auto vps = test::vps(TS::MIV_Main);
    vps.profile_tier_level().ptl_level_idc(std::get<0>(limit));
    unit.checkAndActivateVps(vps);

    // Avoid triggering the MaxNumProjPatches check by increasing the frame rate above 64
    static constexpr auto frameRate = 65;

    unit.activateCasps([=]() {
      auto casps = CommonAtlasSequenceParameterSetRBSP{};
      casps.casps_miv_extension().vui_parameters(
          VuiParameters{}.vui_num_units_in_tick(1).vui_time_scale(frameRate));
      return casps;
    }());

    auto frame = AccessUnit{};

    const auto numProjPatches =
        static_cast<size_t>(invalid) + std::get<1>(limit) / (frameRate * atlasCount);
    CAPTURE(std::get<0>(limit), std::get<1>(limit), invalid, atlasCount, numProjPatches);

    for (int32_t atlasIdx = 0; atlasIdx < atlasCount; ++atlasIdx) {
      frame.atlas.emplace_back().patchParamsList.resize(numProjPatches);
    }

    auto check = [&]() {
      for (int32_t frameIdx = 0; frameIdx < 2 * frameRate; ++frameIdx) {
        unit.checkV3cFrame(frame);
      }
    };
    CHECK_THROWS_IFF(check(), invalid);
  }
}
