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

#include <TMIV/Decoder/MivDecoder.h>

#include "FakeChecker.h"
#include "FakeV3cUnitSource.h"

using Catch::Contains;
using TMIV::Common::Frame;
using TMIV::Common::Vec2i;
using TMIV::MivBitstream::AtduPatchMode;
using TMIV::MivBitstream::AthType;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::AtlasTileDataUnit;
using TMIV::MivBitstream::AtlasTileLayerRBSP;
using TMIV::MivBitstream::CommonAtlasFrameRBSP;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::MivViewParamsList;
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;
using TMIV::MivBitstream::PatchDataUnit;
using TMIV::MivBitstream::PatchInformationData;
using TMIV::MivBitstream::PtlProfileReconstructionIdc;
using TMIV::MivBitstream::PtlProfileToolsetIdc;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VuhUnitType;

namespace test {
auto unit(TMIV::Decoder::V3cUnitSource source, TMIV::PtlChecker::SharedChecker checker) {
  auto result = TMIV::Decoder::MivDecoder{std::move(source)};
  result.replaceChecker(std::move(checker));
  return result;
}

static constexpr auto size = Vec2i{64, 32};

auto minimalVps() {
  auto vps = V3cParameterSet{};
  vps.profile_tier_level()
      .ptl_profile_toolset_idc(PtlProfileToolsetIdc::MIV_Main)
      .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained);
  vps.vps_miv_extension() = {};
  vps.vps_frame_width({}, size.x()).vps_frame_height({}, size.y());
  return vps;
}

auto minGvdVps() {
  auto vps = minimalVps();
  vps.vps_geometry_video_present_flag({}, true).geometry_information({}) = {};

  return vps;
}

auto vpsV3cUnit(const V3cParameterSet &vps) {
  return std::make_shared<V3cUnit>(V3cUnitHeader::vps(), vps);
}

auto minimalCasps() {
  auto casps = CommonAtlasSequenceParameterSetRBSP{};
  casps.casps_miv_extension() = {};

  std::ostringstream buffer;
  casps.encodeTo(buffer);
  return std::tuple{casps, NalUnit{NalUnitHeader{NalUnitType::NAL_CASPS, 0, 1}, buffer.str()}};
}

auto minimalCafIdr(const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV) {
  auto caf = CommonAtlasFrameRBSP{};
  caf.caf_miv_extension()
      .miv_view_params_list()
      .mvp_num_views_minus1(0)
      .mvp_explicit_view_id_flag(false)
      .mvp_inpaint_flag(0, false)
      .mvp_intrinsic_params_equal_flag(true)
      .mvp_depth_quantization_params_equal_flag(true)
      .mvp_pruning_graph_params_present_flag(true)
      .camera_intrinsics(0)
      .ci_cam_type(TMIV::MivBitstream::CiCamType::equirectangular)
      .ci_projection_plane_width_minus1(1920 - 1)
      .ci_projection_plane_height_minus1(1080 - 1);

  const auto maxCommonAtlasFrmOrderCntLsb =
      1U << (caspsV.front().casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4);

  std::ostringstream buffer;
  const auto nuh = NalUnitHeader{NalUnitType::NAL_CAF_IDR, 0, 1};
  caf.encodeTo(buffer, nuh, caspsV, maxCommonAtlasFrmOrderCntLsb);
  return std::tuple{caf, NalUnit{nuh, buffer.str()}};
}

auto minimalCadV3cUnit() {
  const auto vuh = V3cUnitHeader::cad(0);
  const auto ssnh = SampleStreamNalHeader{2};
  auto asb = AtlasSubBitstream{ssnh};

  auto [casps, nuCasps] = minimalCasps();
  auto caspsV = std::vector{casps};
  asb.nal_units().push_back(nuCasps);

  auto [cafIdr, nuCafIdr] = minimalCafIdr(caspsV);
  asb.nal_units().push_back(nuCafIdr);

  return std::make_shared<V3cUnit>(vuh, asb);
}

auto minimalAsps(const V3cUnitHeader &vuh, const V3cParameterSet &vps) {
  auto asps = AtlasSequenceParameterSetRBSP{}
                  .asps_num_ref_atlas_frame_lists_in_asps(1)
                  .asps_frame_width(size.x())
                  .asps_frame_height(size.y());

  std::ostringstream buffer;
  asps.encodeTo(buffer, vuh, vps);
  return std::tuple{asps, NalUnit{NalUnitHeader{NalUnitType::NAL_ASPS, 0, 1}, buffer.str()}};
}

auto minimalAfps(const std::vector<AtlasSequenceParameterSetRBSP> &aspsV) {
  auto afps = AtlasFrameParameterSetRBSP{};

  std::ostringstream buffer;
  afps.encodeTo(buffer, aspsV);
  return std::tuple{afps, NalUnit{NalUnitHeader{NalUnitType::NAL_AFPS, 0, 1}, buffer.str()}};
}

auto minimalAtl(const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                std::vector<AtlasFrameParameterSetRBSP> &afpsV) {
  auto atl = AtlasTileLayerRBSP{};
  atl.atlas_tile_header().ath_type(AthType::I_TILE).ath_ref_atlas_frame_list_asps_flag(true);
  atl.atlas_tile_data_unit() =
      AtlasTileDataUnit{std::pair{AtduPatchMode::I_INTRA, PatchInformationData{PatchDataUnit{}}}};

  std::ostringstream buffer;
  const auto nuh = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 1};
  atl.encodeTo(buffer, nuh, aspsV, afpsV);
  return std::tuple{atl, NalUnit{nuh, buffer.str()}};
}

auto minimalAdV3cUnit(const V3cParameterSet &vps) {
  const auto vuh = V3cUnitHeader::ad(0, {});
  const auto ssnh = SampleStreamNalHeader{2};
  auto asb = AtlasSubBitstream{ssnh};

  const auto [asps, nuAsps] = minimalAsps(vuh, vps);
  const auto aspsV = std::vector{asps};
  asb.nal_units().push_back(nuAsps);

  const auto [afps, nuAfps] = minimalAfps(aspsV);
  auto afpsV = std::vector{afps};
  asb.nal_units().push_back(nuAfps);

  auto [atlIdr, nuAtlIdr] = minimalAtl(aspsV, afpsV);
  asb.nal_units().push_back(nuAtlIdr);

  return std::make_shared<V3cUnit>(vuh, asb);
}
} // namespace test

TEST_CASE("MivDecoder") {
  auto checker = std::make_shared<test::FakeChecker>();

  SECTION("Empty unit source") {
    const auto source = test::FakeV3cUnitSource{};
    auto unit = test::unit(source, checker);

    REQUIRE_THROWS_WITH(unit(), Contains("No VPS"));
    CHECK(checker->checkVuh_callCount == 0);
  }

  SECTION("The first V3C unit has to be a VPS (for this decoder)") {
    auto source = test::FakeV3cUnitSource{};

    const auto ssnh = SampleStreamNalHeader{2};
    const auto asb = AtlasSubBitstream{ssnh};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::ad(0, {}), asb));

    auto unit = test::unit(source, checker);

    REQUIRE_THROWS_WITH(unit(), Contains("No VPS"));
    CHECK(checker->checkVuh_callCount == 0);
    CHECK(checker->checkAndActivateVps_callCount == 0);
  }

  SECTION("The VPS needs to have the MIV extension enabled") {
    auto source = test::FakeV3cUnitSource{};

    const auto vps = V3cParameterSet{};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::vps(), vps));

    auto unit = test::unit(source, checker);

    REQUIRE_THROWS_WITH(unit(), Contains("vps_miv_extension_present_flag()"));
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == V3cUnitHeader::vps());
    REQUIRE(checker->checkAndActivateVps_callCount == 1);
    CHECK(checker->activeVps == vps);
  }

  SECTION("There needs to be at least one access unit following the VPS") {
    auto source = test::FakeV3cUnitSource{};

    auto vps = V3cParameterSet{};
    vps.profile_tier_level()
        .ptl_profile_toolset_idc(PtlProfileToolsetIdc::MIV_Main)
        .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained);
    vps.vps_miv_extension() = {};
    source.units.push_back(std::make_shared<V3cUnit>(V3cUnitHeader::vps(), vps));

    auto unit = test::unit(source, checker);

    REQUIRE_THROWS_WITH(unit(), Contains("access unit"));
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == V3cUnitHeader::vps());
    REQUIRE(checker->checkAndActivateVps_callCount == 1);
    CHECK(checker->activeVps == vps);
  }

  SECTION("Minimal example to decode a frame w/o any video components") {
    auto source = test::FakeV3cUnitSource{};

    const auto vps = test::minimalVps();
    source.units.push_back(test::vpsV3cUnit(vps));
    source.units.push_back(test::minimalCadV3cUnit());
    source.units.push_back(test::minimalAdV3cUnit(vps));

    auto unit = test::unit(source, checker);

    const auto frame = unit();

    REQUIRE(frame.has_value());

    CHECK(checker->checkVuh_callCount == 2);
    CHECK(checker->checkAndActivateNuh_callCount == 5);
    CHECK(checker->checkAndActivateVps_callCount == 1);
    CHECK(checker->checkAndActivateAsps_callCount == 1);
    CHECK(checker->checkAfps_callCount == 1);
    CHECK(checker->checkAtl_callCount == 1);
    CHECK(checker->checkCaf_callCount == 1);
    CHECK(checker->checkVideoFrame_callCount == 0);
  }

  SECTION("Minimal example to decode a frame with geometry video data") {
    auto source = test::FakeV3cUnitSource{};

    const auto vps = test::minGvdVps();
    source.units.push_back(test::vpsV3cUnit(vps));
    source.units.push_back(test::minimalCadV3cUnit());
    source.units.push_back(test::minimalAdV3cUnit(vps));

    auto unit = test::unit(source, checker);

    size_t setFrameServer_callCount{};

    unit.setFrameServer([&](auto... /* args */) {
      ++setFrameServer_callCount;
      return Frame<>::yuv420(test::size, 8);
    });

    const auto frame = unit();

    REQUIRE(frame.has_value());

    CHECK(checker->checkVuh_callCount == 2);
    CHECK(checker->checkAndActivateNuh_callCount == 5);
    CHECK(checker->checkAndActivateVps_callCount == 1);
    CHECK(checker->checkAndActivateAsps_callCount == 1);
    CHECK(checker->checkAfps_callCount == 1);
    CHECK(checker->checkAtl_callCount == 1);
    CHECK(checker->checkCaf_callCount == 1);
    CHECK(checker->checkVideoFrame_callCount == 1);
    CHECK(setFrameServer_callCount == 1);
  }
}
