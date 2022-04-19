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

#include <TMIV/Decoder/DecodeCommonAtlas.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/SeiRBSP.h>

#include "FakeChecker.h"

using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnitHeader;

namespace test {
namespace {
using TMIV::Common::OutputBitstream;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;

auto minimalCasps() {
  auto casps = CommonAtlasSequenceParameterSetRBSP{};

  std::ostringstream buffer;
  casps.encodeTo(buffer);
  return std::tuple{casps, NalUnit{NalUnitHeader{NalUnitType::NAL_CASPS, 0, 1}, buffer.str()}};
}

auto minimalCafIdr(const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV) {
  using TMIV::MivBitstream::CommonAtlasFrameRBSP;

  auto caf = CommonAtlasFrameRBSP{};

  const auto maxCommonAtlasFrmOrderCntLsb =
      1U << (caspsV.front().casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4);

  std::ostringstream buffer;
  const auto nuh = NalUnitHeader{NalUnitType::NAL_CAF_IDR, 0, 1};
  caf.encodeTo(buffer, nuh, caspsV, maxCommonAtlasFrmOrderCntLsb);
  return std::tuple{caf, NalUnit{nuh, buffer.str()}};
}

auto minimalCafTrial(int32_t foc, const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV) {
  using TMIV::MivBitstream::CommonAtlasFrameRBSP;

  auto caf = CommonAtlasFrameRBSP{};

  const auto maxCommonAtlasFrmOrderCntLsb =
      1U << (caspsV.front().casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4);

  caf.caf_common_atlas_frm_order_cnt_lsb(static_cast<uint16_t>(foc % maxCommonAtlasFrmOrderCntLsb));

  std::ostringstream buffer;
  const auto nuh = NalUnitHeader{NalUnitType::NAL_CAF_TRIAL, 0, 1};
  caf.encodeTo(buffer, nuh, caspsV, maxCommonAtlasFrmOrderCntLsb);
  return std::tuple{caf, NalUnit{nuh, buffer.str()}};
}

auto caspsWithVui() {
  using TMIV::MivBitstream::VuiParameters;

  auto casps = CommonAtlasSequenceParameterSetRBSP{};
  casps.casps_miv_extension().vui_parameters(VuiParameters{}.vui_unit_in_metres_flag(true));

  std::ostringstream buffer;
  casps.encodeTo(buffer);
  return std::tuple{casps, NalUnit{NalUnitHeader{NalUnitType::NAL_CASPS, 0, 1}, buffer.str()}};
}

auto fillerData() { return NalUnit{NalUnitHeader{NalUnitType::NAL_FD, 0, 1}, "filler"}; }

auto arrayOfNalUnitsWithVuiAndFd() {
  auto [casps, nuCasps] = caspsWithVui();
  auto caspsV = std::vector{casps};

  auto [cafIdr, nuCafIdr] = minimalCafIdr(caspsV);

  return std::array{nuCasps, nuCafIdr, fillerData()};
}

const auto exampleGup = []() {
  using TMIV::MivBitstream::GeometryUpscalingParameters;
  using TMIV::Common::Half;

  return GeometryUpscalingParameters{}
      .gup_delta_threshold(3)
      .gup_erode_threshold(Half{0.4F})
      .gup_max_curvature(7);
}();

const auto exampleVs = []() {
  using TMIV::MivBitstream::ViewingSpace;
  using TMIV::MivBitstream::ElementaryShapeOperation;
  using TMIV::MivBitstream::ElementaryShape;
  using TMIV::MivBitstream::PrimitiveShape;
  using TMIV::MivBitstream::Cuboid;

  return ViewingSpace{{{ElementaryShapeOperation::add,
                        ElementaryShape{{PrimitiveShape{Cuboid{{}, {}}, {}, {}, {}}}, {}}}}};
}();

const auto exampleVcp = []() {
  using TMIV::MivBitstream::ViewportCameraParameters;

  auto vcp = ViewportCameraParameters{};
  vcp.vcp_cancel_flag = true;

  return vcp;
}();

const auto exampleVp = []() {
  using TMIV::MivBitstream::ViewportPosition;

  return ViewportPosition{10, false, {}, true};
}();

const auto exampleAve = []() {
  using TMIV::MivBitstream::AtlasViewEnabled;

  auto ave = AtlasViewEnabled{};
  ave.ave_cancel_flag(true);
  return ave;
}();

auto prefixSei() {
  using TMIV::MivBitstream::SeiMessage;
  using TMIV::MivBitstream::SeiPayload;
  using TMIV::MivBitstream::SeiRBSP;
  using PT = TMIV::MivBitstream::PayloadType;

  auto sei = SeiRBSP{};
  sei.messages().emplace_back(PT::geometry_upscaling_parameters, SeiPayload{exampleGup});
  sei.messages().emplace_back(PT::viewing_space, SeiPayload{exampleVs});
  sei.messages().emplace_back(PT::viewport_camera_parameters, SeiPayload{exampleVcp});
  sei.messages().emplace_back(PT::filler_payload, SeiPayload{" ~ noise ~ "});
  sei.messages().emplace_back(PT::viewport_position, SeiPayload{exampleVp});
  sei.messages().emplace_back(PT::atlas_view_enabled, SeiPayload{exampleAve});

  std::ostringstream stream;
  sei.encodeTo(stream, NalUnitType::NAL_PREFIX_ESEI);
  return NalUnit{NalUnitHeader{NalUnitType::NAL_PREFIX_ESEI, 0, 1}, stream.str()};
}

auto suffixSei() {
  using TMIV::MivBitstream::SeiMessage;
  using TMIV::MivBitstream::SeiPayload;
  using TMIV::MivBitstream::SeiRBSP;
  using PT = TMIV::MivBitstream::PayloadType;

  auto sei = SeiRBSP{};
  sei.messages().emplace_back(PT::filler_payload, SeiPayload{" ~ noise ~ "});

  std::ostringstream stream;
  sei.encodeTo(stream, NalUnitType::NAL_SUFFIX_ESEI);
  return NalUnit{NalUnitHeader{NalUnitType::NAL_SUFFIX_ESEI, 0, 1}, stream.str()};
}

auto aud() { return NalUnit{NalUnitHeader{NalUnitType::NAL_AUD, 0, 1}, ""}; }

auto eos() { return NalUnit{NalUnitHeader{NalUnitType::NAL_EOS, 0, 1}, ""}; }

auto eob() { return NalUnit{NalUnitHeader{NalUnitType::NAL_EOB, 0, 1}, ""}; }
} // namespace
} // namespace test

TEST_CASE("TMIV::Decoder::decodeCommonAtlas") {
  using TMIV::Common::sourceFromIteratorPair;
  using TMIV::Decoder::decodeCommonAtlas;

  auto checker = std::make_shared<test::FakeChecker>();
  const auto vps = V3cParameterSet{};
  checker->checkAndActivateVps(vps);

  const auto vuh = V3cUnitHeader::cad(0);
  checker->checkVuh(vuh);

  SECTION("Empty NAL unit source") {
    using TMIV::Common::emptySource;

    auto unit = decodeCommonAtlas(emptySource<NalUnit>(), vps, checker);

    REQUIRE_FALSE(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
  }

  SECTION("Minimal functional example") {
    auto [casps, nuCasps] = test::minimalCasps();
    auto caspsV = std::vector{casps};
    auto [cafIdr, nuCafIdr] = test::minimalCafIdr(caspsV);

    const auto data = std::array{nuCasps, nuCafIdr};

    auto unit = decodeCommonAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), vps, checker);

    const auto au = unit();
    REQUIRE(au);
    REQUIRE(au->foc == 0);
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
    REQUIRE(checker->checkAndActivateNuh_callCount == 2);
    CHECK(checker->checkCaf_callCount == 1);

    REQUIRE_FALSE(unit());
    REQUIRE_THROWS(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    REQUIRE(checker->checkAndActivateNuh_callCount == 2);
  }

  SECTION("VUI") {
    const auto data = test::arrayOfNalUnitsWithVuiAndFd();
    auto unit = decodeCommonAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), vps, checker);

    const auto au = unit();
    REQUIRE(au);
    REQUIRE(au->foc == 0);
    REQUIRE(au->casps.casps_miv_extension_present_flag());
    REQUIRE(au->casps.casps_miv_extension().casme_vui_params_present_flag());
    CHECK(au->casps.casps_miv_extension().vui_parameters().vui_unit_in_metres_flag());
  }

  SECTION("SEI") {
    auto [casps, nuCasps] = test::caspsWithVui();
    auto caspsV = std::vector{casps};
    auto [cafIdr, nuCafIdr] = test::minimalCafIdr(caspsV);

    const auto data = std::array{nuCasps, test::prefixSei(), nuCafIdr, test::suffixSei()};

    auto unit = decodeCommonAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), vps, checker);

    const auto au = unit();
    REQUIRE(au);
    REQUIRE(au->vs);
    REQUIRE(au->vcp);
    REQUIRE(au->vp);
    REQUIRE(au->ave);

    CHECK(au->foc == 0);
    CHECK(au->gup == test::exampleGup);
    CHECK(*au->vs == test::exampleVs);
    CHECK(*au->vcp == test::exampleVcp);
    CHECK(*au->vp == test::exampleVp);
    CHECK(*au->ave == test::exampleAve);
  }

  SECTION("Frame order count, repeat CASPS") {
    const auto [casps1, nuCasps1] = test::minimalCasps();
    const auto caspsV1 = std::vector{casps1};
    const auto [cafIdr1, nuCafIdr1] = test::minimalCafIdr(caspsV1);
    const auto [cafTrial1, nuCafTrial1] = test::minimalCafTrial(1, caspsV1);
    const auto [cafTrial2, nuCafTrial2] = test::minimalCafTrial(5, caspsV1);
    const auto [casps2, nuCasps2] = test::minimalCasps();
    const auto caspsV2 = std::vector{casps2};
    const auto [cafIdr2, nuCafIdr2] = test::minimalCafIdr(caspsV2);
    const auto [cafTrial3, nuCafTrial3] = test::minimalCafTrial(4, caspsV2);
    const auto [cafTrial4, nuCafTrial4] = test::minimalCafTrial(18, caspsV2);

    // Test FOC overflow into MSB
    CHECK(cafTrial3.caf_common_atlas_frm_order_cnt_lsb() >
          cafTrial4.caf_common_atlas_frm_order_cnt_lsb());

    const auto data = std::array{nuCasps1, nuCafIdr1, nuCafTrial1, nuCafTrial2,
                                 nuCasps2, nuCafIdr2, nuCafTrial3, nuCafTrial4};

    auto unitAtTest =
        decodeCommonAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), vps, checker);

    const auto reference = std::array{0, 1, 5, 0, 4, 18};

    for (const auto foc : reference) {
      const auto actual = unitAtTest();
      REQUIRE(actual);
      CHECK(actual->caf.caf_common_atlas_frm_order_cnt_lsb() == foc % 16);
      CHECK(actual->foc == foc);
    }

    CHECK_FALSE(unitAtTest());
  }

  SECTION("Ignore AUD, EOS and EOB") {
    auto nuAud = test::aud();
    auto nuEos = test::eos();
    auto nuEob = test::eob();
    auto [casps, nuCasps] = test::minimalCasps();
    auto caspsV = std::vector{casps};
    auto [cafIdr, nuCafIdr] = test::minimalCafIdr(caspsV);

    const auto data = std::array{nuAud, nuCasps, nuCafIdr, nuEos, nuEob};

    auto unit = decodeCommonAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), vps, checker);

    const auto au = unit();
    REQUIRE(au);
    REQUIRE(au->foc == 0);
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
    REQUIRE(checker->checkAndActivateNuh_callCount == 5);
    CHECK(checker->checkCaf_callCount == 1);

    REQUIRE_FALSE(unit());
    REQUIRE_THROWS(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    REQUIRE(checker->checkAndActivateNuh_callCount == 5);
  }
}
