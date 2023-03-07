/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/Decoder/DecodeAtlas.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/SeiRBSP.h>

#include "FakeChecker.h"

using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnitHeader;

namespace test {
namespace {
using TMIV::Common::OutputBitstream;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;

auto minimalAsps(const V3cUnitHeader &vuh, const V3cParameterSet &vps) {
  auto asps = AtlasSequenceParameterSetRBSP{}.asps_num_ref_atlas_frame_lists_in_asps(1);

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
                std::vector<AtlasFrameParameterSetRBSP> &afpsV, int32_t foc) {
  using TMIV::MivBitstream::AtduPatchMode;
  using TMIV::MivBitstream::AthType;
  using TMIV::MivBitstream::AtlasTileDataUnit;
  using TMIV::MivBitstream::AtlasTileLayerRBSP;
  using TMIV::MivBitstream::PatchDataUnit;
  using TMIV::MivBitstream::PatchInformationData;

  auto atl = AtlasTileLayerRBSP{};
  atl.atlas_tile_header()
      .ath_type(AthType::I_TILE)
      .ath_ref_atlas_frame_list_asps_flag(true)
      .ath_atlas_frm_order_cnt_lsb(static_cast<uint16_t>(foc % 16));
  atl.atlas_tile_data_unit() =
      AtlasTileDataUnit{std::pair{AtduPatchMode::I_INTRA, PatchInformationData{PatchDataUnit{}}}};

  std::ostringstream buffer;
  const auto nuh =
      NalUnitHeader{foc == 0 ? NalUnitType::NAL_IDR_N_LP : NalUnitType::NAL_TRAIL_N, 0, 1};
  atl.encodeTo(buffer, nuh, aspsV, afpsV);
  return std::tuple{atl, NalUnit{nuh, buffer.str()}};
}

template <typename Structure> auto seiPayload(const Structure &structure) {
  std::ostringstream stream;
  OutputBitstream bitstream{stream};
  structure.encodeTo(bitstream);
  return stream.str();
}

auto vuhAd() { return V3cUnitHeader::ad(0, {}); }

auto prefixSei() {
  using TMIV::MivBitstream::SeiMessage;
  using TMIV::MivBitstream::SeiRBSP;
  using PT = TMIV::MivBitstream::PayloadType;

  auto sei = SeiRBSP{};
  sei.messages().emplace_back(PT::attribute_smoothing,
                              TMIV::MivBitstream::SeiPayload{"[not implemented, to be ignored]"});

  std::ostringstream stream;
  sei.encodeTo(stream, NalUnitType::NAL_PREFIX_ESEI);
  return NalUnit{NalUnitHeader{NalUnitType::NAL_PREFIX_ESEI, 0, 1}, stream.str()};
}

auto suffixSei() {
  using TMIV::MivBitstream::SeiMessage;
  using TMIV::MivBitstream::SeiRBSP;
  using PT = TMIV::MivBitstream::PayloadType;

  auto sei = SeiRBSP{};
  sei.messages().emplace_back(PT::buffering_period,
                              TMIV::MivBitstream::SeiPayload{"[not implemented, to be ignored]"});

  std::ostringstream stream;
  sei.encodeTo(stream, NalUnitType::NAL_PREFIX_ESEI);
  return NalUnit{NalUnitHeader{NalUnitType::NAL_SUFFIX_ESEI, 0, 1}, stream.str()};
}

auto aud() { return NalUnit{NalUnitHeader{NalUnitType::NAL_AUD, 0, 1}, ""}; }

auto eos() { return NalUnit{NalUnitHeader{NalUnitType::NAL_EOS, 0, 1}, ""}; }

auto eob() { return NalUnit{NalUnitHeader{NalUnitType::NAL_EOB, 0, 1}, ""}; }
} // namespace
} // namespace test

TEST_CASE("AtlasDecoder") {
  using TMIV::Common::sourceFromIteratorPair;
  using TMIV::Decoder::decodeAtlas;

  auto checker = std::make_shared<test::FakeChecker>();
  const auto vps = V3cParameterSet{};
  checker->checkAndActivateVps(vps);

  const auto vuh = V3cUnitHeader::ad(0, {});
  checker->checkVuh(vuh);

  SECTION("Empty NAL unit source") {
    using TMIV::Common::emptySource;

    auto unit = decodeAtlas(emptySource<NalUnit>(), vuh, vps, checker);

    REQUIRE_FALSE(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
  }

  SECTION("Minimal functional example") {
    using TMIV::Common::sourceFromIteratorPair;

    const auto [asps, nuAsps] = test::minimalAsps(vuh, vps);
    const auto aspsV = std::vector{asps};
    const auto [afps, nuAfps] = test::minimalAfps(aspsV);
    auto afpsV = std::vector{afps};
    auto [atlIdr, nuAtlIdr] = test::minimalAtl(aspsV, afpsV, 0);

    const auto data = std::array{nuAsps, nuAfps, nuAtlIdr};

    auto unit = decodeAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), vuh, vps, checker);

    const auto au = unit();
    REQUIRE(au);
    REQUIRE(au->foc == 0);
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
    REQUIRE(checker->checkAndActivateNuh_callCount == 3);
    CHECK(checker->checkAndActivateAsps_callCount == 1);
    CHECK(checker->checkAfps_callCount == 1);
    CHECK(checker->checkAtl_callCount == 1);

    REQUIRE_FALSE(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    REQUIRE(checker->checkAndActivateNuh_callCount == 3);
  }

  SECTION("SEI") {
    const auto [asps, nuAsps] = test::minimalAsps(vuh, vps);
    const auto aspsV = std::vector{asps};
    const auto [afps, nuAfps] = test::minimalAfps(aspsV);
    auto afpsV = std::vector{afps};
    auto [atlIdr, nuAtlIdr] = test::minimalAtl(aspsV, afpsV, 0);

    const auto data = std::array{nuAsps, nuAfps, test::prefixSei(), nuAtlIdr, test::suffixSei()};

    auto unit = decodeAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), test::vuhAd(), vps,
                            checker);

    const auto au = unit();
    REQUIRE(au);
    CHECK(au->foc == 0);
  }

  SECTION("Frame order count, repeat CASPS") {
    const auto [asps1, nuAsps1] = test::minimalAsps(vuh, vps);
    const auto aspsV1 = std::vector{asps1};
    const auto [afps1, nuAfps1] = test::minimalAfps(aspsV1);
    auto afpsV1 = std::vector{afps1};
    const auto [cafIdr1, nuAtlIdr1] = test::minimalAtl(aspsV1, afpsV1, 0);
    const auto [atlTrial1, nuAtlTrial1] = test::minimalAtl(aspsV1, afpsV1, 1);
    const auto [atlTrial2, nuAtlTrial2] = test::minimalAtl(aspsV1, afpsV1, 5);

    const auto [asps2, nuAsps2] = test::minimalAsps(vuh, vps);
    const auto aspsV2 = std::vector{asps2};
    const auto [afps2, nuAfps2] = test::minimalAfps(aspsV2);
    auto afpsV2 = std::vector{afps2};
    const auto [atlIdr2, nuAtlIdr2] = test::minimalAtl(aspsV2, afpsV2, 0);
    const auto [atlTrial3, nuAtlTrial3] = test::minimalAtl(aspsV2, afpsV2, 4);
    const auto [atlTrial4, nuAtlTrial4] = test::minimalAtl(aspsV2, afpsV2, 18);

    // Test FOC overflow into MSB
    CHECK(atlTrial3.atlas_tile_header().ath_atlas_frm_order_cnt_lsb() >
          atlTrial4.atlas_tile_header().ath_atlas_frm_order_cnt_lsb());

    const auto data = std::array{nuAsps1, nuAfps1, nuAtlIdr1, nuAtlTrial1, nuAtlTrial2,
                                 nuAsps2, nuAfps2, nuAtlIdr2, nuAtlTrial3, nuAtlTrial4};

    auto unitAtTest = decodeAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), test::vuhAd(),
                                  vps, checker);

    const auto reference = std::array{0, 1, 5, 0, 4, 18};

    for (const auto foc : reference) {
      const auto actual = unitAtTest();
      REQUIRE(actual);
      CHECK(actual->atlV.front().atlas_tile_header().ath_atlas_frm_order_cnt_lsb() == foc % 16);
      CHECK(actual->foc == foc);
    }

    CHECK_FALSE(unitAtTest());
  }

  SECTION("Ignore AUD, EOS and EOB") {
    const auto nuAud = test::aud();
    const auto nuEos = test::eos();
    const auto nuEob = test::eob();
    const auto [asps, nuAsps] = test::minimalAsps(vuh, vps);
    const auto aspsV = std::vector{asps};
    const auto [afps, nuAfps] = test::minimalAfps(aspsV);
    auto afpsV = std::vector{afps};
    auto [atlIdr, nuAtlIdr] = test::minimalAtl(aspsV, afpsV, 0);

    const auto data = std::array{nuAud, nuAsps, nuAfps, nuAtlIdr, nuEos, nuEob};

    auto unit = decodeAtlas(sourceFromIteratorPair(data.cbegin(), data.cend()), test::vuhAd(), vps,
                            checker);

    const auto au = unit();
    REQUIRE(au);
    REQUIRE(au->foc == 0);
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
    REQUIRE(checker->checkAndActivateNuh_callCount == 6);
    CHECK(checker->checkAndActivateAsps_callCount == 1);
    CHECK(checker->checkAfps_callCount == 1);
    CHECK(checker->checkAtl_callCount == 1);

    REQUIRE_FALSE(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    REQUIRE(checker->checkAndActivateNuh_callCount == 6);
  }
}
