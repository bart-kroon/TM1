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

#include <TMIV/Decoder/AtlasDecoder.h>

#include "FakeChecker.h"
#include "FakeV3cUnitSource.h"

using Catch::Contains;
using TMIV::MivBitstream::AtduPatchMode;
using TMIV::MivBitstream::AthType;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::AtlasTileDataUnit;
using TMIV::MivBitstream::AtlasTileLayerRBSP;
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;
using TMIV::MivBitstream::PatchDataUnit;
using TMIV::MivBitstream::PatchInformationData;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VuhUnitType;

namespace {
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

auto minimalV3cUnit(const V3cParameterSet &vps) {
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
} // namespace

TEST_CASE("AtlasDecoder") {
  using TMIV::Decoder::AtlasDecoder;

  auto checker = std::make_shared<test::FakeChecker>();
  const auto vps = V3cParameterSet{};
  checker->checkAndActivateVps(vps);

  const auto vuh = V3cUnitHeader::ad(0, {});
  checker->checkVuh(vuh);

  SECTION("Empty unit source") {
    auto source = test::FakeV3cUnitSource{};
    auto unit = AtlasDecoder{source, vuh, vps, -1, checker};

    REQUIRE_THROWS_WITH(unit(), Contains("No access units"));
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == vuh);
  }

  SECTION("Minimal functional example") {
    auto source = test::FakeV3cUnitSource{};

    source.units.push_back(minimalV3cUnit(vps));

    auto unit = AtlasDecoder{[&source]() { return source(); }, vuh, vps, -1, checker};

    const auto au = unit();
    REQUIRE(au.has_value());
    REQUIRE(au->foc == 0);
    REQUIRE(checker->checkVuh_callCount == 1);
    CHECK(checker->lastVuh == V3cUnitHeader::ad(0, {}));
    REQUIRE(checker->checkAndActivateNuh_callCount == 3);
    CHECK(checker->checkAndActivateAsps_callCount == 1);
    CHECK(checker->checkAfps_callCount == 1);
    CHECK(checker->checkAtl_callCount == 1);

    REQUIRE_FALSE(unit().has_value());
    REQUIRE_THROWS(unit());
    REQUIRE(checker->checkVuh_callCount == 1);
    REQUIRE(checker->checkAndActivateNuh_callCount == 3);
  }
}
