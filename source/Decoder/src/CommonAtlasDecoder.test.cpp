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

#include <TMIV/Decoder/CommonAtlasDecoder.h>

#include "FakeV3cUnitSource.h"

using Catch::Contains;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::CommonAtlasFrameRBSP;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VuhUnitType;

namespace {
auto minimalCasps() {
  auto casps = CommonAtlasSequenceParameterSetRBSP{};

  std::ostringstream buffer;
  casps.encodeTo(buffer);
  return std::tuple{casps, NalUnit{NalUnitHeader{NalUnitType::NAL_CASPS, 0, 1}, buffer.str()}};
}

auto minimalCafIdr(const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV) {
  auto caf = CommonAtlasFrameRBSP{};

  const auto maxCommonAtlasFrmOrderCntLsb =
      1U << (caspsV.front().casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4);

  std::ostringstream buffer;
  const auto nuh = NalUnitHeader{NalUnitType::NAL_CAF_IDR, 0, 1};
  caf.encodeTo(buffer, nuh, caspsV, maxCommonAtlasFrmOrderCntLsb);
  return std::tuple{caf, NalUnit{nuh, buffer.str()}};
}

auto minimalV3cUnit() {
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
} // namespace

TEST_CASE("CommonAtlasDecoder") {
  using TMIV::Decoder::CommonAtlasDecoder;

  SECTION("Empty unit source") {
    auto source = test::FakeV3cUnitSource{};
    auto unit = CommonAtlasDecoder{source, {}, -1};

    REQUIRE_THROWS_WITH(unit(), Contains("No access units"));
  }

  SECTION("Minimal functional example") {
    auto source = test::FakeV3cUnitSource{};
    const auto vps = V3cParameterSet{};

    source.units.push_back(minimalV3cUnit());

    auto unit = CommonAtlasDecoder{[&source]() { return source(); }, vps, -1};

    const auto au = unit();
    REQUIRE(au.has_value());
    REQUIRE(au->foc == 0);
    REQUIRE_FALSE(unit().has_value());
    REQUIRE_THROWS(unit());
  }
}
