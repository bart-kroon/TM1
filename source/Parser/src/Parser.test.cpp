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

#include <TMIV/Parser/Parser.h>

#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <sstream>

// A bitstream crafted to hit code in the parser
namespace test {
using TMIV::MivBitstream::AccessUnitDelimiterRBSP;
using TMIV::MivBitstream::AtlasAdaptationParameterSetRBSP;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::CommonAtlasFrameRBSP;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::SampleStreamNalUnit;
using TMIV::MivBitstream::SampleStreamV3cHeader;
using TMIV::MivBitstream::SampleStreamV3cUnit;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VuhUnitType;

template <typename Payload> auto createTestV3cUnit(const V3cUnitHeader &vuh, Payload &&payload) {
  const auto vu = V3cUnit{vuh, payload};

  std::ostringstream stream;
  vu.encodeTo(stream);
  return stream.str();
}

auto createVps() { return V3cParameterSet{}; }

auto createTestVpsBitstream() {
  const auto vuh = V3cUnitHeader::vps();
  const auto vps = createVps();
  return createTestV3cUnit(vuh, vps);
}

template <typename Rbps, typename... ParsingDependencies>
auto createTestNalUnit(const NalUnitHeader &nuh, const Rbps &rbps,
                       const ParsingDependencies &...args) {
  std::ostringstream stream;
  rbps.encodeTo(stream, args...);
  return NalUnit{nuh, stream.str()};
}

auto createTestAspsNalUnit() {
  const auto vps = createVps();
  const auto vuh = V3cUnitHeader::ad(0, {});
  const auto asps = AtlasSequenceParameterSetRBSP{};
  return createTestNalUnit(NalUnitHeader{NalUnitType::NAL_ASPS, 0, 1}, asps, vuh, vps);
}

auto createTestAfpsNalUnit() {
  const auto afps = AtlasFrameParameterSetRBSP{};
  const auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  return createTestNalUnit(NalUnitHeader{NalUnitType::NAL_AFPS, 0, 1}, afps, aspsV);
}

auto createTestAapsNalUnit() {
  const auto aaps = AtlasAdaptationParameterSetRBSP{};
  return createTestNalUnit(NalUnitHeader{NalUnitType::NAL_AAPS, 0, 1}, aaps);
}

auto createTestAudNalUnit(NalUnitType nut) {
  const auto aud = AccessUnitDelimiterRBSP{};
  return createTestNalUnit(NalUnitHeader{nut, 0, 1}, aud);
}

auto createEmptyNalUnit(NalUnitType nut) { return NalUnit{{nut, 0, 1}, {}}; }

auto createTestCaspsNalUnit() {
  const auto casps = CommonAtlasSequenceParameterSetRBSP{};
  return createTestNalUnit(NalUnitHeader{NalUnitType::NAL_CASPS, 0, 1}, casps);
}

auto createTestCafNalUnit(NalUnitType nut) {
  const auto caf = CommonAtlasFrameRBSP{};
  const auto nuh = NalUnitHeader{nut, 0, 1};
  const auto caspsV = std::vector<CommonAtlasSequenceParameterSetRBSP>(1);
  static constexpr auto maxCommonAtlasFrmOrderCntLsb = 16U;
  return createTestNalUnit(nuh, caf, nuh, caspsV, maxCommonAtlasFrmOrderCntLsb);
}

auto createTestAtlasData(const SampleStreamNalHeader &ssnh) {
  auto ad = AtlasSubBitstream{ssnh};
  ad.nal_units().push_back(createTestAspsNalUnit());
  ad.nal_units().push_back(createTestAfpsNalUnit());
  ad.nal_units().push_back(createTestAudNalUnit(NalUnitType::NAL_AUD));
  ad.nal_units().push_back(createTestAudNalUnit(NalUnitType::NAL_V3C_AUD));

  for (const auto nut : {NalUnitType::NAL_EOS, NalUnitType::NAL_EOB, NalUnitType::NAL_FD}) {
    ad.nal_units().push_back(createEmptyNalUnit(nut));
  }

  ad.nal_units().push_back(createTestAapsNalUnit());
  ad.nal_units().push_back(createTestCaspsNalUnit());
  ad.nal_units().push_back(createTestCafNalUnit(NalUnitType::NAL_CAF_TRIAL));
  ad.nal_units().push_back(createTestCafNalUnit(NalUnitType::NAL_CAF_IDR));

  return ad;
}

auto createTestAdBitstream() {
  const auto vuh = V3cUnitHeader::ad(0, {});
  const auto ssnh = SampleStreamNalHeader{0};
  const auto ad = createTestAtlasData(ssnh);
  return createTestV3cUnit(vuh, ad);
}

auto createTestBitstream() {
  std::ostringstream stream;

  const auto ssvh = SampleStreamV3cHeader{2};
  ssvh.encodeTo(stream);

  const auto ssvu1 = SampleStreamV3cUnit{createTestVpsBitstream()};
  ssvu1.encodeTo(stream, ssvh);

  const auto ssvu2 = SampleStreamV3cUnit{createTestAdBitstream()};
  ssvu2.encodeTo(stream, ssvh);

  return stream.str();
}
} // namespace test

#include "Parser.test.reference.hpp"

TEST_CASE("Parser") {
  SECTION("Construction does not perform I/O") {
    std::ostringstream outStream;
    [[maybe_unused]] const auto parser = TMIV::Parser::Parser{outStream};
    REQUIRE(outStream.good());
    REQUIRE(outStream.str().empty());
  }

  SECTION("Parsing an empty bitstream throws a runtime error") {
    std::istringstream inStream{""};
    std::ostringstream outStream;

    auto parser = TMIV::Parser::Parser{outStream};
    REQUIRE_THROWS_AS(parser.parseV3cSampleStream(inStream), std::runtime_error);
  }

  SECTION("A parsable bitstream (not decodable)") {
    std::istringstream inStream{test::createTestBitstream()};
    std::ostringstream outStream;

    auto parser = TMIV::Parser::Parser{outStream};
    parser.parseV3cSampleStream(inStream);

    const auto actualHlsLog = outStream.str();
    REQUIRE(actualHlsLog == referenceHlsLog);
  }
}
