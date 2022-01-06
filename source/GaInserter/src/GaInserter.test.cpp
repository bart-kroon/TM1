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

#include <TMIV/GaInserter/GaInserter.h>
#include <TMIV/MivBitstream/GeometryAssistance.h>
#include <TMIV/MivBitstream/ViewParamsList.h>

#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/CafMivExtension.h>
#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <fstream>
#include <sstream>

// A bitstream crafted to hit code in the parser
namespace test {
using TMIV::MivBitstream::AccessUnitDelimiterRBSP;
using TMIV::MivBitstream::AtlasAdaptationParameterSetRBSP;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::CafMivExtension;
using TMIV::MivBitstream::CommonAtlasFrameRBSP;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::MivViewParamsList;
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

auto createCasps() -> CommonAtlasSequenceParameterSetRBSP {
  auto casps = CommonAtlasSequenceParameterSetRBSP{};
  casps.casps_miv_extension_present_flag(true);
  casps.casps_miv_extension()
      .casme_vui_params_present_flag(false)
      .casme_depth_quantization_params_present_flag(false)
      .casme_depth_low_quality_flag(false);
  return casps;
}

auto createTestCaspsNalUnit() {
  auto casps = createCasps();
  return createTestNalUnit(NalUnitHeader{NalUnitType::NAL_CASPS, 0, 1}, casps);
}

auto createTestCafNalUnit(NalUnitType nut) {
  // Contains miv-view parameters.  Just one view.
  auto mvpl = MivViewParamsList{};
  mvpl.mvp_num_views_minus1(0)
      .mvp_explicit_view_id_flag(false)
      .mvp_inpaint_flag(0, false)
      .mvp_intrinsic_params_equal_flag(true)
      .mvp_depth_quantization_params_equal_flag(true)
      .mvp_pruning_graph_params_present_flag(true);
  mvpl.camera_intrinsics(0)
      .ci_cam_type(TMIV::MivBitstream::CiCamType::equirectangular)
      .ci_projection_plane_width_minus1(1920 - 1)
      .ci_projection_plane_height_minus1(1080 - 1);
  auto caf = CommonAtlasFrameRBSP{};
  auto &came = caf.caf_miv_extension();
  came.miv_view_params_list() = mvpl;
  came.came_update_extrinsics_flag(false)
      .came_update_intrinsics_flag(false)
      .came_update_depth_quantization_flag(false);

  const auto nuh = NalUnitHeader{nut, 0, 1};
  std::vector<CommonAtlasSequenceParameterSetRBSP> caspsV;
  caspsV.push_back(createCasps());
  static constexpr auto maxCommonAtlasFrmOrderCntLsb = 16U;
  return createTestNalUnit(nuh, caf, nuh, caspsV, maxCommonAtlasFrmOrderCntLsb);
}

auto createTestCommonAtlasData(const SampleStreamNalHeader &ssnh) {
  auto cad = AtlasSubBitstream{ssnh};
  cad.nal_units().push_back(createTestCaspsNalUnit());
  cad.nal_units().push_back(createTestCafNalUnit(NalUnitType::NAL_CAF_IDR));

  return cad;
}

auto createTestAtlasData(const SampleStreamNalHeader &ssnh) {
  auto cad = AtlasSubBitstream{ssnh};

  return cad;
}

auto createTestCadBitstream() {
  const auto vuh = V3cUnitHeader::cad(0);
  const auto ssnh = SampleStreamNalHeader{0};
  const auto cad = createTestCommonAtlasData(ssnh);
  return createTestV3cUnit(vuh, cad);
}

auto createTestAdBitstream() {
  const auto vuh = V3cUnitHeader::ad(0, {});
  const auto ssnh = SampleStreamNalHeader{2};
  const auto ad = createTestAtlasData(ssnh);
  return createTestV3cUnit(vuh, ad);
}

auto createTestBitstream() {
  std::ostringstream stream;

  const auto ssvh = SampleStreamV3cHeader{2};
  ssvh.encodeTo(stream);

  const auto ssvu1 = SampleStreamV3cUnit{createTestVpsBitstream()};
  ssvu1.encodeTo(stream, ssvh);

  const auto ssvu2 = SampleStreamV3cUnit{createTestCadBitstream()};
  ssvu2.encodeTo(stream, ssvh);

  const auto ssvu3 = SampleStreamV3cUnit{createTestAdBitstream()};
  ssvu3.encodeTo(stream, ssvh);

  const auto ssvu4 = SampleStreamV3cUnit{createTestAdBitstream()};
  ssvu4.encodeTo(stream, ssvh);

  return stream.str();
}
} // namespace test

TEST_CASE("GaInserter") {
  SECTION("no insertion leaves bitstream unchanged") {
    std::istringstream inStream{test::createTestBitstream()};
    std::ostringstream logStream;
    std::ostringstream recodedStream;
    std::vector<TMIV::Common::Json> seiJsons; // injecting nothing.
    auto gaInserter = TMIV::GaInserter::GaInserter{logStream, &recodedStream, seiJsons};
    gaInserter.parseV3cSampleStream(inStream);
    REQUIRE(inStream.str() == recodedStream.str());
  }
  SECTION("frames are inserted") {
    std::istringstream inStream{test::createTestBitstream()};
    std::ostringstream logStream;
    std::ostringstream recodedStream;
    std::vector<TMIV::Common::Json> seiJsons;

#include "GaInserter.test.reference.hpp"

    std::istringstream jStream0{srcJson0};
    const auto json0 = TMIV::Common::Json::loadFrom(jStream0);
    seiJsons.push_back(json0);

    std::istringstream jStream1{srcJson1};
    const auto json1 = TMIV::Common::Json::loadFrom(jStream1);
    seiJsons.push_back(json1);

    auto gaInserter = TMIV::GaInserter::GaInserter{logStream, &recodedStream, seiJsons};
    gaInserter.parseV3cSampleStream(inStream);
    std::istringstream recodedStreamToParse{recodedStream.str()};
    seiJsons.clear();
    logStream.clear();
    logStream.str("");
    recodedStream.clear();
    recodedStream.str("");
    auto gaReader = TMIV::GaInserter::GaInserter{logStream, &recodedStream, seiJsons};
    gaReader.parseV3cSampleStream(recodedStreamToParse);
    REQUIRE(recodedStream.good());
    REQUIRE(inStream.str() != recodedStream.str());
    REQUIRE(logStream.str() == insertedLog);
  }
}
