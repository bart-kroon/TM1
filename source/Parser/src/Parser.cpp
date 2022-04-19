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

#include <TMIV/Parser/Parser.h>

#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

using namespace std::string_view_literals;

namespace TMIV::Parser {
class Parser::Impl {
public:
  explicit Impl(std::ostream &log) : m_log{log} {}

  void parseV3cSampleStream(std::istream &stream) {
    const auto ssvh = MivBitstream::SampleStreamV3cHeader::decodeFrom(stream);
    m_log << ssvh;

    while (stream.peek(), !stream.eof()) {
      const auto ssvu = MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);
      m_log << '\n' << std::string(100, '=') << '\n' << ssvu;
      std::istringstream substream{ssvu.ssvu_v3c_unit()};
      parseV3cUnit(substream, ssvu.ssvu_v3c_unit_size());
    }

    m_log << '\n' << std::string(100, '=') << '\n';
  }

private:
  void parseV3cUnit(std::istream &stream, size_t numBytesInV3CUnit) {
    auto vu = MivBitstream::V3cUnit::decodeFrom(stream, numBytesInV3CUnit);
    m_log << vu.v3c_unit_header();
    m_vuh = vu.v3c_unit_header();
    std::visit([this](const auto &x) { parseV3cUnitPayload(x); }, vu.v3c_unit_payload().payload());
  }

  void parseV3cUnitPayload(const std::monostate & /* unused */) {}

  void parseV3cUnitPayload(const MivBitstream::V3cParameterSet &vps) {
    m_log << vps;
    m_vps = vps;
  }

  void parseV3cUnitPayload(const MivBitstream::AtlasSubBitstream &asb) {
    m_log << asb.sample_stream_nal_header();

    for (const auto &nu : asb.nal_units()) {
      parseNalUnit(nu);
    }
  }

  void parseV3cUnitPayload(const MivBitstream::VideoSubBitstream & /* unused */) {}

  void parseNalUnit(const MivBitstream::NalUnit &nu) {
    m_log << '\n' << std::string(100, '-') << '\n' << nu;
    std::istringstream stream{nu.rbsp()};
    if (nu.nal_unit_header().nal_unit_type() < MivBitstream::NalUnitType::NAL_ASPS) {
      return parseAtl(stream, nu.nal_unit_header());
    }
    switch (nu.nal_unit_header().nal_unit_type()) {
    case MivBitstream::NalUnitType::NAL_ASPS:
      return parseAsps(stream);
    case MivBitstream::NalUnitType::NAL_AFPS:
      return parseAfps(stream);
    case MivBitstream::NalUnitType::NAL_AUD:
    case MivBitstream::NalUnitType::NAL_V3C_AUD:
      return parseAud(stream);
    case MivBitstream::NalUnitType::NAL_EOS:
    case MivBitstream::NalUnitType::NAL_EOB:
    case MivBitstream::NalUnitType::NAL_FD:
      return;
    case MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
    case MivBitstream::NalUnitType::NAL_SUFFIX_NSEI:
    case MivBitstream::NalUnitType::NAL_PREFIX_ESEI:
    case MivBitstream::NalUnitType::NAL_SUFFIX_ESEI:
      return parseSei(stream, nu.nal_unit_header().nal_unit_type());
    case MivBitstream::NalUnitType::NAL_AAPS:
      return parseAaps(stream);
    case MivBitstream::NalUnitType::NAL_CASPS:
      return parseCasps(stream);
    case MivBitstream::NalUnitType::NAL_CAF_TRIAL:
    case MivBitstream::NalUnitType::NAL_CAF_IDR:
      return parseCaf(stream, nu.nal_unit_header());
    default:
      std::cout << "Unknown NAL unit:\n" << nu;
    }
  }

  void parseAtl(std::istream &stream, const MivBitstream::NalUnitHeader &nuh) {
    const auto atl = MivBitstream::AtlasTileLayerRBSP::decodeFrom(stream, nuh, m_aspsV, m_afpsV);
    m_log << atl;
  }

  void parseAsps(std::istream &stream) {
    const auto asps =
        MivBitstream::AtlasSequenceParameterSetRBSP::decodeFrom(stream, *m_vuh, m_vps);
    m_log << asps;
    for (auto &x : m_aspsV) {
      if (x.asps_atlas_sequence_parameter_set_id() == asps.asps_atlas_sequence_parameter_set_id()) {
        x = asps;
        return;
      }
    }
    m_aspsV.push_back(asps);
  }

  void parseAfps(std::istream &stream) {
    const auto afps = MivBitstream::AtlasFrameParameterSetRBSP::decodeFrom(stream, m_aspsV);
    m_log << afps;
    for (auto &x : m_afpsV) {
      if (x.afps_atlas_frame_parameter_set_id() == afps.afps_atlas_frame_parameter_set_id()) {
        x = afps;
        return;
      }
    }
    m_afpsV.push_back(afps);
  }

  void parseAud(std::istream &stream) {
    const auto aud = MivBitstream::AccessUnitDelimiterRBSP::decodeFrom(stream);
    m_log << aud;
  }

  void parseSei(std::istream &stream, MivBitstream::NalUnitType nut) {
    const auto sei = MivBitstream::SeiRBSP::decodeFrom(stream, nut);
    m_log << sei;
  }

  void parseAaps(std::istream &stream) {
    const auto aaps = MivBitstream::AtlasAdaptationParameterSetRBSP::decodeFrom(stream);
    m_log << aaps;
    if (aaps.aaps_log2_max_afoc_present_flag()) {
      m_maxCommonAtlasFrmOrderCntLsb =
          1U << (aaps.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4U);
    }
  }

  void parseCasps(std::istream &stream) {
    const auto casps = MivBitstream::CommonAtlasSequenceParameterSetRBSP::decodeFrom(stream);
    m_log << casps;
    for (auto &x : m_caspsV) {
      if (x.casps_common_atlas_sequence_parameter_set_id() ==
          casps.casps_common_atlas_sequence_parameter_set_id()) {
        x = casps;
        return;
      }
    }
    m_caspsV.push_back(casps);
    m_maxCommonAtlasFrmOrderCntLsb =
        1U << (casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4U);
  }

  void parseCaf(std::istream &stream, const MivBitstream::NalUnitHeader &nuh) {
    const auto caf = MivBitstream::CommonAtlasFrameRBSP::decodeFrom(stream, nuh, m_caspsV,
                                                                    m_maxCommonAtlasFrmOrderCntLsb);
    m_caf = caf;
    m_log << caf;
  }

  std::ostream &m_log;
  std::vector<MivBitstream::CommonAtlasSequenceParameterSetRBSP> m_caspsV;
  std::vector<MivBitstream::AtlasSequenceParameterSetRBSP> m_aspsV;
  std::vector<MivBitstream::AtlasFrameParameterSetRBSP> m_afpsV;
  std::optional<MivBitstream::V3cUnitHeader> m_vuh;
  MivBitstream::V3cParameterSet m_vps;
  MivBitstream::CommonAtlasFrameRBSP m_caf;
  uint32_t m_maxCommonAtlasFrmOrderCntLsb{};
};

Parser::Parser(std::ostream &log) : m_impl{new Impl{log}} {}

Parser::~Parser() = default;

void Parser::parseV3cSampleStream(std::istream &stream) {
  return m_impl->parseV3cSampleStream(stream);
}
} // namespace TMIV::Parser
