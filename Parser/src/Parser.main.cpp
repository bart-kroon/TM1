/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>
#include <TMIV/MivBitstream/FrameOrderCountRBSP.h>
#include <TMIV/MivBitstream/RecViewport.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>
#include <TMIV/MivBitstream/ViewingSpaceHandling.h>

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <variant>
#include <vector>

class Parser {
public:
  explicit Parser(std::ostream &log) : m_log{log} {}

  void parseV3cSampleStream(std::istream &stream) {
    const auto ssvh = TMIV::MivBitstream::SampleStreamV3cHeader::decodeFrom(stream);
    m_log << ssvh;

    while (stream.peek(), !stream.eof()) {
      const auto ssvu = TMIV::MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);
      m_log << ssvu;
      std::istringstream substream{ssvu.ssvu_v3c_unit()};
      parseV3cUnit(substream, ssvu.ssvu_v3c_unit_size());
    }
  }

  void parseV3cUnit(std::istream &stream, size_t numBytesInV3CUnit) {
    auto vu = TMIV::MivBitstream::V3cUnit::decodeFrom(stream, m_vpsV, numBytesInV3CUnit);
    m_log << vu.v3c_unit_header();
    m_vuh = vu.v3c_unit_header();
    std::visit([this](const auto &x) { parseV3cPayload(x); }, vu.v3c_payload().payload());
  }

  void parseV3cPayload(const std::monostate & /* unused */) {}

  void parseV3cPayload(const TMIV::MivBitstream::V3cParameterSet &vps) {
    m_log << vps;
    for (auto &x : m_vpsV) {
      if (x.vps_v3c_parameter_set_id() == vps.vps_v3c_parameter_set_id()) {
        x = vps;
        return;
      }
    }
    m_vpsV.push_back(vps);
    m_vps = vps;
  }

  void parseV3cPayload(const TMIV::MivBitstream::AtlasSubBitstream &asb) {
    m_log << asb.sample_stream_nal_header();

    for (auto &nu : asb.nal_units()) {
      parseNalUnit(nu);
    }
  }

  void parseV3cPayload(const TMIV::MivBitstream::VideoSubBitstream & /* unused */) {}

  void parseNalUnit(const TMIV::MivBitstream::NalUnit &nu) {
    m_log << nu;
    std::istringstream stream{nu.rbsp()};
    if (nu.nal_unit_header().nal_unit_type() < TMIV::MivBitstream::NalUnitType::NAL_ASPS) {
      return parseAtl(stream);
    }
    switch (nu.nal_unit_header().nal_unit_type()) {
    case TMIV::MivBitstream::NalUnitType::NAL_ASPS:
      return parseAsps(stream);
    case TMIV::MivBitstream::NalUnitType::NAL_AFPS:
      return parseAfps(stream);
    case TMIV::MivBitstream::NalUnitType::NAL_AUD:
    case TMIV::MivBitstream::NalUnitType::NAL_V3C_AUD:
      return parseAud(stream);
    case TMIV::MivBitstream::NalUnitType::NAL_EOS:
    case TMIV::MivBitstream::NalUnitType::NAL_EOB:
    case TMIV::MivBitstream::NalUnitType::NAL_FD:
      return;
    case TMIV::MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
    case TMIV::MivBitstream::NalUnitType::NAL_SUFFIX_NSEI:
    case TMIV::MivBitstream::NalUnitType::NAL_PREFIX_ESEI:
    case TMIV::MivBitstream::NalUnitType::NAL_SUFFIX_ESEI:
      return parseSei(stream);
    case TMIV::MivBitstream::NalUnitType::NAL_AAPS:
      return parseAaps(stream);
    }
  }

  void parseAtl(std::istream &stream) {
    const auto atl = TMIV::MivBitstream::AtlasTileLayerRBSP::decodeFrom(stream, *m_vuh, *m_vps,
                                                                        m_aspsV, m_afpsV);
    m_log << atl;
  }

  void parseAsps(std::istream &stream) {
    const auto asps =
        TMIV::MivBitstream::AtlasSequenceParameterSetRBSP::decodeFrom(stream, *m_vuh, *m_vps);
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
    const auto afps = TMIV::MivBitstream::AtlasFrameParameterSetRBSP::decodeFrom(stream, m_aspsV);
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
    const auto aud = TMIV::MivBitstream::AccessUnitDelimiterRBSP::decodeFrom(stream);
    m_log << aud;
  }

  void parseSei(std::istream &stream) {
    const auto sei = TMIV::MivBitstream::SeiRBSP::decodeFrom(stream);
    for (const auto &message : sei.messages()) {
      parseSeiMessage(message);
    }
  }

  void parseAaps(std::istream &stream) {
    const auto aaps =
        TMIV::MivBitstream::AtlasAdaptationParameterSetRBSP::decodeFrom(stream, *m_vps);
    m_log << aaps;
  }

  void parseSeiMessage(const TMIV::MivBitstream::SeiMessage &message) {
    m_log << message;
    std::istringstream stream{message.payload()};
    TMIV::Common::InputBitstream bitstream{stream};

    switch (message.payloadType()) {
    case TMIV::MivBitstream::PayloadType::viewing_space_handling:
      return parseViewingSpaceHandlingSei(bitstream);
    case TMIV::MivBitstream::PayloadType::rec_viewport:
      return parseRecViewportSei(bitstream);
    }
  }

  void parseViewingSpaceHandlingSei(TMIV::Common::InputBitstream &bitstream) {
    const auto vsh = TMIV::MivBitstream::ViewingSpaceHandling::decodeFrom(bitstream);
    m_log << vsh;
  }

  void parseRecViewportSei(TMIV::Common::InputBitstream &bitstream) {
    const auto rv = TMIV::MivBitstream::RecViewport::decodeFrom(bitstream);
    m_log << rv;
  }

private:
  std::ostream &m_log;
  std::vector<TMIV::MivBitstream::V3cParameterSet> m_vpsV;
  std::vector<TMIV::MivBitstream::AtlasSequenceParameterSetRBSP> m_aspsV;
  std::vector<TMIV::MivBitstream::AtlasFrameParameterSetRBSP> m_afpsV;
  std::optional<TMIV::MivBitstream::V3cUnitHeader> m_vuh;
  std::optional<TMIV::MivBitstream::V3cParameterSet> m_vps;
};

auto main(int argc, char *argv[]) -> int {
  try {
    const auto args = std::vector(argv, argv + argc);

    if (args.size() != 3 || strcmp(args[1], "-b") != 0) {
      std::clog << "Usage: Parser -b BITSTREAM" << std::endl;
      return 1;
    }

    std::ifstream stream{args[2], std::ios::binary};
    if (!stream.good()) {
      std::clog << "Failed to open bitstream for reading" << std::endl;
      return 1;
    }

    Parser parser{std::cout};
    parser.parseV3cSampleStream(stream);
    return 0;
  } catch (std::runtime_error &e) {
    std::clog << e.what() << std::endl;
    return 1;
  }
}
