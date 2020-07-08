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

#include <TMIV/MivBitstream/BitrateReport.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <variant>
#include <vector>

class PartialParser {
public:
  void parseV3cSampleStream(std::istream &stream) {
    const auto ssvh = TMIV::MivBitstream::SampleStreamV3cHeader::decodeFrom(stream);

    while (stream.peek(), !stream.eof()) {
      const auto ssvu = TMIV::MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);
      std::istringstream substream{ssvu.ssvu_v3c_unit()};
      parseV3cUnit(substream, ssvu.ssvu_v3c_unit_size());
    }
  }

  void parseV3cUnit(std::istream &stream, size_t numBytesInV3CUnit) {
    auto vu = TMIV::MivBitstream::V3cUnit::decodeFrom(stream, m_vpsV, numBytesInV3CUnit);
    m_report.add(vu.v3c_unit_header(), numBytesInV3CUnit);
    std::visit([this](const auto &x) { parseV3cPayload(x); }, vu.v3c_payload().payload());
  }

  void parseV3cPayload(const std::monostate & /* unused */) {}

  void parseV3cPayload(const TMIV::MivBitstream::V3cParameterSet &vps) {
    for (auto &x : m_vpsV) {
      if (x.vps_v3c_parameter_set_id() == vps.vps_v3c_parameter_set_id()) {
        x = vps;
        return;
      }
    }
    m_vpsV.push_back(vps);
  }

  void parseV3cPayload(const TMIV::MivBitstream::AtlasSubBitstream &asb) {
    for (auto &nu : asb.nal_units()) {
      m_report.add(nu.nal_unit_header(), nu.size());
    }
  }

  void parseV3cPayload(const TMIV::MivBitstream::VideoSubBitstream & /* unused */) {}

  auto report() const -> auto & { return m_report; }

private:
  std::vector<TMIV::MivBitstream::V3cParameterSet> m_vpsV;
  TMIV::MivBitstream::BitrateReport m_report;
};

auto main(int argc, char *argv[]) -> int {
  try {
    const auto args = std::vector(argv, argv + argc);

    if (args.size() != 3 || strcmp(args[1], "-b") != 0) {
      std::clog << "Usage: BitrateReport -b BITSTREAM" << std::endl;
      return 1;
    }

    std::ifstream stream{args[2], std::ios::binary};
    if (!stream.good()) {
      std::clog << "Failed to open bitstream for reading" << std::endl;
      return 1;
    }

    PartialParser parser;
    parser.parseV3cSampleStream(stream);
    parser.report().printTo(std::cout);
    return 0;
  } catch (std::runtime_error &e) {
    std::clog << e.what() << std::endl;
    return 1;
  }
}
