/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/MivBitstream/MivDecoder.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/MivBitstream/VpccSampleStreamFormat.h>
#include <TMIV/MivBitstream/VpccUnit.h>

#include <istream>
#include <sstream>
#include <variant>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
MivDecoder::MivDecoder(std::istream &stream, Mode mode)
    : m_stream{stream}, m_mode{mode}, m_ssvh{sampleStreamVpccHeader(stream, mode)} {}

auto MivDecoder::decodeVpccUnit() -> bool {
  VERIFY_MIVBITSTREAM(m_stream.good());
  const auto ssvu = SampleStreamVpccUnit::decodeFrom(m_stream, m_ssvh);
  VERIFY_MIVBITSTREAM(m_stream.good());

  istringstream substream{ssvu.ssvu_vpcc_unit()};
  const auto vu = VpccUnit::decodeFrom(substream, m_vps, ssvu.ssvu_vpcc_unit_size());
  visit([this, &vu](const auto &payload) { onVpccPayload(vu.vpcc_unit_header(), payload); },
        vu.vpcc_payload().payload());

  m_stream.peek();
  return !m_stream.eof();
}

void MivDecoder::decode() {
  while (decodeVpccUnit())
    ;
}

void MivDecoder::onVpccPayload(const VpccUnitHeader & /* vuh */,
                               const std::monostate & /* payload */) {
  MIVBITSTREAM_ERROR("V-PCC payload of unknown type");
}

void MivDecoder::onVpccPayload(const VpccUnitHeader &vuh, const VpccParameterSet &vps) {
  const auto id = vps.vps_vpcc_parameter_set_id();
  while (m_vps.size() <= id) {
    m_vps.emplace_back();
    m_sequence.emplace_back();
  }
  m_vps[id] = vps;
  m_sequence[id] = Sequence{};
  m_sequence[id].atlas.resize(vps.vps_atlas_count());
}

void MivDecoder::onVpccPayload(const VpccUnitHeader &vuh, const AtlasSubBitstream &ad) {}

void MivDecoder::onVpccPayload(const VpccUnitHeader &vuh, const VideoSubBitstream &vd) {
  if (m_mode == Mode::MIV) {
    VERIFY_MIVBITSTREAM("TMIV does not yet support video sub bitstreams");
  }
}

auto MivDecoder::sampleStreamVpccHeader(istream &stream, MivDecoder::Mode mode)
    -> SampleStreamVpccHeader {
  if (mode == MivDecoder::Mode::TMC2) {
    // Skip TMC2 header
    const uint32_t PCCTMC2ContainerMagicNumber = 23021981;
    const uint32_t PCCTMC2ContainerVersion = 1;
    VERIFY_TMC2BITSTREAM(getUint32(stream) == PCCTMC2ContainerMagicNumber);
    VERIFY_TMC2BITSTREAM(getUint32(stream) == PCCTMC2ContainerVersion);
    VERIFY_TMC2BITSTREAM(getUint64(stream) == 0);
  }
  return SampleStreamVpccHeader::decodeFrom(stream);
}
} // namespace TMIV::MivBitstream
