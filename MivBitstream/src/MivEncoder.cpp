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

#include <TMIV/MivBitstream/MivEncoder.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {

MivEncoder::MivEncoder(std::ostream &stream) : m_stream{stream} {
  cout << m_ssvh;
  m_ssvh.encodeTo(m_stream);
}

void MivEncoder::writeIvSequenceParams(const IvSequenceParams &ivSequenceParams) {
  m_vps = {ivSequenceParams.vps};
  writeVpccUnit(VuhUnitType::VPCC_VPS, 0, ivSequenceParams.vps);
  writeVpccUnit(VuhUnitType::VPCC_AD, specialAtlasId, specialAtlasSubBitstream(ivSequenceParams));
}

void MivEncoder::writeIvAccessUnitParams(const IvAccessUnitParams &ivAccessUnitParams) {
  // TODO....

  m_writeNonAcl = false;
}

template <typename Payload>
void MivEncoder::writeVpccUnit(VuhUnitType vuh_unit_type, uint8_t vuh_atlas_id, Payload &&payload) {
  auto vuh = VpccUnitHeader{vuh_unit_type};
  vuh.vuh_atlas_id(vuh_atlas_id);

  const auto vu = VpccUnit{vuh, forward<Payload>(payload)};

  ostringstream substream;
  vu.encodeTo(substream, m_vps);

  const auto ssvu = SampleStreamVpccUnit{substream.str()};
  ssvu.encodeTo(m_stream, m_ssvh);
  cout << ssvu << vu;
}

template <typename Payload, typename... Args>
void MivEncoder::writeNalUnit(AtlasSubBitstream &asb, NalUnitHeader nuh, Payload &&payload,
                              Args &&... args) const {
  ostringstream substream1;
  payload.encodeTo(substream1, forward<Args>(args)...);

  const auto nu = NalUnit{nuh, substream1.str()};

  ostringstream substream2;
  nu.encodeTo(substream2);

  const auto ssnu = SampleStreamNalUnit{substream2.str()};
  ssnu.encodeTo(m_stream, m_ssnh);
  cout << ssnu << nu;
}

auto MivEncoder::specialAtlasSubBitstream(const IvSequenceParams &ivSequenceParams) const
    -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{m_ssnh};
  writeNalUnit(asb, NalUnitHeader{NalUnitType::NAL_APS, 0, 1},
               adaptationParameterSet(ivSequenceParams));
  return asb;
}

auto MivEncoder::adaptationParameterSet(const IvSequenceParams &ivSequenceParams) const
    -> AdaptationParameterSetRBSP {
  // TODO: Convert viewParamsList to APS
  return {};
}
} // namespace TMIV::MivBitstream
