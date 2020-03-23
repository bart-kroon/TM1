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

#ifndef _TMIV_MIVBITSTREAM_MIVENCODER_H_
#define _TMIV_MIVBITSTREAM_MIVENCODER_H_

#include <TMIV/MivBitstream/IvAccessUnitParams.h>
#include <TMIV/MivBitstream/IvSequenceParams.h>
#include <TMIV/MivBitstream/NalSampleStreamFormat.h>
#include <TMIV/MivBitstream/VpccSampleStreamFormat.h>

#include <sstream>

namespace TMIV::MivBitstream {
class MivEncoder {
public:
  MivEncoder(std::ostream &stream);

  void writeIvSequenceParams(const IvSequenceParams &);
  void writeIvAccessUnitParams(const IvAccessUnitParams &, int intraPeriodFrameCount);

private:
  auto specialAtlasSubBitstream() -> AtlasSubBitstream;
  auto nonAclAtlasSubBitstream(std::uint8_t vai) -> AtlasSubBitstream;
  auto aclAtlasSubBitstream(std::uint8_t vai, int intraPeriodFrameCount) -> AtlasSubBitstream;

  auto adaptationParameterSet() const -> AdaptationParameterSetRBSP;
  auto atlasTileGroupLayer(std::uint8_t vai) const -> AtlasTileGroupLayerRBSP;
  static auto skipAtlasTileGroupLayer() -> AtlasTileGroupLayerRBSP;

  template <typename Payload>
  void writeVpccUnit(VuhUnitType vut, std::uint8_t vai, Payload &&payload);
  template <typename Payload, typename... Args>
  void writeNalUnit(AtlasSubBitstream &asb, NalUnitHeader nuh, Payload &&payload, Args &&... args);

  std::ostream &m_stream;
  SampleStreamVpccHeader m_ssvh{2};
  SampleStreamNalHeader m_ssnh{2};
  IvSequenceParams m_ivs;
  IvAccessUnitParams m_ivau;
  bool m_writeNonAcl{true};
  std::ostringstream m_nalUnitLog;
};
} // namespace TMIV::MivBitstream

#endif