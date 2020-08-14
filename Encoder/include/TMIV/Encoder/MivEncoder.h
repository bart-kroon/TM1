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

#ifndef _TMIV_ENCODER_MIVENCODER_H_
#define _TMIV_ENCODER_MIVENCODER_H_

#include <TMIV/MivBitstream/EncoderParams.h>
#include <TMIV/MivBitstream/NalSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>

#include <sstream>

namespace TMIV::Encoder {
using namespace MivBitstream;

class MivEncoder {
public:
  MivEncoder(std::ostream &stream);

  void writeAccessUnit(const EncoderParams &);

private:
  auto ptlMaxDecodesIdc() const -> PtlMaxDecodesIdc;
  auto commonAtlasSubBitstream() -> AtlasSubBitstream;
  auto commonAtlasFrame() const -> CommonAtlasFrameRBSP;
  auto mivViewParamsList() const -> MivViewParamsList;
  auto mivViewParamsUpdateExtrinsics() const -> MivViewParamsUpdateExtrinsics;
  auto mivViewParamsUpdateIntrinsics() const -> MivViewParamsUpdateIntrinsics;
  auto mivViewParamsUpdateDepthQuantization() const -> MivViewParamsUpdateDepthQuantization;
  auto atlasSubBitstream(std::uint8_t vai) -> AtlasSubBitstream;
  [[nodiscard]] auto atlasTileGroupLayer(std::uint8_t vai) const -> AtlasTileLayerRBSP;
  constexpr auto maxFrmOrderCntLsb() const { return 1U << (m_log2MaxFrmOrderCntLsbMinus4 + 4U); }

  template <typename Payload>
  void writeV3cUnit(VuhUnitType vut, std::uint8_t vai, Payload &&payload);
  template <typename Payload, typename... Args>
  void writeNalUnit(AtlasSubBitstream &asb, NalUnitHeader nuh, Payload &&payload, Args &&... args);

  std::ostream &m_stream;
  SampleStreamV3cHeader m_ssvh{2};
  SampleStreamNalHeader m_ssnh{2};
  EncoderParams m_params;
  bool m_irap{true};
  ViewParamsList m_viewParamsList;
  uint8_t m_log2MaxFrmOrderCntLsbMinus4{};
  uint16_t m_frmOrderCntLsb{};
};
} // namespace TMIV::Encoder

#endif
