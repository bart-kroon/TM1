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

#ifndef TMIV_DECODER_MIVDECODER_H
#define TMIV_DECODER_MIVDECODER_H

#include <TMIV/Common/FlatMap.h>
#include <TMIV/Common/Frame.h>
#include <TMIV/Decoder/AtlasDecoder.h>
#include <TMIV/Decoder/CommonAtlasDecoder.h>
#include <TMIV/Decoder/V3cUnitBuffer.h>
#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/VideoDecoder/VideoDecoderFactory.h>

namespace TMIV::Decoder {
class MivDecoder {
public:
  explicit MivDecoder(V3cUnitSource source);

  MivDecoder(const MivDecoder &other) = delete;
  MivDecoder(MivDecoder &&other) = default;
  auto operator=(const MivDecoder &other) -> MivDecoder & = delete;
  auto operator=(MivDecoder &&other) -> MivDecoder & = default;
  ~MivDecoder();

  using V3cUnitHeader = MivBitstream::V3cUnitHeader;

  // Call signature for the frame server.
  //
  // The V3C unit header and frame index are provided as a way for the frame server to index the
  // requested video frame. The VPS and ASPS are provided for the frame server to determine a
  // suitable format. There is no requirement on the return value to be in nominal format. Frames
  // are required to have sufficient planes.
  using FrameServer = std::function<Common::Frame<>(
      V3cUnitHeader vuh, int32_t frameIdx, const MivBitstream::V3cParameterSet &vps,
      const MivBitstream::AtlasSequenceParameterSetRBSP &asps)>;

  // Provide a frame server for out-of-band video data (if any)
  void setFrameServer(FrameServer value);

  // Pull an access unit from the MIV decoder
  auto operator()() -> std::optional<MivBitstream::AccessUnit>;

private:
  auto decodeVps() -> std::optional<MivBitstream::V3cParameterSet>;

  void resetDecoder();
  void checkCapabilities() const;

  [[nodiscard]] auto decoderId(V3cUnitHeader vuh) const noexcept -> VideoDecoder::DecoderId;

  // Start a video decoder for the video sub bitstream corresponding to the specified V3C unit
  // header. If and only if there are no such units in the V3C unit stream, the out-of-band video
  // frame server will be used instead.
  auto tryStartVideoDecoder(V3cUnitHeader vuh) -> bool;

  // Decode a video frame of the specified video sub-bitstream using the already initalized video
  // decoder. Alternatively, pull out-of-band video from the frame server.
  auto decodeVideoFrame(V3cUnitHeader vuh) -> bool;

  // Pull an out-of-band video frame from the frame server.
  auto pullOutOfBandVideoFrame(V3cUnitHeader vuh) -> bool;

  void decodeCommonAtlas();
  void decodeViewParamsList();
  auto decodeVideoSubBitstreams() -> bool;

  void decodeMvpl(const MivBitstream::MivViewParamsList &mvpl, bool dqParamsPresentFlag);
  void decodeMvpue(const MivBitstream::MivViewParamsUpdateExtrinsics &mvpue);
  void decodeMvpui(const MivBitstream::MivViewParamsUpdateIntrinsics &mvpui);
  void decodeMvpudq(const MivBitstream::MivViewParamsUpdateDepthQuantization &mvpudq);

  void decodeAtlas(size_t k);
  auto decodePatchParamsList(size_t k, MivBitstream::PatchParamsList &ppl) const
      -> const MivBitstream::PatchParamsList &;
  [[nodiscard]] auto decodeBlockToPatchMap(size_t k, const MivBitstream::PatchParamsList &ppl) const
      -> Common::Frame<Common::PatchIdx>;

  V3cUnitBuffer m_inputBuffer;
  FrameServer m_frameServer;

  std::unique_ptr<CommonAtlasDecoder> m_commonAtlasDecoder;
  std::vector<std::unique_ptr<AtlasDecoder>> m_atlasDecoder;

  Common::FlatMap<V3cUnitHeader, std::unique_ptr<VideoDecoder::VideoDecoderBase>> m_videoDecoders;
  Common::FlatMap<V3cUnitHeader, double> m_totalVideoDecodingTime;

  std::optional<CommonAtlasDecoder::AccessUnit> m_commonAtlasAu;
  std::vector<std::optional<AtlasDecoder::AccessUnit>> m_atlasAu;
  MivBitstream::AccessUnit m_au;

  enum class State { initial, decoding, eof };
  State m_state{State::initial};
};
} // namespace TMIV::Decoder

#endif
