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

#ifndef _TMIV_DECODER_MIVDECODER_H_
#define _TMIV_DECODER_MIVDECODER_H_

#include <TMIV/Common/Frame.h>
#include <TMIV/Decoder/AccessUnit.h>
#include <TMIV/Decoder/AtlasDecoder.h>
#include <TMIV/Decoder/CommonAtlasDecoder.h>
#include <TMIV/Decoder/V3cUnitBuffer.h>
#include <TMIV/VideoDecoder/VideoServer.h>

namespace TMIV::Decoder {
class MivDecoder {
public: // Decoder interface
  explicit MivDecoder(V3cUnitSource source);

  ~MivDecoder();

  // Provide a frame server for out-of-band geometry video data (GVD). GVD video sub bitstreams
  // within the bistreams take presedence.
  using GeoFrameServer = std::function<Common::Depth10Frame(
      std::uint8_t atlasId, std::uint32_t frameId, Common::Vec2i frameSize)>;
  void setGeoFrameServer(GeoFrameServer value);

  // Provide a frame server for out-of-band attribute video data (AVD). AVD video sub bitstreams
  // within the bistreams take presedence.
  //
  // NOTE 1: There is no harm in setting an attribute frame server for a bitstream that does not
  //          have any attributes, because the callback will never be invoked.
  //
  // NOTE 2: This version of the test model only supports zero or one attributes, and if there is an
  //         attribute it has to be texture. This is evident from the AttrFrameServer signature.
  using AttrFrameServer = std::function<Common::Texture444Frame(
      std::uint8_t atlasId, std::uint32_t frameId, Common::Vec2i frameSize)>;
  void setAttrFrameServer(AttrFrameServer value);

  auto operator()() -> std::optional<AccessUnit>;

private:
  [[nodiscard]] auto expectIrap() const -> bool;
  auto decodeVps() -> bool;
  void checkCapabilities();
  auto startVideoDecoder(const MivBitstream::V3cUnitHeader &vuh, double &totalTime)
      -> std::unique_ptr<VideoDecoder::VideoServer>;

  void decodeCommonAtlas();
  void decodeViewParamsList();

  void decodeAtlas(uint8_t j);
  void decodeBlockToPatchMap(uint8_t j);
  void decodePatchParamsList(uint8_t j);

  auto decodeGeoVideo(uint8_t j) -> bool;
  auto decodeAttrVideo(uint8_t j) -> bool;

  V3cUnitBuffer m_inputBuffer;
  GeoFrameServer m_geoFrameServer;
  AttrFrameServer m_attrFrameServer;

  std::unique_ptr<CommonAtlasDecoder> m_commonAtlasDecoder;
  std::vector<std::unique_ptr<AtlasDecoder>> m_atlasDecoder;
  std::vector<std::unique_ptr<VideoDecoder::VideoServer>> m_geoVideoDecoder;
  std::vector<std::unique_ptr<VideoDecoder::VideoServer>> m_attrVideoDecoder;

  std::optional<CommonAtlasDecoder::AccessUnit> m_commonAtlasAu;
  std::vector<std::optional<AtlasDecoder::AccessUnit>> m_atlasAu;
  AccessUnit m_au;

  double m_totalGeoVideoDecodingTime{};
  double m_totalAttrVideoDecodingTime{};
};
} // namespace TMIV::Decoder

#endif
