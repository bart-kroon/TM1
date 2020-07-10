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

#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AtlasSubBitstream.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>
#include <TMIV/VideoDecoder/VideoServer.h>

#include <TMIV/Common/Frame.h>

#include <array>
#include <functional>

namespace TMIV::Decoder {
using namespace MivBitstream;

using V3cUnitSource = std::function<std::optional<V3cUnit>()>;

class V3cSampleStreamDecoder {
public:
  explicit V3cSampleStreamDecoder(std::istream &stream);

  auto operator()() -> std::optional<V3cUnit>;

private:
  std::istream &m_stream;
  SampleStreamV3cHeader m_ssvh;
};

class V3cUnitBuffer {
public:
  explicit V3cUnitBuffer(V3cUnitSource source);

  auto operator()(const V3cUnitHeader &vuh) -> std::optional<V3cUnit>;

private:
  V3cUnitSource m_source;
  std::list<V3cUnit> m_buffer;
};

class SpecialAtlasDecoder {
public:
  SpecialAtlasDecoder() = default;
  explicit SpecialAtlasDecoder(V3cUnitSource source, const V3cParameterSet &vps);

  struct AccessUnit {
    int32_t foc{};
    AtlasAdaptationParameterSetRBSP aaps;
  };

  auto operator()() -> std::optional<AccessUnit>;

private:
  V3cUnitSource m_source;
  V3cParameterSet m_vps;
};

class AtlasDecoder {
public:
  AtlasDecoder() = default;
  explicit AtlasDecoder(V3cUnitSource source, const V3cUnitHeader &vuh, const V3cParameterSet &vps);

  struct AccessUnit {
    int32_t foc{};
    AtlasSequenceParameterSetRBSP asps;
    AtlasFrameParameterSetRBSP afps;
    AtlasTileLayerRBSP atl;
    std::vector<SeiMessage> prefixNSei;
    std::vector<SeiMessage> prefixESei;
    std::vector<SeiMessage> suffixNSei;
    std::vector<SeiMessage> suffixESei;
  };

  auto operator()() -> std::optional<AccessUnit>;

private:
  auto decodeAsb() -> bool;
  auto decodeAu() -> AccessUnit;
  void decodePrefixNalUnit(AccessUnit &au, const NalUnit &nu);
  void decodeAclNalUnit(AccessUnit &au, const NalUnit &nu);
  void decodeSuffixNalUnit(AccessUnit &au, const NalUnit &nu);
  void decodeAsps(std::istream &stream);
  void decodeAfps(std::istream &stream);
  void decodeAaps(std::istream &stream);
  void decodeSei(std::vector<SeiMessage> &messages, std::istream &stream);

  V3cUnitSource m_source;
  V3cUnitHeader m_vuh{VuhUnitType::V3C_AD};
  V3cParameterSet m_vps;

  std::list<NalUnit> m_buffer;
  int32_t m_foc{-1};

  std::vector<AtlasSequenceParameterSetRBSP> m_aspsV;
  std::vector<AtlasFrameParameterSetRBSP> m_afpsV;
  std::vector<AtlasAdaptationParameterSetRBSP> m_aapsV;
};

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
  auto expectIrap() const -> bool;
  auto decodeVps() -> bool;
  void checkCapabilities();
  auto startVideoDecoder(const V3cUnitHeader &vuh, double &totalTime)
      -> std::unique_ptr<VideoDecoder::VideoServer>;

  void decodeSpecialAtlas();
  void decodeViewParamsList();

  void decodeAtlas(uint8_t j);
  void decodeBlockToPatchMap(uint8_t j);
  void decodePatchParamsList(uint8_t j);

  auto decodeGeoVideo(uint8_t j) -> bool;
  auto decodeAttrVideo(uint8_t j) -> bool;

  V3cUnitBuffer m_inputBuffer;
  GeoFrameServer m_geoFrameServer;
  AttrFrameServer m_attrFrameServer;

  std::unique_ptr<SpecialAtlasDecoder> m_specialAtlasDecoder;
  std::vector<std::unique_ptr<AtlasDecoder>> m_atlasDecoder;
  std::vector<std::unique_ptr<VideoDecoder::VideoServer>> m_geoVideoDecoder;
  std::vector<std::unique_ptr<VideoDecoder::VideoServer>> m_attrVideoDecoder;

  V3cParameterSet m_vps;
  std::optional<SpecialAtlasDecoder::AccessUnit> m_specialAtlasAu;
  std::vector<std::optional<AtlasDecoder::AccessUnit>> m_atlasAu;
  AccessUnit m_au;

  double m_totalGeoVideoDecodingTime{};
  double m_totalAttrVideoDecodingTime{};
};
} // namespace TMIV::Decoder

#endif
