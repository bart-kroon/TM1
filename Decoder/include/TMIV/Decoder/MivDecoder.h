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
#include <TMIV/MivBitstream/BitrateReport.h>
#include <TMIV/MivBitstream/FrameOrderCountRBSP.h>
#include <TMIV/MivBitstream/RecViewport.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>
#include <TMIV/MivBitstream/ViewingSpaceHandling.h>
#include <TMIV/VideoDecoder/VideoServer.h>

#include <TMIV/Common/Frame.h>

#include <array>
#include <functional>

namespace TMIV::Decoder {
using namespace MivBitstream;

class MivDecoder {
public: // Decoder interface
  // Construct a MivDecoder and read the sample stream V3C header
  explicit MivDecoder(std::istream &stream);

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

  // Decode V3C sample bitstream
  //
  // Register listeners to obtain output. The decoding is stopped prematurely when any of the
  // listeners returns false.
  void decode();

  // Optional bitrate reporting
  void enableBitrateReporting();
  void printBitrateReport(std::ostream &stream) const;

public: // Callback signatures
  // Callback that will be called when a VPS is decoded.
  using SequenceListener = std::function<bool(const V3cParameterSet &)>;

  // Callback that will be called when an access unit (frame) is decoded.
  using FrameListener = std::function<bool(const AccessUnit &)>;

public: // Callback registrations
  std::vector<SequenceListener> onSequence;
  std::vector<FrameListener> onFrame;

private: // Decoder output
  void outputSequence(const V3cParameterSet &vps);
  void outputFrame(const V3cUnitHeader &vuh);
  void outputAtlasData(AccessUnit &au);
  [[nodiscard]] auto haveFrame(const V3cUnitHeader &vuh) const -> bool;

private: // Video deecoding processes
  auto decodeVideoSubBitstreams(const V3cParameterSet &vps) -> bool;
  void startGeoVideoDecoders(const V3cParameterSet &vps);
  void startAttrVideoDecoders(const V3cParameterSet &vps);
  void outputGeoVideoData(AccessUnit &au);
  void outputAttrVideoData(AccessUnit &au);

private: // Decoding processes
  void decodeV3cPayload(const V3cUnitHeader &vuh, const V3cPayload::Payload &payload);
  void decodeVps(const V3cUnitHeader &vuh, const V3cParameterSet &vps);
  void decodeAsb(const V3cUnitHeader &vuh, const AtlasSubBitstream &asb,
                 const std::uint8_t atlasCountMinus1);

  void decodeNalUnit(const V3cUnitHeader &vuh, const NalUnit &nu,
                     const std::uint8_t atlasCountMinus1);
  static void decodeUnknownNalUnit(const V3cUnitHeader &vuh, const NalUnit &nu);

  void decodeAtl(const V3cUnitHeader &vuh, const NalUnitHeader &nuh, const AtlasTileLayerRBSP &atl);
  static auto decodeMvpl(const MivViewParamsList &mvpl) -> ViewParamsList;
  static auto decodeAtdu(const AtlasTileDataUnit &atdu, const AtlasTileHeader &ath,
                         const AtlasSequenceParameterSetRBSP &asps) -> PatchParamsList;
  static auto decodeBlockToPatchMap(const AtlasTileDataUnit &atdu,
                                    const AtlasSequenceParameterSetRBSP &asps)
      -> Common::BlockToPatchMap;
  void decodeAsps(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                  AtlasSequenceParameterSetRBSP asps);
  void decodeAfps(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                  const AtlasFrameParameterSetRBSP &afps);
  void decodeAaps(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                  const AtlasAdaptationParameterSetRBSP &aaps);
  void decodeFoc(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                 const FrameOrderCountRBSP &foc);
  static void decodeAud(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                        AccessUnitDelimiterRBSP aud);
  void decodeEos(const V3cUnitHeader &vuh, const NalUnitHeader &nuh);
  void decodeEob(const V3cUnitHeader &vuh, const NalUnitHeader &nuh);
  static void decodeSei(const V3cUnitHeader &vuh, const NalUnitHeader &nuh, const SeiRBSP &sei);
  static void decodeSeiMessage(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                               const SeiMessage &message);
  static void decodeViewingSpaceHandling(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                                         const ViewingSpaceHandling &vh);
  static void decodeRecViewport(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                                const RecViewport &vh);

private: // Parsers
  void parseAsps(const V3cUnitHeader &vuh, const NalUnit &nu);
  void parseAfps(const V3cUnitHeader &vuh, const NalUnit &nu);
  void parseAaps(const V3cUnitHeader &vuh, const NalUnit &nu, const uint8_t atlasCountMinus1);
  void parseFoc(const V3cUnitHeader &vuh, const NalUnit &nu);
  void parseAtl(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parseAud(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parseV3cAud(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parsePrefixNSei(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parseSuffixNSei(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parsePrefixESei(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parseSuffixESei(const V3cUnitHeader &vuh, const NalUnit &nu);
  static void parseViewingSpaceHandlingSei(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                                           const SeiMessage &message);
  static void parseRecViewportSei(const V3cUnitHeader &vuh, const NalUnitHeader &nuh,
                                  const SeiMessage &message);

private: // Internal decoder state
  std::istream &m_stream;
  GeoFrameServer m_geoFrameServer;
  AttrFrameServer m_attrFrameServer;
  SampleStreamV3cHeader m_ssvh;

  struct Atlas {
    std::vector<AtlasSequenceParameterSetRBSP> aspsV;
    std::vector<AtlasFrameParameterSetRBSP> afpsV;
    std::vector<AtlasAdaptationParameterSetRBSP> aapsV;

    struct Frame {
      AtlasTileHeader ath;
      ViewParamsList viewParamsList;
      PatchParamsList patchParamsList;
      Common::BlockToPatchMap blockToPatchMap;
    };

    std::vector<std::shared_ptr<Frame>> frames;

    std::shared_ptr<Frame> intraFrame;

    std::string geoVideoData;
    std::string attrVideoData;
    std::unique_ptr<VideoDecoder::VideoServer> geoVideoServer;
    std::unique_ptr<VideoDecoder::VideoServer> attrVideoServer;
  };

  struct Sequence {
    std::vector<Atlas> atlas;
    Atlas specialAtlas;
    std::int32_t frameId{-1}; // picture order count
  };

  std::vector<V3cParameterSet> m_vpsV;
  std::vector<Sequence> m_sequenceV;

  bool m_stop{};
  double m_totalGeoVideoDecodingTime{};
  double m_totalAttrVideoDecodingTime{};

private: // Bitrate reporting (pimpl idiom)
  std::unique_ptr<BitrateReport> m_bitrateReport;

private: // Access internal decoder state
  [[nodiscard]] auto vps(const V3cUnitHeader &vuh) const -> const V3cParameterSet &;
  [[nodiscard]] auto sequence(const V3cUnitHeader &vuh) const -> const Sequence &;
  auto sequence(const V3cUnitHeader &vuh) -> Sequence &;
  [[nodiscard]] auto atlas(const V3cUnitHeader &vuh) const -> const Atlas &;
  auto atlas(const V3cUnitHeader &vuh) -> Atlas &;
  [[nodiscard]] auto specialAtlas(const V3cUnitHeader &vuh) const -> const Atlas &;
  auto specialAtlas(const V3cUnitHeader &vuh) -> Atlas &;
  [[nodiscard]] auto aspsV(const V3cUnitHeader &vuh) const
      -> const std::vector<AtlasSequenceParameterSetRBSP> &;
  [[nodiscard]] auto afpsV(const V3cUnitHeader &vuh) const
      -> const std::vector<AtlasFrameParameterSetRBSP> &;
  [[nodiscard]] auto aapsV(const V3cUnitHeader &vuh) const
      -> const std::vector<AtlasAdaptationParameterSetRBSP> &;
};
} // namespace TMIV::Decoder

#endif
