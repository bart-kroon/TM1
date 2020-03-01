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

#ifndef _TMIV_MIVBITSTREAM_MIVDECODER_H_
#define _TMIV_MIVBITSTREAM_MIVDECODER_H_

#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AtlasSubBitstream.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>
#include <TMIV/MivBitstream/VpccSampleStreamFormat.h>
#include <TMIV/MivBitstream/VpccUnit.h>

#include <TMIV/Common/Frame.h>

#include <array>
#include <functional>

namespace TMIV::MivBitstream {
class MivDecoder {
public: // Integration testing
  enum Mode {
    MIV, // Parse a 3VC bitstream with MIV extension
    TMC2 // Parse a bitstream that was produced by TMC2
  };

  // The mode varies per executable. The MIV decoder mode is normally MIV. The other modes are for
  // testing purposes.
  static const Mode mode;

public: // Frame servers
  using GeoFrameServer = std::function<Common::Depth10Frame(
      std::uint8_t atlasId, std::uint32_t frameId, Common::Vec2i frameSize)>;
  using AttrFrameServer = std::function<Common::Texture444Frame(
      std::uint8_t atlasId, std::uint32_t frameId, Common::Vec2i frameSize)>;

public: // Decoder interface
  // Construct a MivDecoder and read the sample stream V-PCC header
  //
  // This version of TMIV does not implement video data sub bitstreams so we need to smuggle in
  // those frames using a callback.  The attribute server will return empty frames if there is no
  // attribute. There is only one attribute and that is texture.
  MivDecoder(std::istream &stream, GeoFrameServer geoFrameServer, AttrFrameServer attrFrameServer);

  // Decode the next V-PCC unit
  //
  // Register listeners to obtain output. The decoding is stopped prematurely when any of the
  // listeners returns false.
  auto decodeVpccUnit() -> bool;

  // Decode (remainder of) V-PCC sample bitstream
  //
  // Register listeners to obtain output. The decoding is stopped prematurely when any of the
  // listeners returns false.
  void decode();

public: // Callback signatures
  // Callback that will be called when a VPS is decoded.
  using SequenceListener = std::function<bool(const VpccParameterSet &)>;

  // Callback that will be called when an access unit (frame) is decoded.
  using FrameListener = std::function<bool(const AccessUnit &)>;

public: // Callback registrations
  std::vector<SequenceListener> onSequence;
  std::vector<FrameListener> onFrame;

private: // Decoder output
  void outputSequence(const VpccParameterSet &vps);
  void outputFrame(const VpccUnitHeader &vuh);
  auto haveFrame(const VpccUnitHeader &vuh) const -> bool;

private: // Decoding processes
  void decodeVpccPayload(const VpccUnitHeader &vuh, const VpccPayload::Payload &payload);
  void decodeVps(const VpccUnitHeader &vuh, const VpccParameterSet &vps);
  void decodeAsb(const VpccUnitHeader &vuh, const AtlasSubBitstream &asb);

  void decodeNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void decodeUnknownNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu);

  void decodeAtgl(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                  const AtlasTileGroupLayerRBSP &atgl);
  static auto decodeMvpl(const MivViewParamsList &mvpl) -> ViewParamsList;
  static auto decodeAtgdu(const AtlasTileGroupDataUnit &atgdu,
                          const AtlasSequenceParameterSetRBSP &asps) -> PatchParamsList;
  static auto decodeBlockToPatchMap(const AtlasTileGroupDataUnit &atgdu,
                                    const AtlasSequenceParameterSetRBSP &asps)
      -> Common::BlockToPatchMap;
  void decodeAsps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                  AtlasSequenceParameterSetRBSP asps);
  void decodeAfps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                  AtlasFrameParameterSetRBSP afps);
  void decodeAps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                 const AdaptationParameterSetRBSP &aps);
  static void decodeAud(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                        AccessUnitDelimiterRBSP aud);
  static void decodeVpccAud(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                            AccessUnitDelimiterRBSP aud);
  void decodeEos(const VpccUnitHeader &vuh, const NalUnitHeader &nuh);
  void decodeEob(const VpccUnitHeader &vuh, const NalUnitHeader &nuh);
  static void decodeFd(const VpccUnitHeader &vuh, const NalUnitHeader &nuh);
  static void decodePrefixNSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                               const SeiRBSP &sei);
  static void decodeSuffixNSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                               const SeiRBSP &sei);
  static void decodePrefixESei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                               const SeiRBSP &sei);
  static void decodeSuffixESei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                               const SeiRBSP &sei);

private: // Parsers
  void parseAsps(const VpccUnitHeader &vuh, const NalUnit &nu);
  void parseAfps(const VpccUnitHeader &vuh, const NalUnit &nu);
  void parseAps(const VpccUnitHeader &vuh, const NalUnit &nu);
  void parseAtgl(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void parseAud(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void parseVpccAud(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void parsePrefixNSei(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void parseSuffixNSei(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void parsePrefixESei(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void parseSuffixESei(const VpccUnitHeader &vuh, const NalUnit &nu);

private: // Internal decoder state
  std::istream &m_stream;
  GeoFrameServer m_geoFrameServer;
  AttrFrameServer m_attrFrameServer;
  SampleStreamVpccHeader m_ssvh;

  struct Atlas {
    std::vector<AtlasSequenceParameterSetRBSP> aspsV;
    std::vector<AtlasFrameParameterSetRBSP> afpsV;
    std::vector<AdaptationParameterSetRBSP> apsV;

    struct Frame {
      AtlasTileGroupHeader atgh;
      ViewParamsList viewParamsList;
      PatchParamsList patchParamsList;
      Common::BlockToPatchMap blockToPatchMap;
    };

    std::vector<std::shared_ptr<Frame>> frames;
  };

  struct Sequence {
    std::vector<Atlas> atlas;
    std::optional<Atlas> specialAtlas;
    std::int32_t frameId{-1}; // picture order count
  };

  std::vector<VpccParameterSet> m_vpsV;
  std::vector<Sequence> m_sequenceV;

  bool m_stop{};

private: // Access internal decoder state
  auto vps(const VpccUnitHeader &vuh) const -> const VpccParameterSet &;
  auto sequence(const VpccUnitHeader &vuh) const -> const Sequence &;
  auto sequence(const VpccUnitHeader &vuh) -> Sequence &;
  auto atlas(const VpccUnitHeader &vuh) const -> const Atlas &;
  auto atlas(const VpccUnitHeader &vuh) -> Atlas &;
  auto specialAtlas(const VpccUnitHeader &vuh) const -> const Atlas &;
  auto specialAtlas(const VpccUnitHeader &vuh) -> Atlas &;
  auto aspsV(const VpccUnitHeader &vuh) const -> const std::vector<AtlasSequenceParameterSetRBSP> &;
  auto afpsV(const VpccUnitHeader &vuh) const -> const std::vector<AtlasFrameParameterSetRBSP> &;
  auto apsV(const VpccUnitHeader &vuh) const -> const std::vector<AdaptationParameterSetRBSP> &;
};
} // namespace TMIV::MivBitstream

#endif
