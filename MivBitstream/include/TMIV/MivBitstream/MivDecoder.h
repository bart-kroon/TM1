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

#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>
#include <TMIV/MivBitstream/AdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSubBitstream.h>
#include <TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h>
#include <TMIV/MivBitstream/EndOfAtlasSubBitstreamRBSP.h>
#include <TMIV/MivBitstream/EndOfSequenceRBSP.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>
#include <TMIV/MivBitstream/VpccParameterSet.h>
#include <TMIV/MivBitstream/VpccSampleStreamFormat.h>
#include <TMIV/MivBitstream/VpccUnit.h>

#include <array>

namespace TMIV::MivBitstream {
class MivDecoder {
public:
  enum Mode {
    MIV, // Parse a 3VC bitstream with MIV extension
    TMC2 // Parse a bitstream that was produced by TMC2
  };

  // The mode varies per executable. The MIV decoder mode is normally MIV. The other modes are for
  // testing purposes.
  static const Mode mode;

  // Construct a MivDecoder and read the sample stream V-PCC header
  explicit MivDecoder(std::istream &stream);

  // Decode the next V-PCC unit
  auto decodeVpccUnit() -> bool;

  // Decode everything
  void decode();

protected:
  // This function is called when a V-PCC unit of unknown type has been decoded.
  virtual void onVpccPayload(const VpccUnitHeader &vuh, const std::monostate &payload);

  // This function is called when a V-PCC parameter set (VPS) has been decoded.
  virtual void onVpccPayload(const VpccUnitHeader &vuh, const VpccParameterSet &vps);

  // This function is called when an atlas sub bitstream V-PCC unit has been decoded.
  virtual void onVpccPayload(const VpccUnitHeader &vuh, const AtlasSubBitstream &ad);

  // This function is called when a video sub bitstream V-PCC unit has been decoded.
  virtual void onVpccPayload(const VpccUnitHeader &vuh, const VideoSubBitstream &vd);

  // This function is called for each decoded nal unit.
  virtual void onNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu);

  // This function is called for each decoded nal unit of unknown type.
  virtual void onUnknownNalUnit(const VpccUnitHeader &vuh, const NalUnit &nu);

  // This function is called when an atlas tile group layer (ATGL) has been decoded.
  virtual void onAtgl(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                      AtlasTileGroupLayerRBSP atgl);

  // This function is called when an atlas sequence parameter set (ASPS) has been decoded.
  virtual void onAsps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                      AtlasSequenceParameterSetRBSP asps);

  // This function is called when an atlas sequence parameter set (ASPS) has been decoded.
  virtual void onAfps(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                      AtlasFrameParameterSetRBSP afps);

  // This function is called when an access unit delimiter (AUD) has been decoded.
  virtual void onAud(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                     AccessUnitDelimiterRBSP aud);

  // This function is called when a V-PCC access unit delimiter (V-PCC AUD) has been decoded.
  virtual void onVpccAud(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                         AccessUnitDelimiterRBSP aud);

  // This function is called when an end of sequence (EOS) has been decoded.
  virtual void onEos(const VpccUnitHeader &vuh, const NalUnitHeader &nuh, EndOfSequenceRBSP eos);

  // This function is called when an end of atlas sub bitstream (EOB) has been decoded.
  virtual void onEob(const VpccUnitHeader &vuh, const NalUnitHeader &nuh,
                     EndOfAtlasSubBitstreamRBSP eob);

  // This function is called when a prefix non-essential SEI has been decoded.
  virtual void onPrefixNSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh, SeiRBSP sei);

  // This function is called when a suffix non-essential SEI has been decoded.
  virtual void onSuffixNSei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh, SeiRBSP sei);

  // This function is called when a prefix essential SEI has been decoded.
  virtual void onPrefixESei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh, SeiRBSP sei);

  // This function is called when aa suffix essential SEI has been decoded.
  virtual void onSuffixESei(const VpccUnitHeader &vuh, const NalUnitHeader &nuh, SeiRBSP sei);

private:
  std::istream &m_stream;
  SampleStreamVpccHeader m_ssvh;

  struct Atlas {
    std::vector<AtlasSequenceParameterSetRBSP> asps;
    std::vector<AtlasFrameParameterSetRBSP> afps;
    std::vector<AdaptationParameterSetRBSP> aps;
    AtlasTileGroupLayerRBSP atgl;
  };

  struct Sequence {
    std::vector<Atlas> atlas;
    std::optional<Atlas> specialAtlas;
  };

  static auto sampleStreamVpccHeader(std::istream &) -> SampleStreamVpccHeader;

  void decodeAcl(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeAsps(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeAfps(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeAud(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeVpccAud(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeEos(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeEob(const VpccUnitHeader &vuh, const NalUnit &nu);
  static void decodeFd(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodePrefixNSei(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeSuffixNSei(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodePrefixESei(const VpccUnitHeader &vuh, const NalUnit &nu);
  void decodeSuffixESei(const VpccUnitHeader &vuh, const NalUnit &nu);

  auto sequence(const VpccUnitHeader &vuh) const -> const Sequence &;
  auto sequence(const VpccUnitHeader &vuh) -> Sequence &;
  auto atlas(const VpccUnitHeader &vuh) const -> const Atlas &;
  auto atlas(const VpccUnitHeader &vuh) -> Atlas &;
  auto specialAtlas(const VpccUnitHeader &vuh) const -> const Atlas &;
  auto specialAtlas(const VpccUnitHeader &vuh) -> Atlas &;
  auto asps(const VpccUnitHeader &vuh) const -> const std::vector<AtlasSequenceParameterSetRBSP> &;
  auto afps(const VpccUnitHeader &vuh) const -> const std::vector<AtlasFrameParameterSetRBSP> &;

  std::vector<VpccParameterSet> m_vps;
  std::vector<Sequence> m_sequence;
};
} // namespace TMIV::MivBitstream

#endif
