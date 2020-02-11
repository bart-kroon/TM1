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

#include <TMIV/MivBitstream/AdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSubBitstream.h>
#include <TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>
#include <TMIV/MivBitstream/VpccParameterSet.h>
#include <TMIV/MivBitstream/VpccSampleStreamFormat.h>
#include <TMIV/MivBitstream/VpccUnit.h>

#include <array>

namespace TMIV::MivBitstream {
class MivDecoder {
public:
  enum Mode {
    MIV, // The goal is to achieve maximum conformance with the MIV specification
    TMC2 // The goal is to parse a bitstream that was produced by TMC2
  };

  // Construct a MivDecoder and read the sample stream V-PCC header
  explicit MivDecoder(std::istream &stream, Mode mode = Mode::MIV);

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

  // This function is called when a vide osub bitstream V-PCC unit has been decoded.
  virtual void onVpccPayload(const VpccUnitHeader &vuh, const VideoSubBitstream &vd);

private:
  std::istream &m_stream;
  Mode m_mode;
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

  static auto sampleStreamVpccHeader(std::istream &, Mode) -> SampleStreamVpccHeader;

  std::vector<VpccParameterSet> m_vps;
  std::vector<Sequence> m_sequence;
};
} // namespace TMIV::MivBitstream

#endif
