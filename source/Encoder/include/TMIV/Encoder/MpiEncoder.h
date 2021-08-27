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

#ifndef TMIV_ENCODER_MPIENCODER_H
#define TMIV_ENCODER_MPIENCODER_H

#include <TMIV/Common/Json.h>
#include <TMIV/Encoder/EncoderParams.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/MpiPcs/Frame.h>
#include <TMIV/Packer/IPacker.h>

#include <deque>
#include <memory>

namespace TMIV::Encoder {
class MpiEncoder {
public:
  MpiEncoder(const Common::Json &rootNode, const Common::Json &componentNode);

  void prepareSequence(const MivBitstream::SequenceConfig &config);
  auto processAccessUnit(int32_t firstFrameId, int32_t lastFrameId) -> const EncoderParams &;
  auto popAtlas() -> Common::MVD10Frame;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> size_t { return m_maxLumaSamplesPerFrame; }

  using MpiPcsFrameReader = std::function<MpiPcs::Frame(int32_t)>;

  void setMpiPcsFrameReader(const MpiPcsFrameReader &mpiPcsFrameReader) {
    m_mpiPcsFrameReader = mpiPcsFrameReader;
  }

private:
  [[nodiscard]] auto vuiParameters() const -> MivBitstream::VuiParameters;
  void setGiGeometry3dCoordinatesBitdepthMinus1();
  void prepareIvau();
  [[nodiscard]] auto log2FocLsbMinus4() const -> uint8_t;
  void incrementFoc();

  auto readFrame(int32_t frameIndex) -> MpiPcs::Frame { return m_mpiPcsFrameReader(frameIndex); }

  Common::Json m_rootNode;
  MpiPcsFrameReader m_mpiPcsFrameReader;

  // Parameters
  static constexpr auto maxIntraPeriod = 32;
  int32_t m_intraPeriod{};
  Common::Vec2i m_blockSizeDepthQualityDependent;
  std::vector<Common::Vec2i> m_overrideAtlasFrameSizes{};
  uint32_t m_textureDilation{};
  uint32_t m_transparencyDynamic{};

  // Attributes
  std::deque<MpiPcs::Frame> m_mpiFrameBuffer;
  std::vector<Common::BlockToPatchMap> m_blockToPatchMapPerAtlas;
  std::unique_ptr<Packer::IPacker> m_packer;
  int32_t m_blockSize{};
  size_t m_maxLumaSamplesPerFrame{};
  EncoderParams m_params;
  double m_frameRate{};
};
} // namespace TMIV::Encoder

#endif
