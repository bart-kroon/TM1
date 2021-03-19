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

#ifndef TMIV_ENCODER_IMPIENCODER_H
#define TMIV_ENCODER_IMPIENCODER_H

#include <TMIV/Common/Frame.h>
#include <TMIV/Encoder/EncoderParams.h>
#include <TMIV/MivBitstream/SequenceConfig.h>

namespace TMIV::Encoder {
// TODO(BK): Study if IEncoder and IMpiEncoder can be fully or partially aligned to reduce code
// duplication in Encoder.main.cpp and MpiEncoder.main.cpp
class IMpiEncoder {
public:
  using MpiPcsFrameReader = std::function<Common::MpiPcs::Frame(int)>;

private:
  MpiPcsFrameReader m_mpiPcsFrameReader;

public:
  IMpiEncoder() = default;
  IMpiEncoder(const IMpiEncoder &) = delete;
  IMpiEncoder(IMpiEncoder &&) = default;
  auto operator=(const IMpiEncoder &) -> IMpiEncoder & = delete;
  auto operator=(IMpiEncoder &&) -> IMpiEncoder & = default;
  virtual ~IMpiEncoder() = default;
  void setMpiPcsFrameReader(const MpiPcsFrameReader &mpiPcsFrameReader) {
    m_mpiPcsFrameReader = mpiPcsFrameReader;
  }
  virtual void prepareSequence(const MivBitstream::SequenceConfig &config) = 0;
  virtual auto processAccessUnit(int firstFrameId, int lastFrameId) -> const EncoderParams & = 0;
  virtual auto popAtlas() -> Common::MVD10Frame = 0;
  [[nodiscard]] virtual auto maxLumaSamplesPerFrame() const -> size_t = 0;

protected:
  auto readFrame(int frameIndex) -> Common::MpiPcs::Frame {
    return m_mpiPcsFrameReader(frameIndex);
  }
};
} // namespace TMIV::Encoder

#endif
