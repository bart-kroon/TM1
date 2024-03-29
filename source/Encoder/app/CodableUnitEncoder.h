/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_ENCODER_APP_CODABLEUNITENCODER_H
#define TMIV_ENCODER_APP_CODABLEUNITENCODER_H

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/Sink.h>
#include <TMIV/Encoder/Encoder.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <fstream>

namespace TMIV::Encoder {
using MivBitstream::EncoderParams;

class CodableUnitEncoder : public Common::IStageSink<CodableUnit> {
public:
  CodableUnitEncoder(const Common::Json &config, IO::Placeholders placeholders);

  void encode(CodableUnit frame) override;

  auto saveV3cFrameList(const Common::DeepFrameList &deepFrameList) const -> Common::Json::Array;
  auto saveAtlasFrame(MivBitstream::AtlasId atlasId, int32_t frameIdx,
                      const Common::DeepFrame &frame) const -> Common::Json::Array;

  void flush() override;

  [[nodiscard]] auto bytesWritten() { return m_outputBitstream.tellp(); }

private:
  const Common::Json &m_config;
  IO::Placeholders m_placeholders;
  std::filesystem::path m_outputBitstreamPath;
  std::ofstream m_outputBitstream;
  Common::Sink<EncoderParams> m_mivEncoder;
  int32_t m_outputFrameIdx{};
};
} // namespace TMIV::Encoder

#endif
