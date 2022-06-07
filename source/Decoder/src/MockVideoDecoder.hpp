/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_DECODER_MOCK_VIDEO_DECODER_H
#define TMIV_DECODER_MOCK_VIDEO_DECODER_H

#include <TMIV/Common/Frame.h>
#include <TMIV/Common/Source.h>
#include <TMIV/MivBitstream/VideoSubBitstream.h>

using namespace std::string_view_literals;

namespace TMIV::Decoder::test {
// Simulate a video decoder that does not return a frame, for instance because the decoding happens
// on the GPU
class MockVideoDecoder {
public:
  MockVideoDecoder(Common::Source<MivBitstream::VideoSubBitstream> source, int frameCount,
                   int intraPeriod)
      : m_source{source}, m_frameCount{frameCount}, m_intraPeriod{intraPeriod} {}

  auto operator()() {
    auto result = std::optional<Common::DecodedFrame>{};

    PRECONDITION(m_frameIdx <= m_frameCount);

    if (m_frameIdx < m_frameCount) {
      result = Common::DecodedFrame{};
      result->irap = m_frameIdx % m_intraPeriod == 0;

      if (result->irap && m_have) {
        _consume();
      }
    } else {
      _flush();
    }

    ++m_frameIdx;
    return result;
  }

private:
  void _consume() { m_have = m_source().has_value(); }

  void _flush() {
    while (m_have) {
      _consume();
    }
  }

  Common::Source<MivBitstream::VideoSubBitstream> m_source;
  int m_frameCount;
  int m_intraPeriod;
  int m_frameIdx{};
  bool m_have{true};
};
} // namespace TMIV::Decoder::test

#endif
