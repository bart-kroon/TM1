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

#ifndef TMIV_COMMON_DECODER_H
#define TMIV_COMMON_DECODER_H

#include "Source.h"
#include "verify.h"

#include <queue>

namespace TMIV::Common {
// A decoder is a source of B's that adapts a source of A's.
//
// For instance, a video decoder is a source of video frames that adapts a source of NAL units.
template <typename A, typename B> class Decoder {
public:
  explicit Decoder(Common::Source<A> source) : m_source{std::move(source)} {}

  Decoder(const Decoder &) = delete;
  Decoder(Decoder &&) noexcept = default;
  auto operator=(const Decoder &) -> Decoder & = delete;
  auto operator=(Decoder &&) noexcept -> Decoder & = default;
  virtual ~Decoder() = default;

  auto operator()() -> std::optional<B> {
    VERIFY(m_state != State::end);

    if (m_state == State::initial) {
      m_state = State::decoding;
    }

    while (m_state == State::decoding && m_buffer.empty()) {
      if (!decodeSome()) {
        m_state = State::flushing;
      }
    }

    if (m_buffer.empty()) {
      m_state = State::end;
      return std::nullopt;
    }

    auto frame = std::move(m_buffer.front());
    m_buffer.pop();
    return frame;
  }

protected:
  // The abstract decodeSome method pulls any number of A's to push any number of B's and returns
  // true iff there is potentially more to decode.
  virtual auto decodeSome() -> bool = 0;

  auto pull() -> std::optional<A> { return m_source ? m_source() : std::nullopt; }
  void push(B value) { m_buffer.push(std::move(value)); }

private:
  enum class State { initial, decoding, flushing, end };

  State m_state{State::initial};
  Common::Source<A> m_source;
  std::queue<B> m_buffer;
};

template <typename A, typename B, typename... Args>
using DecoderFactory = std::function<Common::Source<B>(Common::Source<A>, Args...)>;
} // namespace TMIV::Common

#endif
