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

#include <TMIV/Multiplexer/DecodeAnnexBStream.h>

#include <TMIV/Common/verify.h>

using namespace std::string_view_literals;

namespace TMIV::Multiplexer {
class AnnexBBytestreamDecoder {
public:
  explicit AnnexBBytestreamDecoder(std::shared_ptr<std::istream> stream)
      : m_stream{std::move(stream)} {
    VERIFY(m_stream->good());
  }

  auto operator()() -> std::optional<std::string> {
    VERIFY(m_state != State::end);

    if (m_state == State::flushing) {
      m_state = State::end;
      return std::nullopt;
    }

    if (m_state == State::initial) {
      if (m_stream->peek(), m_stream->eof()) {
        m_state = State::end;
        return std::nullopt;
      }

      m_state = State::decoding;
      skipStartCode();
    }

    m_buffer.clear();

    for (auto code = m_stream->get(); code != std::char_traits<char>::eof();
         code = m_stream->get()) {
      m_buffer.push_back(std::char_traits<char>::to_char_type(code));

      if (tryRemoveTrailingStartCode()) {
        return m_buffer;
      }
    }

    m_state = State::flushing;
    return m_buffer;
  }

private:
  void skipStartCode() {
    VERIFY_BITSTREAM(m_stream->get() == 0);
    VERIFY_BITSTREAM(m_stream->get() == 0);

    const auto code = m_stream->get();
    VERIFY_BITSTREAM(code == 0 || code == 1);

    if (code == 0) {
      VERIFY_BITSTREAM(m_stream->get() == 1);
    }
  }

  auto tryRemoveTrailingStartCode() -> bool {
    if (4 <= m_buffer.size() && m_buffer.substr(m_buffer.size() - 4, 4) == "\0\0\0\1"sv) {
      m_buffer.erase(m_buffer.size() - 4, 4);
      return true;
    }
    if (3 <= m_buffer.size() && m_buffer.substr(m_buffer.size() - 3, 3) == "\0\0\1"sv) {
      m_buffer.erase(m_buffer.size() - 3, 3);
      return true;
    }
    return false;
  }

  enum class State { initial, decoding, flushing, end };

  std::shared_ptr<std::istream> m_stream{};
  State m_state{State::initial};
  std::string m_buffer{};
};

auto decodeAnnexBStream(std::shared_ptr<std::istream> stream) -> Common::Source<std::string> {
  return [decoder = AnnexBBytestreamDecoder{std::move(stream)}]() mutable { return decoder(); };
}
} // namespace TMIV::Multiplexer
