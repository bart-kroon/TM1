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

#ifndef TMIV_ENCODER_ANNEX_B_H
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/verify.h>

namespace TMIV::Encoder {
template <typename StreamChar, typename StreamTraits, typename BufferChar>
void readNalUnitFromAnnexBStreamIntoBuffer(std::basic_istream<StreamChar, StreamTraits> &stream,
                                           std::vector<BufferChar> &buffer) {
  VERIFY(stream.good());

  buffer.clear();

  if (stream.peek(), stream.eof()) {
    return;
  }

  VERIFY_BITSTREAM(stream.get() == 0);
  VERIFY_BITSTREAM(stream.get() == 0);

  auto code = stream.get();
  VERIFY_BITSTREAM(code == 0 || code == 1);

  if (code == 0) {
    code = stream.get();
    VERIFY_BITSTREAM(code == 1);
  }

  auto zeroCount = 0;
  code = stream.get();

  while (code != std::char_traits<StreamChar>::eof()) {
    if (code == 0 && zeroCount < 3) {
      ++zeroCount;
    } else if (code == 1 && 2 <= zeroCount) {
      stream.seekg(std::streamoff{-1} - zeroCount, std::ios::cur);
      return;
    } else {
      while (0 < zeroCount) {
        buffer.emplace_back();
        --zeroCount;
      }
      static_assert(sizeof(StreamChar) == 1);
      static_assert(sizeof(BufferChar) == 1);
      buffer.push_back(static_cast<StreamChar>(code));
    }
    code = stream.get();
  }

  stream.clear();
}
} // namespace TMIV::Encoder
