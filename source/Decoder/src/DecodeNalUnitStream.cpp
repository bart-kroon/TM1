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

#include <TMIV/Decoder/DecodeNalUnitStream.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Decoder.h>
#include <TMIV/Common/verify.h>

#include <sstream>

namespace TMIV::Decoder {
class NalUnitBytestreamDecoder : public Common::Decoder<std::string, std::string> {
public:
  NalUnitBytestreamDecoder(Common::Source<std::string> source)
      : Common::Decoder<std::string, std::string>{std::move(source)} {}

protected:
  auto decodeSome() -> bool override {
    if (auto bytestream = pull()) {
      auto pos = size_t{};

      while (pos < bytestream->size()) {
        // Decode the NAL unit size
        VERIFY_BITSTREAM(pos + 4 <= bytestream->size());
        std::istringstream stream{bytestream->substr(pos, 4)};
        const auto size = Common::getUint32(stream);
        pos += 4;

        // Decode the payload bytes
        VERIFY_BITSTREAM(pos + size <= bytestream->size());
        push(bytestream->substr(pos, size));
        pos += size;
      }
      return true;
    }
    return false;
  }
};

auto decodeNalUnitStream(Common::Source<std::string> source) -> Common::Source<std::string> {
  return [decoder = std::make_shared<NalUnitBytestreamDecoder>(std::move(source))]() {
    return (*decoder)();
  };
}
} // namespace TMIV::Decoder
