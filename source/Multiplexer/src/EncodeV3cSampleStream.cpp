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

#include <TMIV/Multiplexer/EncodeV3cSampleStream.h>

#include <sstream>

namespace TMIV::Multiplexer {
namespace {
auto encodeV3cUnits(Common::Source<MivBitstream::V3cUnit> source) {
  return [source = std::move(source)]() -> std::optional<std::string> {
    if (auto unit = source()) {
      std::ostringstream stream;
      unit->encodeTo(stream);
      return stream.str();
    }
    return std::nullopt;
  };
}

void encodeV3cSampleStream(MivBitstream::SampleStreamV3cHeader ssvh,
                           const Common::Source<std::string> &source, std::ostream &stream) {
  ssvh.encodeTo(stream);

  while (auto unit = source()) {
    MivBitstream::SampleStreamV3cUnit{*unit}.encodeTo(stream, ssvh);
  }
}

void encodeV3cSampleStream(const Common::Source<std::string> &source, std::ostream &stream) {
  std::vector<std::string> buffer;
  auto precisionBytesMinus1 = uint8_t{};

  while (auto unit = source()) {
    while (unit->size() >= uint64_t{1} << 8 * (precisionBytesMinus1 + 1)) {
      ++precisionBytesMinus1;
    }
    buffer.push_back(*unit);
  }

  encodeV3cSampleStream(MivBitstream::SampleStreamV3cHeader{precisionBytesMinus1},
                        Common::sourceFromIteratorPair(buffer.cbegin(), buffer.cend()), stream);
}
} // namespace

void encodeV3cSampleStream(MivBitstream::SampleStreamV3cHeader ssvh,
                           Common::Source<MivBitstream::V3cUnit> source, std::ostream &stream) {
  return encodeV3cSampleStream(ssvh, encodeV3cUnits(std::move(source)), stream);
}

void encodeV3cSampleStream(Common::Source<MivBitstream::V3cUnit> source, std::ostream &stream) {
  return encodeV3cSampleStream(encodeV3cUnits(std::move(source)), stream);
}
} // namespace TMIV::Multiplexer
