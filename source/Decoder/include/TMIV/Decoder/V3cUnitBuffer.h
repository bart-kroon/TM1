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

#ifndef TMIV_DECODER_V3CUNITBUFFER_H
#define TMIV_DECODER_V3CUNITBUFFER_H

#include <TMIV/Common/Source.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <list>

namespace TMIV::Decoder {
class V3cUnitBuffer {
public:
  // Callback for when reading past a VPS in search for a certain V3C unit
  // The VPS at the start of the bitstream needs to be read explicitly
  using OnVps = std::function<void(MivBitstream::V3cUnit)>;

  V3cUnitBuffer(Common::Source<MivBitstream::V3cUnit> source, OnVps onVps);

  auto operator()(MivBitstream::V3cUnitHeader vuh) -> std::optional<MivBitstream::V3cUnit>;

private:
  Common::Source<MivBitstream::V3cUnit> m_source;
  std::list<MivBitstream::V3cUnit> m_buffer;
  OnVps m_onVps;
};

auto videoSubBitstreamSource(std::shared_ptr<V3cUnitBuffer> buffer, MivBitstream::V3cUnitHeader vuh)
    -> Common::Source<MivBitstream::VideoSubBitstream>;

auto atlasSubBitstreamSource(std::shared_ptr<V3cUnitBuffer> buffer, MivBitstream::V3cUnitHeader vuh)
    -> Common::Source<MivBitstream::AtlasSubBitstream>;

struct V3cUnitBufferError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};
} // namespace TMIV::Decoder

#endif
