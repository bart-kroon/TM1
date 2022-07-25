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

#include <TMIV/Decoder/V3cUnitBuffer.h>

#include <fmt/format.h>

#include <utility>

namespace TMIV::Decoder {
V3cUnitBuffer::V3cUnitBuffer(Common::Source<MivBitstream::V3cUnit> source, OnVps onVps)
    : m_source{std::move(source)}, m_onVps{std::move(onVps)} {}

auto V3cUnitBuffer::operator()(MivBitstream::V3cUnitHeader vuh)
    -> std::optional<MivBitstream::V3cUnit> {
  auto i = m_buffer.begin();

  for (;;) {
    if (i == m_buffer.end()) {
      if (m_source == nullptr) {
        return {};
      }
      if (auto vu = m_source()) {
        i = m_buffer.insert(i, std::move(*vu));
      } else {
        m_source = nullptr;
        return {};
      }
    }
    if (i->v3c_unit_header() == vuh) {
      auto vu = std::move(*i);
      m_buffer.erase(i);
      return vu;
    }
    if (i->v3c_unit_header() == MivBitstream::V3cUnitHeader::vps()) {
      auto vu = std::move(*i);
      i = m_buffer.erase(i);
      m_onVps(vu);
    } else if (vuh == MivBitstream::V3cUnitHeader::vps()) {
      throw V3cUnitBufferError(fmt::format("Expected a VPS but found the following V3C unit: {}",
                                           i->v3c_unit_header().summary()));
    } else {
      ++i;
    }
  }
}

auto videoSubBitstreamSource(std::shared_ptr<V3cUnitBuffer> buffer, MivBitstream::V3cUnitHeader vuh)
    -> Common::Source<MivBitstream::VideoSubBitstream> {
  return [buffer = std::move(buffer), vuh]() -> std::optional<MivBitstream::VideoSubBitstream> {
    if (auto v3cUnit = (*buffer)(vuh)) {
      return v3cUnit->v3c_unit_payload().video_sub_bitstream();
    }
    return std::nullopt;
  };
}

auto atlasSubBitstreamSource(std::shared_ptr<V3cUnitBuffer> buffer, MivBitstream::V3cUnitHeader vuh)
    -> Common::Source<MivBitstream::AtlasSubBitstream> {
  return [buffer = std::move(buffer), vuh]() -> std::optional<MivBitstream::AtlasSubBitstream> {
    if (auto v3cUnit = (*buffer)(vuh)) {
      return v3cUnit->v3c_unit_payload().atlas_sub_bitstream();
    }
    return std::nullopt;
  };
}
} // namespace TMIV::Decoder
