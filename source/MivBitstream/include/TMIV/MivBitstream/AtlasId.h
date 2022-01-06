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

#ifndef TMIV_MIVBITSTREAM_ATLASID_H
#define TMIV_MIVBITSTREAM_ATLASID_H

#include <TMIV/Common/Bitstream.h>

#include <cstdint>
#include <ostream>

namespace TMIV::MivBitstream {
// Use a type to avoid confusing atlas index and ID
//
// NOTE(BK): The class interface deliberately disallows integer computations
class AtlasId {
public:
  constexpr AtlasId() noexcept = default;

  template <typename Integer, typename = std::enable_if_t<std::is_integral_v<Integer>>>
  constexpr explicit AtlasId(Integer value) noexcept
      : m_value{Common::verifyDownCast<uint8_t>(value)} {}

  friend auto operator<<(std::ostream &stream, AtlasId atlasId) -> std::ostream & {
    return stream << int32_t{atlasId.m_value};
  }

  constexpr auto operator==(AtlasId other) const noexcept { return m_value == other.m_value; }
  constexpr auto operator!=(AtlasId other) const noexcept { return m_value != other.m_value; }
  constexpr auto operator<(AtlasId other) const noexcept { return m_value < other.m_value; }
  constexpr auto operator>(AtlasId other) const noexcept { return m_value > other.m_value; }
  constexpr auto operator<=(AtlasId other) const noexcept { return m_value <= other.m_value; }
  constexpr auto operator>=(AtlasId other) const noexcept { return m_value >= other.m_value; }

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AtlasId {
    return AtlasId(bitstream.readBits<uint8_t>(6));
  }

  void encodeTo(Common::OutputBitstream &bitstream) const { bitstream.writeBits(m_value, 6); }

private:
  friend struct fmt::formatter<TMIV::MivBitstream::AtlasId>;

  uint8_t m_value{};
};
} // namespace TMIV::MivBitstream

template <> struct fmt::formatter<TMIV::MivBitstream::AtlasId> {
  fmt::formatter<int32_t> base;

  constexpr auto parse(format_parse_context &ctx) { return base.parse(ctx); }

  template <typename FormatContext>
  auto format(const TMIV::MivBitstream::AtlasId &id, FormatContext &ctx) {
    return base.format(id.m_value, ctx);
  }
};

#endif
