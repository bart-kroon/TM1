/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_COMMON_BITSTREAM_H
#error "Include the .h, not the .hpp"
#endif

#include "verify.h"

namespace TMIV::Common {
template <typename Integral, typename> auto ceilLog2(Integral range) -> uint8_t {
  if constexpr (std::is_signed_v<Integral>) {
    PRECONDITION(0 <= range);
  }
  auto range_ = static_cast<std::make_unsigned_t<Integral>>(range);

  if (range_ == 0) {
    return 0;
  }
  --range_;
  auto bits = uint8_t{};
  while (range_ > 0) {
    ++bits;
    range_ /= 2;
  }
  return bits;
}

using uchar = std::make_unsigned_t<std::istream::char_type>;
constexpr uint32_t charBits = std::numeric_limits<uchar>::digits;

template <typename Ordinal, typename Integral, typename>
auto InputBitstream::readBits(Integral bits) -> Ordinal {
  static_assert(is_ordinal_v<Ordinal>);

  const auto bits_ = downCast<uint32_t>(bits);

  if constexpr (std::is_enum_v<Ordinal>) {
    VERIFY_BITSTREAM(bits_ <= std::numeric_limits<std::underlying_type_t<Ordinal>>::digits);
  } else {
    VERIFY_BITSTREAM(bits_ <= std::numeric_limits<Ordinal>::digits);
  }

  while (m_size < bits_) {
    VERIFY_BITSTREAM(m_size + charBits <= std::numeric_limits<uint64_t>::digits);
    VERIFY_BITSTREAM(m_stream.get().good());

    const auto value = m_stream.get().get();
    m_buffer = (m_buffer << charBits) | static_cast<uchar>(value);
    m_size += charBits;
  }

  m_size -= bits_;
  auto value = m_buffer >> m_size;
  m_buffer &= (1 << m_size) - 1;

  VERIFY_BITSTREAM(static_cast<uint64_t>(Ordinal(value)) == value);
  return static_cast<Ordinal>(value);
}

template <typename Ordinal1, typename Ordinal2, typename>
auto InputBitstream::getUVar(Ordinal2 range) -> Ordinal1 {
  static_assert(is_ordinal_v<Ordinal1>);

  return readBits<Ordinal1>(ceilLog2(range));
}

template <typename Ordinal> auto InputBitstream::getUExpGolomb() -> Ordinal {
  static_assert(is_ordinal_v<Ordinal>);

  auto leadingBits = 0U;
  while (!getFlag()) {
    ++leadingBits;
  }
  const auto mask = (uint64_t{1} << leadingBits) - 1;
  const auto value = mask + readBits<uint64_t>(leadingBits);

  VERIFY_BITSTREAM(static_cast<uint64_t>(static_cast<Ordinal>(value)) == value);
  return static_cast<Ordinal>(value);
}

template <typename Ordinal> auto InputBitstream::getSExpGolomb() -> Ordinal {
  static_assert(is_ordinal_v<Ordinal>);

  const auto codeNum = getUExpGolomb<uint64_t>();
  const auto absValue = static_cast<int64_t>((codeNum + 1) / 2);
  const auto value = (codeNum & 1) == 1 ? absValue : -absValue;

  VERIFY_BITSTREAM(static_cast<int64_t>(static_cast<Ordinal>(value)) == value);
  return static_cast<Ordinal>(value);
}

template <typename Ordinal, typename Integral, typename>
void OutputBitstream::writeBits(Ordinal value, Integral bits) {
  if constexpr (std::is_signed_v<Ordinal>) {
    VERIFY_BITSTREAM(value >= 0);
  }
  writeBits_(std::make_unsigned_t<Ordinal>(value), downCast<uint32_t>(bits));
}

template <typename Ordinal1, typename Ordinal2, typename>
void OutputBitstream::putUVar(Ordinal1 value, Ordinal2 range) {
  if constexpr (std::is_signed_v<Ordinal1>) {
    VERIFY_BITSTREAM(0 <= value);
  }
  if constexpr (std::is_signed_v<Ordinal2>) {
    VERIFY_BITSTREAM(0 <= range);
  }
  putUVar_(std::make_unsigned_t<Ordinal1>(value), std::make_unsigned_t<Ordinal2>(range));
}

template <typename Ordinal, typename> void OutputBitstream::putUExpGolomb(Ordinal value) {
  if constexpr (std::is_signed_v<Ordinal>) {
    VERIFY_BITSTREAM(value >= 0);
  }
  putUExpGolomb_(std::make_unsigned_t<Ordinal>(value));
}
} // namespace TMIV::Common
