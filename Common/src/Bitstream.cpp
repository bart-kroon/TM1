/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/Common/Bitstream.h>

#include <cstring>
#include <iostream>
#include <limits>

using namespace std;

namespace {
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define verify(condition) (void)(!!(condition) || verifyFailed(#condition, __FILE__, __LINE__))

auto verifyFailed(char const *condition, char const *file, int line) -> bool {
  cerr << "Failed to encode/decode bitstream: " << condition << " [" << file << "@" << line << endl;
  abort();
  return false;
}
} // namespace

namespace TMIV::Common {
using uchar = make_unsigned_t<istream::char_type>;
constexpr unsigned charBits = numeric_limits<uchar>::digits;

auto InputBitstream::tellg() const -> std::streampos {
  return m_stream.tellg() * charBits - m_size;
}

auto InputBitstream::readBits(unsigned bits) -> uint_least64_t {
  while (m_size < bits) {
    verify(m_size + charBits <= numeric_limits<uint_least64_t>::digits);
    verify(m_stream.good());

    const auto value = m_stream.get();
    m_buffer = (m_buffer << charBits) | uchar(value);
    m_size += charBits;
  }

  m_size -= bits;
  auto value = m_buffer >> m_size;
  m_buffer &= (1 << m_size) - 1;
  return value;
}

auto InputBitstream::getFloat16() -> Common::Half {
  using Common::Half;
  return Half::decode(getUint16());
}

auto InputBitstream::getFloat32() -> float {
  uint32_t code = getUint32();
  float value;
  memcpy(&value, &code, 4);
  return value;
}

namespace {
auto numBits(uint_least64_t range) -> unsigned {
  if (range == 0) {
    return 0;
  }
  --range;
  unsigned bits = 0;
  while (range > 0) {
    ++bits;
    range /= 2;
  }
  return bits;
}
} // namespace

auto InputBitstream::getUVar(uint_least64_t range) -> uint_least64_t {
  auto value = readBits(numBits(range));
  verify(!value || value < range);
  return value;
}

auto InputBitstream::getUExpGolomb() -> uint_least64_t {
  auto leadingBits = 0U;
  while (getFlag()) {
    ++leadingBits;
  }
  auto mask = (uint_least64_t{1} << leadingBits) - 1;
  return mask + readBits(leadingBits);
}

void InputBitstream::byteAlign() {
  if (readBits(m_size % 8) != 0) {
    throw runtime_error("Non-zero bit in byte alignment");
  }
}

void InputBitstream::rbspTrailingBits() {
  const auto rbsp_stop_one_bit = getFlag();
  verify(rbsp_stop_one_bit);
  byteAlign();
}

auto InputBitstream::moreData() -> bool {
  verify(m_stream.good() && !m_stream.eof());

  if (m_size > 0) {
    return true;
  }
  m_stream.peek();
  auto result = !m_stream.eof();
  m_stream.clear();
  return result;
}

auto InputBitstream::moreRbspData() -> bool {
  // Return false if there is no more data.
  if (!moreData()) {
    return false;
  }

  // Store bitstream state.
  const auto streamPos = m_stream.tellg();
  const auto size = m_size;
  const auto buffer = m_buffer;

  // Skip first bit. It may be part of a RBSP or a rbsp_one_stop_bit.
  getFlag();

  while (moreData()) {
    if (getFlag()) {
      // We found a one bit beyond the first bit. Restore bitstream state and return true.
      m_stream.seekg(streamPos);
      m_stream.clear();
      m_size = size;
      m_buffer = buffer;
      return true;
    }
  }

  // We did not found a one bit beyond the first bit. Restore bitstream state and return false.
  m_stream.seekg(streamPos);
  m_stream.clear();
  m_size = size;
  m_buffer = buffer;
  return false;
}

void InputBitstream::reset() {
  m_size = 0;
  m_buffer = 0;
}

auto OutputBitstream::tellp() const -> std::streampos {
  return charBits * m_stream.tellp() + m_size;
}

void OutputBitstream::writeBits(uint_least64_t value, unsigned bits) {
  verify((value >> bits) == 0);
  verify(m_size + bits <= numeric_limits<uint_least64_t>::digits);

  m_buffer = (m_buffer << bits) | value;
  m_size += bits;

  while (m_size >= charBits) {
    m_size -= charBits;
    m_stream.put(char(m_buffer >> m_size));
  }
}

void OutputBitstream::putUVar(uint_least64_t value, uint_least64_t range) {
  verify(!value || value < range);
  return writeBits(value, numBits(range));
}

void OutputBitstream::putUExpGolomb(uint_least64_t value) {
  auto bits = numBits(value + 2) - 1;
  for (auto i = 0U; i < bits; ++i) {
    putFlag(true);
  }
  putFlag(false);
  auto mask = (uint_least64_t{1} << bits) - 1;
  writeBits(value - mask, bits);
}

void OutputBitstream::putFloat16(Common::Half value) { putUint16(value.encode()); }

void OutputBitstream::putFloat32(float value) {
  uint32_t code;
  memcpy(&code, &value, 4);
  putUint32(code);
}

void OutputBitstream::byteAlign() { writeBits(0, (1 + ~m_size) % 8); }

void OutputBitstream::rbspTrailingBits() {
  const auto rbsp_stop_one_bit = true;
  putFlag(rbsp_stop_one_bit);
  byteAlign();
}
} // namespace TMIV::Common
