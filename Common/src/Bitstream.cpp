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

#include <TMIV/Common/Bitstream.h>

#include <cstring>
#include <iostream>
#include <limits>

using namespace std;

namespace TMIV::Common {
auto verifyBitstreamFailed(char const *condition, char const *file, int line) -> bool {
  cerr << "Failed to encode/decode bitstream: " << condition << " [" << file << "@" << line << endl;
  abort();
  return false;
}

auto InputBitstream::tellg() const -> streampos { return m_stream.tellg() * charBits - m_size; }

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

auto ceilLog2(uint64_t range) -> unsigned {
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

void InputBitstream::byteAlign() {
  if (readBits<uint8_t>(m_size % 8) != 0) {
    throw runtime_error("Non-zero bit in byte alignment");
  }
}

void InputBitstream::rbspTrailingBits() {
  const auto rbsp_stop_one_bit = getFlag();
  VERIFY_BITSTREAM(rbsp_stop_one_bit);
  byteAlign();
}

auto InputBitstream::moreData() -> bool {
  VERIFY_BITSTREAM(m_stream.good() && !m_stream.eof());

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

auto OutputBitstream::tellp() const -> streampos { return charBits * m_stream.tellp() + m_size; }

void OutputBitstream::writeBits_(uint64_t value, unsigned bits) {
  VERIFY_BITSTREAM((value >> bits) == 0);
  VERIFY_BITSTREAM(m_size + bits <= numeric_limits<uint64_t>::digits);

  m_buffer = (m_buffer << bits) | value;
  m_size += bits;

  while (m_size >= charBits) {
    m_size -= charBits;
    m_stream.put(char(m_buffer >> m_size));
  }
}

void OutputBitstream::putUVar_(uint64_t value, uint64_t range) {
  VERIFY_BITSTREAM(!value || value < range);
  return writeBits(value, ceilLog2(range));
}

void OutputBitstream::putUExpGolomb_(uint64_t value) {
  auto bits = ceilLog2(value + 2) - 1;
  for (auto i = 0U; i < bits; ++i) {
    putFlag(true);
  }
  putFlag(false);
  auto mask = (uint64_t{1} << bits) - 1;
  writeBits(value - mask, bits);
}

void OutputBitstream::putSExpGolomb(int64_t value) {
  putUExpGolomb((uint64_t(abs(value)) << 1U) - uint64_t(value > 0));
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