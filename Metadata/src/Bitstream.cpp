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

#include <TMIV/Metadata/Bitstream.h>

#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

#include "verify.h"

using namespace std;

namespace TMIV::Metadata {
auto InputBitstream::readBits(unsigned bits) -> uint_least64_t {
  using uchar = make_unsigned_t<istream::char_type>;
  constexpr unsigned chunk = numeric_limits<uchar>::digits;

  while (m_size < bits) {
    assert(m_size + chunk <= numeric_limits<uint_least64_t>::digits);

    m_buffer = (m_buffer << chunk) | uchar(m_stream.get());
    m_size += chunk;
  }

  m_size -= bits;
  auto value = m_buffer >> m_size;
  m_buffer &= (1 << m_size) - 1;
  return value;
}

auto InputBitstream::getFloat32() -> float {
  uint32_t code = getUint32();
  float value;
  memcpy(&value, &code, 4);
  return value;
}

namespace {
auto ceilLog2(uint_least64_t range) -> unsigned {
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
  auto value = readBits(ceilLog2(range));
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

void InputBitstream::reset() {
  m_size = 0;
  m_buffer = 0;
}

void OutputBitstream::writeBits(uint_least64_t value, unsigned bits) {
  using uchar = make_unsigned_t<istream::char_type>;
  constexpr unsigned chunk = numeric_limits<uchar>::digits;

  verify((value >> bits) == 0);
  assert(m_size + bits <= numeric_limits<uint_least64_t>::digits);

  m_buffer = (m_buffer << bits) | value;
  m_size += bits;

  while (m_size >= chunk) {
    m_size -= chunk;
    m_stream.put(char(m_buffer >> m_size));
  }
}

void OutputBitstream::putUVar(uint_least64_t value, uint_least64_t range) {
  verify(!value || value < range);
  return writeBits(value, ceilLog2(range));
}

void OutputBitstream::putUExpGolomb(uint_least64_t value) {
  auto bits = ceilLog2(value + 2) - 1;
  for (auto i = 0U; i < bits; ++i) {
    putFlag(true);
  }
  putFlag(false);
  auto mask = (uint_least64_t{1} << bits) - 1;
  writeBits(value - mask, bits);
}

void OutputBitstream::putFloat32(float value) {
  uint32_t code;
  memcpy(&code, &value, 4);
  putUint32(code);
}

void OutputBitstream::byteAlign() { writeBits(0, (1 + ~m_size) % 8); }
} // namespace TMIV::Metadata
