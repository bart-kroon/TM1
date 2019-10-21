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

#ifndef _TMIV_METADATA_BITSTREAM_H_
#define _TMIV_METADATA_BITSTREAM_H_

#include <cstdint>
#include <istream>

namespace TMIV::Metadata {
class InputBitstream {
public:
  explicit InputBitstream(std::istream &stream) : m_stream{stream} {}
  InputBitstream(const InputBitstream &) = delete;
  InputBitstream(InputBitstream &&) = default;
  InputBitstream &operator=(const InputBitstream &) = delete;
  InputBitstream &operator=(InputBitstream &&) = delete;
  ~InputBitstream() = default;

  auto readBits(unsigned bits) -> uint_least64_t;
  bool getFlag() { return readBits(1) > 0; }
  auto getUint8() -> uint8_t { return uint8_t(readBits(8)); }
  auto getUint16() -> uint16_t { return uint16_t(readBits(16)); }
  auto getUint32() -> uint32_t { return uint32_t(readBits(32)); }
  auto getFloat32() -> float;
  auto getUVar(uint_least64_t range) -> uint_least64_t;
  auto getUExpGolomb() -> uint_least64_t;
  void byteAlign();
  void reset();

private:
  std::istream &m_stream;
  uint_least64_t m_buffer{};
  unsigned m_size{};
};

class OutputBitstream {
public:
  explicit OutputBitstream(std::ostream &stream) : m_stream{stream} {}
  OutputBitstream(const OutputBitstream &) = delete;
  OutputBitstream(OutputBitstream &&) = default;
  OutputBitstream &operator=(const OutputBitstream &) = delete;
  OutputBitstream &operator=(OutputBitstream &&) = delete;
  ~OutputBitstream() { byteAlign(); };

  void writeBits(uint_least64_t value, unsigned bits);
  void putFlag(bool value) { writeBits(int(value), 1); }
  void putUint8(uint8_t value) { writeBits(value, 8); }
  void putUint16(uint16_t value) { writeBits(value, 16); }
  void putUint32(uint32_t value) { writeBits(value, 32); }
  void putFloat32(float value);
  void putUVar(uint_least64_t value, uint_least64_t range);
  void putUExpGolomb(uint_least64_t value);
  void byteAlign();

private:
  std::ostream &m_stream;
  uint_least64_t m_buffer{};
  unsigned m_size{};
};
} // namespace TMIV::Metadata

#endif
