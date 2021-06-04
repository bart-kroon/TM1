/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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
#define TMIV_COMMON_BITSTREAM_H

#include <TMIV/Common/Half.h>

#include <cstdint>
#include <istream>
#include <limits>
#include <type_traits>

namespace TMIV::Common {
auto ceilLog2(uint64_t range) -> uint8_t;

class InputBitstream {
public:
  explicit InputBitstream(std::istream &stream) : m_stream{stream} {}

  // Input bit position indicator
  [[nodiscard]] auto tellg() const -> std::streampos;

  template <typename Integer> auto readBits(unsigned bits) -> Integer;

  auto getFlag() -> bool { return readBits<uint8_t>(1) > 0; }
  auto getUint8() { return readBits<uint8_t>(8); }
  auto getUint16() { return readBits<uint16_t>(16); }
  auto getUint32() { return readBits<uint32_t>(32); }
  auto getUint64() -> uint64_t;
  auto getInt16() { return static_cast<int16_t>(getUint16()); }
  auto getInt32() { return static_cast<int32_t>(getUint32()); }
  auto getFloat16() -> Common::Half;
  auto getFloat32() -> float;

  template <typename Integer> auto getUVar(uint64_t range) -> Integer;

  template <typename Integer> auto getUExpGolomb() -> Integer;

  template <typename Integer> auto getSExpGolomb() -> Integer;

  [[nodiscard]] auto byteAligned() const -> bool;
  void byteAlignment();
  void zeroAlign();
  void rbspTrailingBits();
  auto moreData() -> bool;
  auto moreRbspData() -> bool;
  void reset();

  std::istream &m_stream;
  uint64_t m_buffer{};
  unsigned m_size{};
};

class OutputBitstream {
public:
  explicit OutputBitstream(std::ostream &stream) : m_stream{stream} {}

  // Output bit position indicator
  [[nodiscard]] auto tellp() const -> std::streampos;

  template <typename Integer> void writeBits(const Integer &value, unsigned bits);

  void putFlag(bool value) { writeBits(int{static_cast<int>(value)}, 1); }
  void putUint8(uint8_t value) { writeBits(value, 8); }
  void putUint16(uint16_t value) { writeBits(value, 16); }
  void putUint32(uint32_t value) { writeBits(value, 32); }
  void putUint64(uint64_t value);
  void putInt16(int16_t value) { putUint16(static_cast<uint16_t>(value)); }
  void putInt32(int32_t value) { putUint32(static_cast<uint32_t>(value)); }
  void putFloat16(Common::Half value);
  void putFloat32(float value);

  template <typename Integer> void putUVar(const Integer &value, uint64_t range);

  template <typename Integer> void putUExpGolomb(const Integer &value);

  void putSExpGolomb(int64_t value);

  [[nodiscard]] auto byteAligned() const -> bool;
  void byteAlignment();
  void zeroAlign();
  void rbspTrailingBits();

private:
  void writeBits_(uint64_t value, unsigned bits);
  void putUVar_(uint64_t value, uint64_t range);
  void putUExpGolomb_(uint64_t value);

  std::ostream &m_stream;
  uint64_t m_buffer{};
  unsigned m_size{};
};
} // namespace TMIV::Common

#include "Bitstream.hpp"

#endif
