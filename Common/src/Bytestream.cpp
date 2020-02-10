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

#include <TMIV/Common/Bytestream.h>

#include <iostream>

using namespace std;

namespace {
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define verify(condition) (void)(!!(condition) || verifyFailed(#condition, __FILE__, __LINE__))

auto verifyFailed(char const *condition, char const *file, int line) -> bool {
  cerr << "Failed to encode/decode byte stream: " << condition << " [" << file << "@" << line
       << endl;
  abort();
  return false;
}
} // namespace

namespace TMIV::Common {
auto readBytes(istream &stream, size_t bytes) -> uint_least64_t {
  verify(bytes <= 8);
  auto result = uint_least64_t{0};
  while (bytes-- > 0) {
    char buffer;
    stream.get(buffer);
    result = (result << 8) | uint8_t(buffer);
  }
  verify(stream.good());
  return result;
}

auto getUint8(istream &stream) -> uint8_t { return uint8_t(readBytes(stream, 1)); }
auto getUint16(istream &stream) -> uint16_t { return uint16_t(readBytes(stream, 2)); }
auto getUint32(istream &stream) -> uint32_t { return uint32_t(readBytes(stream, 4)); }
auto getUint64(istream &stream) -> uint64_t { return uint64_t(readBytes(stream, 8)); }

auto readString(istream &stream, size_t bytes) -> string {
  auto result = string(bytes, '\0');
  stream.read(result.data(), bytes);
  verify(stream.good());
  return result;
}

void writeBytes(ostream &stream, uint_least64_t value, size_t bytes) {
  verify(bytes <= 8);
  if (bytes > 1) {
    writeBytes(stream, value >> 8, bytes - 1);
  }
  if (bytes > 0) {
    stream.put(char(value));
  }
  verify(stream.good());
}

void putUint8(ostream &stream, uint8_t value) { writeBytes(stream, value, 1); }
void putUint16(ostream &stream, uint8_t value) { writeBytes(stream, value, 2); }
void putUint32(ostream &stream, uint8_t value) { writeBytes(stream, value, 4); }
void putUint64(ostream &stream, uint8_t value) { writeBytes(stream, value, 8); }

void writeString(ostream &stream, const string &buffer) {
  stream.write(buffer.data(), buffer.size());
  verify(stream.good());
}
} // namespace TMIV::Common
