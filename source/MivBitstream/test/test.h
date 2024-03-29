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

#ifndef TMIV_MIVBITSTREAM_TEST_H
#define TMIV_MIVBITSTREAM_TEST_H

#include <catch2/catch_test_macros.hpp>

#include <TMIV/Common/Bitstream.h>

#include <array>
#include <sstream>

namespace {
template <typename Type, typename... Args>
void byteCodingTest(const Type &reference, int32_t size, Args &&...args) {
  CAPTURE(reference);

  std::stringstream stream;
  reference.encodeTo(stream, args...);
  REQUIRE(size == stream.tellp());

  const auto actual = Type::decodeFrom(stream, std::forward<Args>(args)...);
  CAPTURE(actual);
  REQUIRE(size == stream.tellg());

  REQUIRE((actual == reference));
}

template <typename Type, typename... Args>
auto unitCodingTest(const Type &reference, int32_t size, Args &&...args) -> bool {
  std::stringstream stream;
  REQUIRE(size == static_cast<int32_t>(reference.encodeTo(stream, args...)));
  REQUIRE(size == stream.tellp());

  const auto actual = Type::decodeFrom(stream, std::forward<Args>(args)..., size);
  REQUIRE(size == stream.tellg());

  return actual == reference;
}

template <typename Type, typename... Args>
void bitCodingTest(const Type &reference, int32_t bitsize, Args &&...args) {
  CAPTURE(reference);

  std::stringstream stream;
  TMIV::Common::OutputBitstream obitstream{stream};
  reference.encodeTo(obitstream, args...);
  CHECK(bitsize == obitstream.tellp());
  obitstream.zeroAlign();

  TMIV::Common::InputBitstream ibitstream{stream};
  const auto actual = Type::decodeFrom(ibitstream, std::forward<Args>(args)...);
  CAPTURE(actual);
  REQUIRE(bitsize == ibitstream.tellg());

  REQUIRE((actual == reference));
}

template <typename Type> auto toString(const Type &metadata) -> std::string {
  std::ostringstream stream;
  stream << metadata;
  return stream.str();
}

template <typename Type, typename... Args>
auto toString(const Type &metadata, Args &&...args) -> std::string {
  std::ostringstream stream;
  metadata.printTo(stream, std::forward<Args>(args)...);
  return stream.str();
}
} // namespace

#endif
