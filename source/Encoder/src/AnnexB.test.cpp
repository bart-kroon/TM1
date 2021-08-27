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

#include <catch2/catch.hpp>

#include <TMIV/Encoder/AnnexB.h>

#include <sstream>

using namespace std::string_view_literals;
using namespace std::string_literals;

namespace {
void writeStartCode(std::ostream &stream, int32_t zeroCount) {
  for (int32_t i = 0; i < zeroCount; ++i) {
    stream.put('\0');
  }
  stream.put('\1');
}

auto exampleBitstream(int32_t zeroCount, bool nextNalUnit, std::string_view payload) {
  std::ostringstream stream;

  writeStartCode(stream, zeroCount);
  stream << payload;

  if (nextNalUnit) {
    writeStartCode(stream, zeroCount);
    stream << payload;
  }

  return stream.str();
}
} // namespace

TEST_CASE("readNalUnitFromAnnexBStreamIntoBuffer") {
  using TMIV::Encoder::readNalUnitFromAnnexBStreamIntoBuffer;

  SECTION("The NAL unit is read into the buffer, replacing the current data") {
    const auto zeroCount = GENERATE(2, 3);
    const auto nextNalUnit = GENERATE(false, true);
    const auto payload = GENERATE("[ NAL unit ]"s, "\0\0\0 Not a start code"s);
    const auto initialBufferSize = GENERATE(0, 5);
    CAPTURE(zeroCount, nextNalUnit, payload, initialBufferSize);

    std::istringstream stream{exampleBitstream(zeroCount, nextNalUnit, payload)};

    auto buffer = std::vector<char>(initialBufferSize, 'b');
    readNalUnitFromAnnexBStreamIntoBuffer(stream, buffer);

    REQUIRE(buffer.size() == payload.size());
    CHECK(std::equal(buffer.cbegin(), buffer.cend(), payload.cbegin()));
    CHECK(stream.good());
    CHECK(stream.tellg() == static_cast<std::streamoff>(payload.size()) + zeroCount + 1);
  }

  SECTION("When the stream is at the end, the buffer is empty") {
    const auto readCount = GENERATE(0, 10);
    CAPTURE(readCount);

    std::istringstream stream{std::string(readCount, 'x')};
    stream.seekg(readCount);

    auto buffer = std::vector<unsigned char>{};
    readNalUnitFromAnnexBStreamIntoBuffer(stream, buffer);

    CHECK(buffer.empty());
    CHECK(stream.eof());
  }

  SECTION("When the stream fails, an exception is thrown") {
    std::istringstream stream;
    stream.get();

    auto buffer = std::vector<signed char>{};
    REQUIRE_THROWS(readNalUnitFromAnnexBStreamIntoBuffer(stream, buffer));
  }

  SECTION("When the start code could not be parsed, an exception is thrown") {
    const auto bitstream = GENERATE("Not a start code"s, "\0\1 is also not a start code"s);
    CAPTURE(bitstream);

    std::istringstream stream{bitstream};

    auto buffer = std::vector<char>{};
    REQUIRE_THROWS(readNalUnitFromAnnexBStreamIntoBuffer(stream, buffer));
  }
}
