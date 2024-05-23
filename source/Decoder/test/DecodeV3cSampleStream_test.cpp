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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>

#include <TMIV/Decoder/DecodeV3cSampleStream.h>

#include <sstream>

using namespace std::string_literals;

namespace test {
namespace {
auto exampleBytestream(uint8_t unitCount) {
  using TMIV::MivBitstream::SampleStreamNalHeader;
  using TMIV::MivBitstream::SampleStreamNalUnit;
  using TMIV::MivBitstream::V3cParameterSet;
  using TMIV::MivBitstream::V3cUnit;
  using TMIV::MivBitstream::V3cUnitHeader;

  std::ostringstream stream;

  const auto ssvh = SampleStreamNalHeader{0};
  ssvh.encodeTo(stream);

  for (uint8_t i = 0; i < unitCount; ++i) {
    std::ostringstream substream;
    auto vu = V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}.vps_v3c_parameter_set_id(i)};
    vu.encodeTo(substream);
    SampleStreamNalUnit{substream.str()}.encodeTo(stream, ssvh);
  }

  return stream.str();
}
} // namespace
} // namespace test

TEST_CASE("TMIV::Decoder::decodeV3cSampleStream") {
  using TMIV::Decoder::decodeV3cSampleStream;

  SECTION("Empty stream") {
    using TMIV::Common::V3cBitstreamError;

    std::istringstream stream;

    REQUIRE_THROWS_AS(decodeV3cSampleStream(stream), std::runtime_error);
  }

  SECTION("Example") {
    const auto unitCount = GENERATE(uint8_t{}, uint8_t{1}, uint8_t{7});
    CAPTURE(unitCount);

    std::istringstream stream{test::exampleBytestream(unitCount)};

    auto unitAtTest = decodeV3cSampleStream(stream);

    for (uint8_t i = 0; i < unitCount; ++i) {
      const auto actual = unitAtTest();
      REQUIRE(actual);
      CHECK(actual->v3c_unit_payload().v3c_parameter_set().vps_v3c_parameter_set_id() == i);
    }

    CHECK_FALSE(unitAtTest());
  }
}
