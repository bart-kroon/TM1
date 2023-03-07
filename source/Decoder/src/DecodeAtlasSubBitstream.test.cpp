/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/Decoder/DecodeAtlasSubBitstream.h>

using TMIV::MivBitstream::AtlasSubBitstream;

namespace test {
namespace {
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitType;

auto asb(size_t unitCount) {
  using TMIV::MivBitstream::NalUnitHeader;
  using TMIV::MivBitstream::SampleStreamNalHeader;

  auto result = AtlasSubBitstream{SampleStreamNalHeader{2}};

  if (0 < unitCount) {
    result.nal_units().emplace_back(NalUnitHeader{NalUnitType::NAL_AUD, 0, 1}, "payload");
  }
  for (size_t i = 1; i < unitCount; ++i) {
    result.nal_units().emplace_back(NalUnitHeader{NalUnitType::NAL_FD, 0, 1}, "payload");
  }

  return result;
}
} // namespace
} // namespace test

TEST_CASE("TMIV::Decoder::decodeAtlasSubBitstream") {
  using TMIV::Decoder::decodeAtlasSubBitstream;

  SECTION("Disengaged source") {
    auto unit = decodeAtlasSubBitstream(nullptr);
    REQUIRE_FALSE(unit());
  }

  SECTION("Empty source") {
    using TMIV::Common::emptySource;

    auto unit = decodeAtlasSubBitstream(emptySource<AtlasSubBitstream>());
    REQUIRE_FALSE(unit());
  }

  SECTION("Some ASB's with some NAL units") {
    using TMIV::Common::uniformSource;

    const size_t unitCount = GENERATE(0, 1, 4);
    const size_t asbCount = GENERATE(0, 1, 3);

    const auto asb = test::asb(unitCount);
    auto unitAtTest = decodeAtlasSubBitstream(uniformSource(asbCount, asb));

    for (size_t i = 0; i < unitCount * asbCount; ++i) {
      auto nalUnit = unitAtTest();
      REQUIRE(nalUnit);
      CHECK(*nalUnit == asb.nal_units()[i % unitCount]);
    }
  }
}
