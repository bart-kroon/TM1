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

#include <TMIV/Multiplexer/EncodeV3cSampleStream.h>

namespace test {
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;

const auto v3cUnit = V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}};
} // namespace test

TEST_CASE("TMIV::Multiplexer::encodeV3cSampleStream") {
  using TMIV::Common::uniformSource;
  using TMIV::MivBitstream::SampleStreamV3cHeader;
  using TMIV::Multiplexer::encodeV3cSampleStream;

  const size_t unitCount = GENERATE(0, 1, 4);
  CAPTURE(unitCount);

  SECTION("Provide SSVH to write all source units to the output with minimal work") {
    const auto precisionBytesMinus1 = GENERATE(uint8_t{}, uint8_t{1}, uint8_t{4});
    CAPTURE(precisionBytesMinus1);

    const auto ssvh = SampleStreamV3cHeader{precisionBytesMinus1};
    std::ostringstream stream;
    encodeV3cSampleStream(ssvh, uniformSource(unitCount, test::v3cUnit), stream);

    CHECK(stream.str().size() == 1 + (precisionBytesMinus1 + 19) * unitCount);
  }

  SECTION("Buffer the entire input to choose the optimal value for "
          "ssvh_unit_size_precision_bytes_minus1") {
    std::ostringstream stream;
    encodeV3cSampleStream(uniformSource(unitCount, test::v3cUnit), stream);

    CHECK(stream.str().size() == 1 + 19 * unitCount);
  }
}
