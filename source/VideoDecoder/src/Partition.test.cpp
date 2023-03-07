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

#include <TMIV/VideoDecoder/Partition.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Formatters.h>

namespace test {
namespace {
using TMIV::Common::emptySource;
using TMIV::Common::OutputBitstream;
using TMIV::Common::Source;

enum class Codec { HEVC, VVC };

template <Codec codec> struct NalUnitHeader {
  static constexpr auto forbidden_zero_bit = false;
  static constexpr auto nuh_reserved_zero_bit = false;
  static constexpr auto nal_layer_id = 0;

  uint8_t nal_unit_type{};
  uint8_t nah_temporal_id_plus1{1};

  auto encodeTo(std::ostream &stream) const {
    OutputBitstream bitstream{stream};

    if constexpr (codec == Codec::HEVC) {
      bitstream.putFlag(forbidden_zero_bit);
      bitstream.writeBits(nal_unit_type, 6);
      bitstream.writeBits(nal_layer_id, 6);
      bitstream.writeBits(nah_temporal_id_plus1, 3);
    } else {
      bitstream.putFlag(forbidden_zero_bit);
      bitstream.putFlag(nuh_reserved_zero_bit);
      bitstream.writeBits(nal_layer_id, 6);
      bitstream.writeBits(nal_unit_type, 5);
      bitstream.writeBits(nah_temporal_id_plus1, 3);
    }
  }
};

enum class NUT { parameterSet, nonIrapVcl, irapVcl, nonIrapPrefix };

template <Codec codec> auto generateNalUnit(NUT nut) -> std::string {
  std::ostringstream stream;

  NalUnitHeader<codec> nuh;

  if constexpr (codec == Codec::HEVC) {
    nuh.nal_unit_type = [nut]() -> uint8_t {
      switch (nut) {
      case NUT::parameterSet:
        return 33; // SPS_NUT
      case NUT::nonIrapVcl:
        return 2; // TSA_N
      case NUT::irapVcl:
        return 21; // CRA_NUT
      case NUT::nonIrapPrefix:
        return 39; // PREFIX_SEI_NUT
      }
      UNREACHABLE;
    }();
  } else if constexpr (codec == Codec::VVC) {
    nuh.nal_unit_type = [nut]() -> uint8_t {
      switch (nut) {
      case NUT::parameterSet:
        return 15; // SPS_NUT
      case NUT::nonIrapVcl:
        return 0; // TRAIL_NUT
      case NUT::irapVcl:
        return 8; // IDR_N_LP
      case NUT::nonIrapPrefix:
        return 23; // PREFIX_SEI_NUT
      }
      UNREACHABLE;
    }();
  }

  nuh.nah_temporal_id_plus1 = [nut]() -> uint8_t {
    switch (nut) {
    case NUT::parameterSet:
    case NUT::irapVcl:
      return 1;
    case NUT::nonIrapVcl:
    case NUT::nonIrapPrefix:
      return 2; // PREFIX_SEI_NUT
    }
    UNREACHABLE;
  }();

  nuh.encodeTo(stream);

  // Add a string that is visible in the debugger
  fmt::print(stream, "[Payload of the {} {} NAL unit]\n", static_cast<int32_t>(codec),
             static_cast<int32_t>(nut));

  return stream.str();
}

using CvsPattern = std::vector<NUT>;
const auto cvsPatternA = CvsPattern{NUT::irapVcl};
const auto cvsPatternB = CvsPattern{NUT::irapVcl, NUT::nonIrapVcl, NUT::nonIrapVcl};
const auto cvsPatternC =
    CvsPattern{NUT::parameterSet, NUT::nonIrapPrefix, NUT::irapVcl,       NUT::nonIrapPrefix,
               NUT::nonIrapVcl,   NUT::nonIrapPrefix, NUT::nonIrapPrefix, NUT::nonIrapVcl};

using BitstreamPattern = std::vector<CvsPattern>;
const auto bitstreamPatternA = BitstreamPattern{cvsPatternA};
const auto bitstreamPatternB = BitstreamPattern{cvsPatternA, cvsPatternB, cvsPatternC, cvsPatternA};
const auto bitstreamPatternC = BitstreamPattern{cvsPatternC, cvsPatternB, cvsPatternA, cvsPatternC};
const auto bitstreamPatternD =
    BitstreamPattern{cvsPatternA, cvsPatternA, cvsPatternB, cvsPatternB, cvsPatternC, cvsPatternC};

template <Codec codec> auto cvs(const CvsPattern &cvsPattern) -> std::vector<std::string> {
  auto units = std::vector<std::string>();

  for (auto nut : cvsPattern) {
    units.push_back(generateNalUnit<codec>(nut));
  }
  return units;
}

template <Codec codec>
auto inputSource(const BitstreamPattern &bitstreamPattern) -> Source<std::string> {
  auto units = std::vector<std::string>();

  for (const auto &cvsPattern : bitstreamPattern) {
    auto cvsUnits = cvs<codec>(cvsPattern);
    units.insert(units.end(), cvsUnits.cbegin(), cvsUnits.cend());
  }

  return [units = std::move(units), i = size_t{}]() mutable -> std::optional<std::string> {
    if (i < units.size()) {
      return units[i++];
    }
    return std::nullopt;
  };
}

template <Codec codec>
auto referenceSource(const BitstreamPattern &bitstreamPattern) -> Source<std::vector<std::string>> {
  return [pattern = bitstreamPattern,
          i = size_t{}]() mutable -> std::optional<std::vector<std::string>> {
    if (i < pattern.size()) {
      return cvs<codec>(pattern[i++]);
    }
    return std::nullopt;
  };
}

template <typename FunctionAtTest> void testUnavailable(FunctionAtTest &&functionAtTest) {
  const auto unitSource = emptySource<std::string>();
  const auto unitAtTest = functionAtTest(unitSource);
  REQUIRE_FALSE(unitAtTest);
}

template <typename FunctionAtTest> void testEmptySource(FunctionAtTest &&functionAtTest) {
  const auto unitSource = emptySource<std::string>();
  const auto unitAtTest = functionAtTest(unitSource);
  REQUIRE(unitAtTest);
  REQUIRE_FALSE(unitAtTest());
}

template <Codec codec, typename FunctionAtTest>
void testBitstreamPatterns(FunctionAtTest &&functionAtTest) {
  const auto pattern =
      GENERATE(bitstreamPatternA, bitstreamPatternB, bitstreamPatternC, bitstreamPatternD);

  auto unitAtTest = functionAtTest(inputSource<codec>(pattern));
  auto referenceSource_ = referenceSource<codec>(pattern);

  while (const auto reference = referenceSource_()) {
    const auto actual = unitAtTest();
    REQUIRE(actual);
    CHECK(*actual == *reference);
  }

  CHECK_FALSE(unitAtTest());
}
} // namespace
} // namespace test

TEST_CASE("VideoDecoder::partitionAvcProgressiveHigh") {
  using TMIV::VideoDecoder::partitionAvcProgressiveHigh;

  SECTION("Unavailable") { test::testUnavailable(partitionAvcProgressiveHigh); }
}

TEST_CASE("VideoDecoder::partitionHevcMain10") {
  using TMIV::VideoDecoder::partitionHevcMain10;

#if HAVE_HM
  SECTION("Empty source") { test::testEmptySource(partitionHevcMain10); }
  SECTION("Bitstream patterns") {
    test::testBitstreamPatterns<test::Codec::HEVC>(partitionHevcMain10);
  }
#else
  SECTION("Unavailable") { test::testUnavailable(partitionHevcMain10); }
#endif
}

TEST_CASE("VideoDecoder::partitionHevc444") {
  using TMIV::VideoDecoder::partitionHevc444;

#if HAVE_HM
  SECTION("Empty source") { test::testEmptySource(partitionHevc444); }
  SECTION("Bitstream patterns") {
    test::testBitstreamPatterns<test::Codec::HEVC>(partitionHevc444);
  }
#else
  SECTION("Unavailable") { test::testUnavailable(partitionHevc444); }
#endif
}

TEST_CASE("VideoDecoder::partitionVvcMain10") {
  using TMIV::VideoDecoder::partitionVvcMain10;

#if HAVE_VVDEC
  SECTION("Empty source") { test::testEmptySource(partitionVvcMain10); }
  SECTION("Bitstream patterns") {
    test::testBitstreamPatterns<test::Codec::VVC>(partitionVvcMain10);
  }
#else
  SECTION("Unavailable") { test::testUnavailable(partitionVvcMain10); }
#endif
}
