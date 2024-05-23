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
#include <catch2/matchers/catch_matchers_string.hpp>

#include <TMIV/Multiplexer/Multiplexer.h>

using namespace std::string_literals;

using Catch::Matchers::ContainsSubstring;
using TMIV::Common::Source;
using TMIV::Common::sourceFromIteratorPair;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::Multiplexer::CodedVideoSequenceSourceFactory;
using VUT = TMIV::MivBitstream::VuhUnitType;

namespace test {
namespace {
using TMIV::MivBitstream::AiAttributeTypeId;
using TMIV::MivBitstream::AtlasId;

constexpr auto vuh_v3c_parameter_set_id = 7;
constexpr auto vuh_atlas_id = AtlasId{4};
constexpr auto ai_attribute_count = 1;
constexpr auto vuh_attribute_index = 0;
constexpr auto ai_attribute_type_id = AiAttributeTypeId::ATTR_REFLECTANCE;
const auto codedVideoSequence = std::vector{"[Some]"s, "[video]"s};
const auto codedVideoSequenceSize = 4 + 6 + 4 + 7;

[[nodiscard]] auto ssnh() noexcept {
  using TMIV::MivBitstream::SampleStreamNalHeader;

  return SampleStreamNalHeader{1};
}

[[nodiscard]] auto asb(size_t irapCount) {
  using TMIV::Common::V3cBitstreamError;
  using TMIV::MivBitstream::AtlasSubBitstream;
  using TMIV::MivBitstream::NalUnit;
  using TMIV::MivBitstream::NalUnitHeader;
  using NUT = TMIV::MivBitstream::NalUnitType;

  auto result = AtlasSubBitstream{ssnh()};

  const auto nalCafIdr = NalUnitHeader{NUT::NAL_CAF_IDR, 0, 1};

  while (irapCount-- > 0) {
    result.nal_units().emplace_back(nalCafIdr, ""s);
  }
  return result;
}

[[nodiscard]] auto vpsithoutVideo() noexcept {
  return V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}};
}

[[nodiscard]] auto vpsWithVideo() {
  auto vps = V3cParameterSet{};
  vps.vps_v3c_parameter_set_id(vuh_v3c_parameter_set_id)
      .vps_atlas_id(0, vuh_atlas_id)
      .vps_attribute_video_present_flag(vuh_atlas_id, true)
      .attribute_information(vuh_atlas_id)
      .ai_attribute_count(ai_attribute_count)
      .ai_attribute_type_id(vuh_attribute_index, ai_attribute_type_id);
  return V3cUnit{V3cUnitHeader::vps(), vps};
}

[[nodiscard]] auto cad(size_t irapCount) noexcept {
  return V3cUnit{V3cUnitHeader::cad(vuh_v3c_parameter_set_id), asb(irapCount)};
}

[[nodiscard]] auto ad() noexcept {
  return V3cUnit{V3cUnitHeader::ad(vuh_v3c_parameter_set_id, vuh_atlas_id), asb(0)};
}

[[nodiscard]] auto vsb() noexcept {
  using TMIV::MivBitstream::VideoSubBitstream;

  return VideoSubBitstream{"[video]"};
}

[[nodiscard]] auto ovd() noexcept {
  using TMIV::MivBitstream::VideoSubBitstream;

  return V3cUnit{V3cUnitHeader::ovd(vuh_v3c_parameter_set_id, vuh_atlas_id), vsb()};
}

[[nodiscard]] auto gvd() noexcept {
  using TMIV::MivBitstream::VideoSubBitstream;

  return V3cUnit{V3cUnitHeader::gvd(vuh_v3c_parameter_set_id, vuh_atlas_id), VideoSubBitstream{}};
}

[[nodiscard]] auto avd() noexcept {
  using TMIV::MivBitstream::VideoSubBitstream;

  return V3cUnit{V3cUnitHeader::avd(vuh_v3c_parameter_set_id, vuh_atlas_id, vuh_attribute_index),
                 vsb()};
}

[[nodiscard]] auto pvd() noexcept {
  using TMIV::MivBitstream::VideoSubBitstream;

  return V3cUnit{V3cUnitHeader::pvd(vuh_v3c_parameter_set_id, vuh_atlas_id), vsb()};
}

[[nodiscard]] auto unreachableSourceFactory() noexcept {
  return [](const V3cParameterSet &vps, V3cUnitHeader vuh,
            AiAttributeTypeId attrTypeId) -> Source<std::vector<std::string>> {
    CHECK(vps == vpsWithVideo().v3c_unit_payload().v3c_parameter_set());
    CHECK(vuh == avd().v3c_unit_header());
    CHECK(attrTypeId == ai_attribute_type_id);
    return []() -> std::optional<std::vector<std::string>> { UNREACHABLE; };
  };
}

[[nodiscard]] auto uniformSourceFactory(size_t n) noexcept -> CodedVideoSequenceSourceFactory {
  using TMIV::Common::uniformSource;

  return [n]([[maybe_unused]] auto &&...args) {
    return uniformSource<std::vector<std::string>>(n, codedVideoSequence);
  };
}
} // namespace
} // namespace test

TEST_CASE("TMIV::Multiplexer::multiplex") {
  using TMIV::Multiplexer::multiplex;

  SECTION("A disengaged source gives an empty source") {
    auto unitAtTest = multiplex(nullptr, nullptr);
    CHECK_FALSE(unitAtTest());
  }

  SECTION("An empty source gives an empty source") {
    using TMIV::Common::emptySource;

    auto unitAtTest = multiplex(emptySource<V3cUnit>(), nullptr);
    CHECK_FALSE(unitAtTest());
  }

  SECTION("Most V3C units are blindly copied") {
    using TMIV::Common::at;

    const auto data = std::array{test::vpsWithVideo(), test::ad(), test::ovd(), test::gvd(),
                                 test::avd(),          test::ad(), test::pvd()};

    auto unitAtTest = multiplex(sourceFromIteratorPair(data.cbegin(), data.cend()),
                                test::unreachableSourceFactory());

    for (size_t i = 0; i < data.size(); ++i) {
      CAPTURE(i);
      auto actual = unitAtTest();
      REQUIRE(actual);
      CHECK(*actual == at(data, i));
    }

    CHECK_FALSE(unitAtTest());
  }

  SECTION("The first unit has to be a VPS") {
    const auto firstUnit =
        GENERATE(test::ad(), test::cad(0), test::ovd(), test::gvd(), test::pvd(), test::avd());
    const auto data = std::array{firstUnit};

    auto unitAtTest = multiplex(sourceFromIteratorPair(data.cbegin(), data.cend()), nullptr);

    REQUIRE_THROWS_WITH(unitAtTest(), ContainsSubstring("V3C_VPS"));
  }

  SECTION("MIV requires at least one video sub-bitstream") {
    const auto data = std::array{test::vpsithoutVideo()};

    auto unitAtTest = multiplex(sourceFromIteratorPair(data.cbegin(), data.cend()), nullptr);

    REQUIRE_THROWS_WITH(unitAtTest(),
                        ContainsSubstring("MIV requires at least one video sub-bitstream"));
  }

  SECTION("For each IRAP in the CAD, a coded video sequence is copied to a new V3C unit of that "
          "video component") {
    const size_t cadCount = GENERATE(0, 1, 3);
    const size_t irapCount = GENERATE(0, 1, 3);
    CAPTURE(cadCount, irapCount);

    auto data = std::vector{test::vpsWithVideo()};
    for (size_t i = 0; i < cadCount; ++i) {
      data.push_back(test::cad(irapCount));
    }

    auto unitAtTest = multiplex(sourceFromIteratorPair(data.cbegin(), data.cend()),
                                test::uniformSourceFactory(irapCount * cadCount));

    CHECK(unitAtTest() == std::optional{test::vpsWithVideo()});

    for (size_t i = 0; i < cadCount; ++i) {
      CHECK(unitAtTest() == test::cad(irapCount));

      auto actual = unitAtTest();
      REQUIRE(actual);
      CHECK(actual->v3c_unit_payload().video_sub_bitstream().data().size() ==
            irapCount * test::codedVideoSequenceSize);
    }
  }
}