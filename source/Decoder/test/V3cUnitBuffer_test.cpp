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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <TMIV/Decoder/V3cUnitBuffer.h>

using TMIV::Decoder::V3cUnitBuffer;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VideoSubBitstream;

using namespace std::string_literals;

TEST_CASE("TMIV::Decoder::V3cUnitBuffer") {
  std::vector<V3cUnit> receivedByOnVps;
  const auto onVps = [&](const V3cUnit &vu) { receivedByOnVps.push_back(vu); };

  SECTION("Empty V3C unit stream") {
    using TMIV::Common::emptySource;

    auto unit = V3cUnitBuffer{emptySource<V3cUnit>(), onVps};
    const auto vuh = GENERATE(V3cUnitHeader::vps(), V3cUnitHeader::ovd(0, {}));
    REQUIRE_FALSE(unit(vuh));
    REQUIRE(receivedByOnVps.empty());
  }

  SECTION("V3C unit stream VPS, OVD") {
    using TMIV::Common::sourceFromIteratorPair;

    const auto data = std::array{V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}},
                                 V3cUnit{V3cUnitHeader::ovd(0, {}), VideoSubBitstream{}}};
    auto unit = V3cUnitBuffer{sourceFromIteratorPair(data.cbegin(), data.cend()), onVps};

    SECTION("Read in order") {
      REQUIRE(unit(V3cUnitHeader::vps()));
      REQUIRE(unit(V3cUnitHeader::ovd(0, {})));
      REQUIRE_FALSE(unit(V3cUnitHeader::ovd(0, {})));
      REQUIRE(receivedByOnVps.empty());
    }

    SECTION("Read out-of-order") {
      REQUIRE(unit(V3cUnitHeader::ovd(0, {})));
      REQUIRE_FALSE(unit(V3cUnitHeader::ovd(0, {})));
      REQUIRE(receivedByOnVps.size() == 1);
    }
  }

  SECTION("V3C unit stream VPS, OVD, GVD, VPS, GVD, OVD") {
    using TMIV::Common::sourceFromIteratorPair;

    static constexpr auto vps = V3cUnitHeader::vps();
    static constexpr auto ovd = V3cUnitHeader::ovd(0, {});
    static constexpr auto gvd = V3cUnitHeader::gvd(0, {}, 0, false);

    const auto data =
        std::array{V3cUnit{vps, V3cParameterSet{}},   V3cUnit{ovd, VideoSubBitstream{}},
                   V3cUnit{gvd, VideoSubBitstream{}}, V3cUnit{vps, V3cParameterSet{}},
                   V3cUnit{ovd, VideoSubBitstream{}}, V3cUnit{gvd, VideoSubBitstream{}}};
    auto unit = V3cUnitBuffer{sourceFromIteratorPair(data.cbegin(), data.cend()), onVps};

    SECTION("Read in bitstream order, incl. 2nd VPS") {
      REQUIRE(unit(vps));
      REQUIRE(unit(ovd));
      REQUIRE(unit(gvd));
      REQUIRE(unit(vps));
      REQUIRE(unit(gvd));
      REQUIRE(unit(ovd));
      REQUIRE_FALSE(unit(gvd));
      REQUIRE(receivedByOnVps.empty());
    }

    SECTION("Read in MIV bitstream order, callback for 2nd VPS") {
      REQUIRE(unit(vps));
      REQUIRE(unit(ovd));
      REQUIRE(unit(gvd));
      REQUIRE(receivedByOnVps.empty());
      REQUIRE(unit(ovd));
      REQUIRE(receivedByOnVps.size() == 1);
      REQUIRE(unit(gvd));
      REQUIRE_FALSE(unit(gvd));
      REQUIRE(receivedByOnVps.size() == 1);
    }

    SECTION("Read in component order, callback for 2nd VPS") {
      REQUIRE(unit(vps));
      REQUIRE(unit(ovd));
      REQUIRE(receivedByOnVps.empty());
      REQUIRE(unit(ovd));
      REQUIRE(receivedByOnVps.size() == 1);
      REQUIRE_FALSE(unit(ovd));
      REQUIRE(unit(gvd));
      REQUIRE(unit(gvd));
      REQUIRE_FALSE(unit(gvd));
      REQUIRE(receivedByOnVps.size() == 1);
    }
  }

  SECTION("The first unit has to be a VPS (for this decoder)") {
    using Catch::Matchers::ContainsSubstring;
    using TMIV::Common::sourceFromIteratorPair;

    static constexpr auto vps = V3cUnitHeader::vps();
    static constexpr auto ovd = V3cUnitHeader::ovd(0, {});

    const auto data =
        std::array{V3cUnit{ovd, VideoSubBitstream{}}, V3cUnit{vps, V3cParameterSet{}}};
    auto unit = V3cUnitBuffer{sourceFromIteratorPair(data.cbegin(), data.cend()), onVps};

    REQUIRE_THROWS_WITH(unit(vps),
                        ContainsSubstring("Expected a VPS but found the following V3C unit"));
  }
}

TEST_CASE("TMIV::Decoder::videoSubBitstreamSource") {
  using TMIV::Common::sourceFromIteratorPair;
  using TMIV::Decoder::V3cUnitBuffer;
  using TMIV::Decoder::videoSubBitstreamSource;
  using TMIV::MivBitstream::V3cParameterSet;
  using TMIV::MivBitstream::V3cUnit;
  using TMIV::MivBitstream::V3cUnitHeader;
  using TMIV::MivBitstream::VideoSubBitstream;

  const auto data =
      std::array{V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}},
                 V3cUnit{V3cUnitHeader::ovd(0, {}), VideoSubBitstream{"first OVD"s}},
                 V3cUnit{V3cUnitHeader::gvd(0, {}), VideoSubBitstream{"first GVD"s}},
                 V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}},
                 V3cUnit{V3cUnitHeader::ovd(0, {}), VideoSubBitstream{"second OVD"s}},
                 V3cUnit{V3cUnitHeader::gvd(0, {}), VideoSubBitstream{"second GVD"s}}};

  auto onVpsCallCount = 0;
  const auto onVps = [&]([[maybe_unused]] const V3cUnit &vu) { ++onVpsCallCount; };

  auto unitBuffer =
      std::make_shared<V3cUnitBuffer>(sourceFromIteratorPair(data.cbegin(), data.cend()), onVps);
  REQUIRE((*unitBuffer)(V3cUnitHeader::vps()));

  SECTION("Pulling an entire sub-bitstream out-of-order") {
    auto unitAtTest = videoSubBitstreamSource(unitBuffer, V3cUnitHeader::gvd(0, {}));

    const auto actual1 = unitAtTest();
    REQUIRE(actual1);
    CHECK(actual1->data() == "first GVD"s);

    const auto actual2 = unitAtTest();
    REQUIRE(actual2);
    CHECK(actual2->data() == "second GVD"s);

    REQUIRE_FALSE(unitAtTest());
    CHECK(onVpsCallCount == 1);
  }

  SECTION("Pulling a non-existent sub-bitstream") {
    auto unitAtTest = videoSubBitstreamSource(unitBuffer, V3cUnitHeader::avd(0, {}, 3));
    REQUIRE_FALSE(unitAtTest());
    CHECK(onVpsCallCount == 1);
  }
}

namespace test {
namespace {
auto asb(size_t unitCount) {
  using TMIV::MivBitstream::NalUnitHeader;
  using TMIV::MivBitstream::NalUnitType;
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

TEST_CASE("TMIV::Decoder::atlasSubBitstreamSource") {
  using TMIV::Common::sourceFromIteratorPair;
  using TMIV::Decoder::atlasSubBitstreamSource;

  const auto data = std::array{V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}},
                               V3cUnit{V3cUnitHeader::cad(0), test::asb(1)},
                               V3cUnit{V3cUnitHeader::ad(0, {}), test::asb(2)},
                               V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}},
                               V3cUnit{V3cUnitHeader::cad(0), test::asb(3)},
                               V3cUnit{V3cUnitHeader::ad(0, {}), test::asb(4)}};

  auto onVpsCallCount = 0;
  const auto onVps = [&]([[maybe_unused]] const V3cUnit &vu) { ++onVpsCallCount; };

  auto unitBuffer =
      std::make_shared<V3cUnitBuffer>(sourceFromIteratorPair(data.cbegin(), data.cend()), onVps);
  REQUIRE((*unitBuffer)(V3cUnitHeader::vps()));

  SECTION("Pulling an entire sub-bitstream out-of-order") {
    auto unitAtTest = atlasSubBitstreamSource(unitBuffer, V3cUnitHeader::cad(0));

    const auto actual1 = unitAtTest();
    REQUIRE(actual1);
    CHECK(actual1->nal_units().size() == 1);

    const auto actual2 = unitAtTest();
    REQUIRE(actual2);
    CHECK(actual2->nal_units().size() == 3);

    REQUIRE_FALSE(unitAtTest());
    CHECK(onVpsCallCount == 1);
  }

  SECTION("Pulling a non-existent sub-bitstream") {
    auto unitAtTest = atlasSubBitstreamSource(unitBuffer, V3cUnitHeader::ad(2, {}));
    REQUIRE_FALSE(unitAtTest());
    CHECK(onVpsCallCount == 1);
  }
}