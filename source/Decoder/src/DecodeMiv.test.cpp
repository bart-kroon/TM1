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

#include <TMIV/Decoder/DecodeMiv.h>

#include <TMIV/MivBitstream/Formatters.h>

#include "FakeChecker.h"

using TMIV::Common::DecodedFrame;
using TMIV::Common::emptySource;
using TMIV::Common::Source;
using TMIV::Common::sourceFromIteratorPair;
using TMIV::Decoder::AtlasAccessUnit;
using TMIV::Decoder::CommonAtlasAccessUnit;
using TMIV::MivBitstream::AtlasSubBitstream;
using TMIV::MivBitstream::PtlProfileReconstructionIdc;
using TMIV::MivBitstream::PtlProfileToolsetIdc;
using TMIV::MivBitstream::SampleStreamNalHeader;
using TMIV::MivBitstream::V3cParameterSet;
using TMIV::MivBitstream::V3cUnit;
using TMIV::MivBitstream::V3cUnitHeader;
using TMIV::MivBitstream::VideoSubBitstream;
using TMIV::PtlChecker::SharedChecker;

namespace test {
namespace {
using TMIV::Common::Vec2i;
using TMIV::MivBitstream::AtduPatchMode;
using TMIV::MivBitstream::AthType;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::CommonAtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::NalUnit;
using TMIV::MivBitstream::NalUnitHeader;
using TMIV::MivBitstream::NalUnitType;

template <typename A, typename B, typename... Args> auto unreachableFactory() {
  return []([[maybe_unused]] Source<A> source, [[maybe_unused]] Args... args) -> Source<B> {
    UNREACHABLE;
  };
}

auto unreachableVideoDecoderFactory() {
  return unreachableFactory<VideoSubBitstream, DecodedFrame, const V3cParameterSet &,
                            V3cUnitHeader>();
}

auto unreachableCommonAtlasDecoderFactory() {
  return unreachableFactory<AtlasSubBitstream, CommonAtlasAccessUnit, const V3cParameterSet &>();
}

auto unreachableAtlasDecoderFactory() {
  return unreachableFactory<AtlasSubBitstream, AtlasAccessUnit, const V3cParameterSet &,
                            V3cUnitHeader>();
}

template <typename I, typename A, typename B, typename... Args>
auto decoderFactoryFromIteratorPair(I first, I last) {
  return
      [first, last]([[maybe_unused]] Source<A> source, [[maybe_unused]] Args... args) -> Source<B> {
        return sourceFromIteratorPair(first, last);
      };
}

template <typename I> auto videoDecoderFactoryFromIteratorPair(I first, I last) {
  return decoderFactoryFromIteratorPair<I, VideoSubBitstream, DecodedFrame, const V3cParameterSet &,
                                        V3cUnitHeader>(first, last);
}

template <typename I> auto commonAtlasDecoderFactoryFromIteratorPair(I first, I last) {
  return decoderFactoryFromIteratorPair<I, AtlasSubBitstream, CommonAtlasAccessUnit,
                                        const V3cParameterSet &>(first, last);
}

template <typename I> auto atlasDecoderFactoryFromIteratorPair(I first, I last) {
  return decoderFactoryFromIteratorPair<I, AtlasSubBitstream, AtlasAccessUnit,
                                        const V3cParameterSet &, V3cUnitHeader>(first, last);
}

constexpr auto size = Vec2i{64, 32};

auto minimalVps() {
  auto vps = V3cParameterSet{};
  vps.profile_tier_level()
      .ptl_profile_toolset_idc(PtlProfileToolsetIdc::MIV_Main)
      .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained);
  vps.vps_miv_extension() = {};
  vps.vps_frame_width({}, size.x()).vps_frame_height({}, size.y());
  vps.vps_geometry_video_present_flag({}, true).geometry_information({}) = {};
  vps.calculateExtensionLengths();
  return vps;
}

auto videoFrame(bool irap) {
  using TMIV::Common::Frame;

  return DecodedFrame{Frame<>::yuv420(test::size, 8), irap};
}

auto commonAtlasFrame(int32_t foc) {
  using TMIV::Decoder::CommonAtlasAccessUnit;
  using TMIV::MivBitstream::VuiParameters;

  auto au = CommonAtlasAccessUnit{};
  au.foc = foc;
  au.casps.casps_miv_extension().vui_parameters(VuiParameters{}.vui_unit_in_metres_flag(true));
  return au;
}

auto atlasFrame(int32_t foc, size_t atlSize) {
  using TMIV::Decoder::AtlasAccessUnit;

  auto au = AtlasAccessUnit{};
  au.atlV.resize(atlSize);

  for (auto &atl : au.atlV) {
    atl.atlas_tile_header().ath_type(AthType::I_TILE);
    atl.atlas_tile_data_unit().atdu_patch_mode(0, AtduPatchMode::I_END);
  }
  au.foc = foc;
  return au;
}

struct FramePattern {
  bool videoIrap{};
  int32_t commonAtlasFoc{};
  int32_t atlasFoc{};
};

using Pattern = std::vector<FramePattern>;

auto operator<<(std::ostream &stream, FramePattern x) -> std::ostream & {
  fmt::print(stream, "(v:{}, cad:{}, ad:{})\n           ", x.videoIrap, x.commonAtlasFoc,
             x.atlasFoc);
  return stream;
}

auto videoFrameCollection(const Pattern &pattern) {
  auto result = std::vector<DecodedFrame>{};
  std::transform(pattern.cbegin(), pattern.cend(), std::back_inserter(result),
                 [](const auto &x) { return videoFrame(x.videoIrap); });
  return result;
}

auto commonAtlasFrameCollection(const Pattern &pattern) {
  using TMIV::Decoder::CommonAtlasAccessUnit;

  auto result = std::vector<CommonAtlasAccessUnit>{};
  for (const auto &x : pattern) {
    if (0 <= x.commonAtlasFoc) {
      result.push_back(commonAtlasFrame(x.commonAtlasFoc));
    }
  }
  return result;
}

auto atlasFrameCollection(const Pattern &pattern) {
  using TMIV::Decoder::AtlasAccessUnit;

  auto result = std::vector<AtlasAccessUnit>{};
  for (const auto &x : pattern) {
    if (0 <= x.atlasFoc) {
      result.push_back(atlasFrame(x.atlasFoc, 1));
    }
  }
  return result;
}

using TMIV::Decoder::ErrorCode;

auto code(ErrorCode code) {
  using Catch::Matchers::Equals;
  using TMIV::Decoder::errorStringFor;

  return Equals(errorStringFor(code));
}
} // namespace
} // namespace test

TEST_CASE("TMIV::Decoder::decodeMiv (1)") {
  using test::code;
  using TMIV::Decoder::decodeMiv;
  using E = TMIV::Decoder::ErrorCode;

  auto checker = std::make_shared<test::FakeChecker>();

  SECTION("Empty source transforms to empty source") {
    using TMIV::Common::emptySource;

    auto unit = decodeMiv(emptySource<V3cUnit>(), test::unreachableVideoDecoderFactory(), checker,
                          test::unreachableCommonAtlasDecoderFactory(),
                          test::unreachableAtlasDecoderFactory());

    CHECK_FALSE(unit());
    CHECK(checker->checkVuh_callCount == 0);
  }

  SECTION("The first V3C unit has to be a VPS (for this decoder)") {
    const auto data =
        std::array{V3cUnit{V3cUnitHeader::ad(0, {}), AtlasSubBitstream{SampleStreamNalHeader{2}}},
                   V3cUnit{V3cUnitHeader::vps(), V3cParameterSet{}}};

    auto unit = decodeMiv(sourceFromIteratorPair(data.cbegin(), data.cend()),
                          test::unreachableVideoDecoderFactory(), checker,
                          test::unreachableCommonAtlasDecoderFactory(),
                          test::unreachableAtlasDecoderFactory());
    REQUIRE_THROWS_WITH(unit(), code(E::expected_vps));
    CHECK(checker->checkVuh_callCount == 0);
  }

  SECTION("Check capabilities") {
    SECTION("The VPS needs to have the MIV extension enabled") {
      const auto vps = []() { return V3cParameterSet{}.vps_frame_width({}, 7); }();
      REQUIRE(vps != V3cParameterSet{});

      const auto data = std::array{V3cUnit{V3cUnitHeader::vps(), vps}};

      auto unit = decodeMiv(sourceFromIteratorPair(data.cbegin(), data.cend()),
                            test::unreachableVideoDecoderFactory(), checker,
                            test::unreachableCommonAtlasDecoderFactory(),
                            test::unreachableAtlasDecoderFactory());

      REQUIRE_THROWS_WITH(unit(), code(E::expected_miv_extension));
      REQUIRE(checker->checkVuh_callCount == 1);
      CHECK(checker->lastVuh == V3cUnitHeader::vps());
      REQUIRE(checker->checkAndActivateVps_callCount == 1);
      CHECK(checker->activeVps == vps);
    }

    SECTION("Check capabilities: decoding a frame w/o any video components is not possible") {
      const auto vps = []() {
        auto result = V3cParameterSet{};
        result.profile_tier_level()
            .ptl_profile_toolset_idc(PtlProfileToolsetIdc::MIV_Main)
            .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Rec_Unconstrained);
        result.vps_miv_extension() = {};
        result.calculateExtensionLengths();
        return result;
      }();

      const auto data = std::array{V3cUnit{V3cUnitHeader::vps(), vps}};

      auto unit = decodeMiv(sourceFromIteratorPair(data.cbegin(), data.cend()),
                            test::unreachableVideoDecoderFactory(), checker,
                            test::unreachableCommonAtlasDecoderFactory(),
                            test::unreachableAtlasDecoderFactory());

      REQUIRE_THROWS_WITH(unit(), code(E::expected_video));
      REQUIRE(checker->checkVuh_callCount == 1);
      CHECK(checker->lastVuh == V3cUnitHeader::vps());
      REQUIRE(checker->checkAndActivateVps_callCount == 1);
      CHECK(checker->activeVps == vps);
    }
  }

  SECTION("Minimal example to decode a MIV IRAP AU") {
    const auto vps = test::minimalVps();

    const auto v3cUnitData = std::array{V3cUnit{V3cUnitHeader::vps(), vps}};
    const auto videoData = std::array{test::videoFrame(true)};
    const auto commonAtlasData = std::array{test::commonAtlasFrame(0)};
    const auto atlasData = std::array{test::atlasFrame(0, 1)};

    auto unit = decodeMiv(
        sourceFromIteratorPair(v3cUnitData.cbegin(), v3cUnitData.cend()),
        test::videoDecoderFactoryFromIteratorPair(videoData.cbegin(), videoData.cend()), checker,
        test::commonAtlasDecoderFactoryFromIteratorPair(commonAtlasData.cbegin(),
                                                        commonAtlasData.cend()),
        test::atlasDecoderFactoryFromIteratorPair(atlasData.cbegin(), atlasData.cend()));

    const auto frame = unit();

    REQUIRE(frame.has_value());

    CHECK(checker->checkVuh_callCount == 1);
    CHECK(checker->checkAndActivateVps_callCount == 1);
    CHECK(checker->checkVideoFrame_callCount == 1);
    CHECK(checker->checkV3cFrame_callCount == 1);

    // PTL checking of (common) atlas sub-bitstreams is delegated, not this unit:
    CHECK(checker->checkAndActivateNuh_callCount == 0);
    CHECK(checker->checkAndActivateAsps_callCount == 0);
    CHECK(checker->checkAfps_callCount == 0);
    CHECK(checker->checkAtl_callCount == 0);
    CHECK(checker->checkCaf_callCount == 0);
  }
}

namespace test {
namespace {
void runPattern(bool good, const Pattern &pattern) {
  using TMIV::Decoder::decodeMiv;

  CAPTURE(pattern);

  const auto vps = test::minimalVps();
  const auto v3cUnitData = std::array{V3cUnit{V3cUnitHeader::vps(), vps}};
  const auto videoData = videoFrameCollection(pattern);
  const auto commonAtlasData = commonAtlasFrameCollection(pattern);
  const auto atlasData = atlasFrameCollection(pattern);

  const auto unit = decodeMiv(
      sourceFromIteratorPair(v3cUnitData.cbegin(), v3cUnitData.cend()),
      videoDecoderFactoryFromIteratorPair(videoData.cbegin(), videoData.cend()),
      std::make_shared<test::FakeChecker>(),
      commonAtlasDecoderFactoryFromIteratorPair(commonAtlasData.cbegin(), commonAtlasData.cend()),
      atlasDecoderFactoryFromIteratorPair(atlasData.cbegin(), atlasData.cend()));

  for (size_t frameIdx = 0; frameIdx < videoData.size(); ++frameIdx) {
    CAPTURE(frameIdx);
    const auto frame = unit();
    if (good) {
      REQUIRE(frame);
    }
  }

  auto end = unit();
  if (good) {
    REQUIRE_FALSE(end);
  }
}
} // namespace
} // namespace test

TEST_CASE("TMIV::Decoder::decodeMiv, sub-bitstream synchronization") {
  using E = TMIV::Decoder::ErrorCode;
  using test::code;

  SECTION("Decode multiple MIV AU's using some valid patterns") {
    const auto pattern = GENERATE(
        test::Pattern{{true, 0, 0}}, // This one should match with the test case directly above
        test::Pattern{{true, 0, 0}, {false, -1, -1}, {false, -1, -1}},
        test::Pattern{{true, 0, 0}, {false, -1, -1}, {false, 1, 1}, {false, -1, -1}},
        test::Pattern{{true, 0, 0}, {true, 0, 0}, {false, 1, 1}, {true, 0, 0}, {false, -1, -1}},
        test::Pattern{{true, 0, 0}, {false, -1, 1}, {false, 2, 2}, {true, 0, 0}, {false, 1, -1}});
    test::runPattern(true, pattern);
  }

  SECTION("Expected atlas AU to be an IRAP") {
    CHECK_THROWS_WITH(
        test::runPattern(false, {{true, 0, 0}, {false, -1, 2}, {false, 2, 1}, {true, 0, 0}}),
        code(E::expected_atlas_to_be_irap));
  }

  SECTION("Expected common atlas AU to be an IRAP") {
    CHECK_THROWS_WITH(
        test::runPattern(false, {{true, 0, 0}, {false, 2, -1}, {false, 1, 2}, {true, 0, 0}}),
        code(E::expected_common_atlas_to_be_irap));
  }

  SECTION("Expected common atlas AU to be an IRAP") {
    CHECK_THROWS_WITH(test::runPattern(false, {{true, 0, 0}, {false, 1, 1}, {false, 1, 2}}),
                      code(E::misaligned_common_atlas_foc));
  }

  SECTION("Expected atlas AU to be an IRAP") {
    CHECK_THROWS_WITH(test::runPattern(false, {{true, 0, 0}, {false, 1, 1}, {false, 2, 1}}),
                      code(E::misaligned_atlas_foc));
  }

  SECTION("Common atlas IRAP is missing") {
    CHECK_THROWS_WITH(test::runPattern(false, {{true, 0, 0}, {true, -1, 0}}),
                      code(E::missing_common_atlas_irap));
  }

  SECTION("Atlas IRAP is missing") {
    CHECK_THROWS_WITH(test::runPattern(false, {{true, 0, 0}, {true, 0, -1}}),
                      code(E::missing_atlas_irap));
  }

  SECTION("Misaligned IRAP AU's in the video sub-bitstreams") {
    CHECK_THROWS_WITH(test::runPattern(false, {{false, 0, 0}}), code(E::misaligned_video_irap));
  }
}
