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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

using TMIV::Common::SampleValue;
using TMIV::MivBitstream::DepthQuantization;
using TMIV::MivBitstream::DepthTransform;
using TMIV::MivBitstream::OccupancyTransform;
using TMIV::MivBitstream::PatchParams;
using TMIV::MivBitstream::ViewParams;

TEST_CASE("OccupancyTransform") {
  SECTION("Constructor for per-view occupancy threshold signalling (source)") {
    REQUIRE_NOTHROW(OccupancyTransform{ViewParams{}});
  }

  SECTION("Constructor for per-patch occupancy threshold signalling (codec)") {
    REQUIRE_NOTHROW(OccupancyTransform{ViewParams{}, PatchParams{}});
  }

  SECTION("Test for occupancy (codec, full occupancy)") {
    const auto unit = OccupancyTransform{ViewParams{}};

    const auto x =
        GENERATE(uint16_t{}, uint16_t{1}, uint16_t{0xFF}, uint16_t{0xFFF}, uint16_t{0xFFFF});

    REQUIRE(unit.occupant(x));
  }

  SECTION("Test for occupancy (codec, per-view occupancy)") {
    auto vp = ViewParams{};
    vp.dq.dq_depth_occ_threshold_default(4000);

    const auto unit = OccupancyTransform{vp};

    const auto x1 = GENERATE(uint16_t{}, uint16_t{1}, uint16_t{3999});
    REQUIRE(!unit.occupant(x1));

    const auto x2 = GENERATE(uint16_t{4000}, uint16_t{0xFFFF});
    REQUIRE(unit.occupant(x2));
  }

  SECTION("Test for occupancy (codec, per-view occupancy)") {
    auto vp = ViewParams{};
    vp.dq.dq_depth_occ_threshold_default(4000);

    const auto unit = OccupancyTransform{vp, PatchParams{}};

    REQUIRE(!unit.occupant(3999));
    REQUIRE(unit.occupant(4000));
  }

  SECTION("Test for occupancy (codec, per-patch occupancy)") {
    auto vp = ViewParams{};
    vp.dq.dq_depth_occ_threshold_default(4000);

    auto pp = PatchParams{};
    pp.atlasPatchDepthOccThreshold(1000);

    const auto unit = OccupancyTransform{vp, pp};

    const auto x1 = GENERATE(uint16_t{}, uint16_t{1}, uint16_t{999});
    REQUIRE(!unit.occupant(x1));

    const auto x2 = GENERATE(uint16_t{1000}, uint16_t{4000}, uint16_t{0xFFFF});
    REQUIRE(unit.occupant(x2));
  }
}

TEST_CASE("DepthTransform") {
  SECTION("Constructor for per-view depth transform signalling (source)") {
    REQUIRE_THROWS(DepthTransform{DepthQuantization{}, 0});
  }

  SECTION("The constructor checks the normalized disparity range") {
    // NOTE(BK): The test model places conditions on dq_norm_disp_low and dq_norm_disp_high:
    //   * dq_norm_disp_low != dq_norm_disp_high
    //   * 0 < max(dq_norm_disp_low, dq_norm_disp_high)
    //   * finite dq_norm_disp_high
    //   * finite dq_norm_disp_high
    // but the draft specification (ISO/IEC JTC 1/SC 29/WG 4 N 4) is silent on this.

    auto dq1 = DepthQuantization{};
    dq1.dq_norm_disp_low(-1.F);
    dq1.dq_norm_disp_high(-2.F);

    REQUIRE_THROWS(DepthTransform{dq1, 0});

    auto dq2 = DepthQuantization{};
    dq2.dq_norm_disp_low(1.F);
    dq2.dq_norm_disp_high(NAN);

    REQUIRE_THROWS(DepthTransform{dq2, 0});
  }

  SECTION("Constructor for per-view depth transform signalling (codec)") {
    REQUIRE_THROWS(DepthTransform{DepthQuantization{}, PatchParams{}, 0});
  }

  SECTION("Expand a level to normalized disparity [m^-1], per-view depth transform") {
    const auto normDispLow = GENERATE(-2.F, 0.F, 10.F);
    const auto normDispHigh = GENERATE(-1.F, 0.F, 100.F);

    if (normDispLow != normDispHigh && 0 < std::max(normDispLow, normDispHigh)) {
      auto dq = DepthQuantization{};
      dq.dq_norm_disp_low(normDispLow);
      dq.dq_norm_disp_high(normDispHigh);

      const auto bits = GENERATE(1U, 12U);
      constexpr uint16_t lowLevel = 0;
      const auto highLevel = TMIV::Common::maxLevel<uint16_t>(bits);

      const auto unit = DepthTransform{dq, bits};

      const auto minNormDisp = unit.minNormDisp();
      const auto lowDisp = std::max(minNormDisp, normDispLow);
      const auto highDisp = std::max(minNormDisp, normDispHigh);

      REQUIRE(unit.expandNormDisp(lowLevel) == lowDisp);
      if (lowLevel < highLevel) {
        REQUIRE(unit.expandNormDisp(highLevel) == highDisp);
      }
    }
  }

  SECTION("Expand a level to normalized disparity [m^-1], PLS method") {
    const auto normDispLow = GENERATE(-2.F, 0.F, 10.F);
    const auto normDispHigh = GENERATE(-1.F, 0.F, 100.F);

    if (normDispLow != normDispHigh && 0 < std::max(normDispLow, normDispHigh) &&
        normDispLow < normDispHigh) {
      auto dq = DepthQuantization{};
      dq.dq_norm_disp_low(normDispLow);
      dq.dq_norm_disp_high(normDispHigh);

      dq.dq_quantization_law(2);
      dq.dq_pivot_count_minus1(0);
      float normDisp_at_pivot_point =
          normDispLow +
          (normDispHigh - normDispLow) / static_cast<float>(dq.dq_pivot_count_minus1() + 2);
      dq.dq_pivot_norm_disp(0, normDisp_at_pivot_point);

      const auto bits = GENERATE(1U, 12U);

      constexpr uint16_t lowLevel = 0;
      const auto highLevel = TMIV::Common::maxLevel<uint16_t>(bits);
      const uint16_t midLevel =
          lowLevel + (highLevel - lowLevel) / static_cast<uint16_t>(dq.dq_pivot_count_minus1() + 2);
      const uint16_t midLevel_minus1 = static_cast<uint16_t>(
          std::clamp(static_cast<int32_t>(lowLevel), static_cast<int32_t>(midLevel) - 1,
                     static_cast<int32_t>(highLevel)));
      const uint16_t midLevel_plus1 = static_cast<uint16_t>(
          std::clamp(static_cast<int32_t>(lowLevel), static_cast<int32_t>(midLevel) + 1,
                     static_cast<int32_t>(highLevel)));

      const auto unit = DepthTransform{dq, bits};
      const auto lowDisp = unit.expandNormDisp(lowLevel);
      const auto midDisp = unit.expandNormDisp(midLevel);
      const auto highDisp = unit.expandNormDisp(highLevel);

      REQUIRE(unit.expandNormDisp(lowLevel) == lowDisp);

      if (lowLevel < highLevel) {
        REQUIRE(unit.expandNormDisp(lowLevel) == lowDisp);
        REQUIRE(unit.expandNormDisp(midLevel_minus1) <= midDisp);
        REQUIRE(unit.expandNormDisp(midLevel_plus1) >= midDisp);
        REQUIRE(unit.expandNormDisp(highLevel) <= highDisp);
      }
    }
  }

  SECTION("Expand a level to normalized disparity [m^-1], per-patch depth transform") {
    const auto normDispLow = GENERATE(-2.F, 3.F);
    const auto normDispHigh = GENERATE(5.F, 7.F);

    auto dq = DepthQuantization{};
    dq.dq_norm_disp_low(normDispLow);
    dq.dq_norm_disp_high(normDispHigh);

    const auto bits = GENERATE(12U, 16U);

    const auto reference = DepthTransform{dq, bits};

    const auto offsetD = GENERATE(0, 1, 100, 1000);
    const auto rangeD = GENERATE(0, 100, 1000);

    auto pp = PatchParams{};
    pp.atlasPatch3dOffsetD(offsetD);
    pp.atlasPatch3dRangeD(rangeD);

    const auto unit = DepthTransform{dq, pp, bits};

    const auto test = [&](SampleValue d, SampleValue n) {
      REQUIRE(unit.expandNormDisp(d) == reference.expandNormDisp(offsetD + n));
    };

    SECTION("Test patch offset") {
      test(0, 0);

      if (0 < rangeD) {
        test(1, 1);
        test(rangeD / 2, rangeD / 2);
        test(rangeD - 1, rangeD - 1);
        test(rangeD, rangeD);
      }
    }
    SECTION("Test patch range") {
      test(rangeD + 1, rangeD);
      test(std::numeric_limits<SampleValue>::max(), rangeD);
    }
  }

  SECTION("Expand a level to depth [m]") {
    auto dq = DepthQuantization{};
    dq.dq_norm_disp_low(3.F);
    dq.dq_norm_disp_high(7.F);

    auto pp = PatchParams{};
    pp.atlasPatch3dOffsetD(100);
    pp.atlasPatch3dRangeD(2000);

    const auto unit = DepthTransform{dq, pp, 12};

    const auto x = GENERATE(0, 1, 7, 0x100, 0x200, 0x3EE, 0x3FF);

    REQUIRE(unit.expandDepth(x) * unit.expandNormDisp(x) == Catch::Approx(1.));
  }

  SECTION("Expand a frame of 10-bit levels to depth [m]") {
    const auto frame = [] {
      auto x = TMIV::Common::Frame<>::lumaOnly({4, 3}, 10);
      std::iota(x.getPlane(0).begin(), x.getPlane(0).end(), uint16_t{});
      return x;
    }();

    auto dq = DepthQuantization{};
    dq.dq_norm_disp_low(3.F);
    dq.dq_norm_disp_high(7.F);

    const auto unit = DepthTransform{dq, frame.getBitDepth()};

    const auto actual = unit.expandDepth(frame);

    for (int32_t i = 0; i < 12; ++i) {
      REQUIRE(actual[i] == unit.expandDepth(i));
    }
  }

  SECTION("Expand a frame of 16-bit levels to depth [m]") {
    const auto frame = [] {
      auto x = TMIV::Common::Frame<>::lumaOnly({4, 3}, 16);
      std::iota(x.getPlane(0).begin(), x.getPlane(0).end(), uint16_t{0xE000});
      return x;
    }();

    auto dq = DepthQuantization{};
    dq.dq_norm_disp_low(3.F);
    dq.dq_norm_disp_high(7.F);

    const auto unit = DepthTransform{dq, 16};

    const auto actual = unit.expandDepth(frame);

    for (int32_t i = 0; i < 12; ++i) {
      REQUIRE(actual[i] == unit.expandDepth(0xE000 + i));
    }
  }

  SECTION("Quantize normalized disparity [m^-1] to a level") {
    const auto normDispLow = GENERATE(-2.F, 3.F);
    const auto normDispHigh = GENERATE(5.F, 7.F);

    auto dq = DepthQuantization{};
    dq.dq_norm_disp_low(normDispLow);
    dq.dq_norm_disp_high(normDispHigh);

    constexpr auto bits = 14U;
    constexpr auto maxLevel = TMIV::Common::maxLevel(bits);

    const auto unit = DepthTransform{dq, bits};

    REQUIRE(unit.quantizeNormDisp(normDispLow, 0) == 0);
    REQUIRE(unit.quantizeNormDisp(normDispHigh, 0) == maxLevel);

    SECTION("Invalid depth values are set to zero") {
      REQUIRE(unit.quantizeNormDisp(-1.F, 100) == 0);
      REQUIRE(unit.quantizeNormDisp(0.F, 200) == 0);
      REQUIRE(unit.quantizeNormDisp(NAN, 300) == 0);
    }

    SECTION("Valid depth values are clamped to [minLevel, maxLevel]") {
      const auto x = GENERATE(1E-3F, 0.01, 0.1, 1., 10., 100.);
      const auto minLevel = GENERATE(0U, 1U, 10U, 100U, 1000U);

      REQUIRE(minLevel <= unit.quantizeNormDisp(x, minLevel));
    }

    SECTION("Valid depth values are clamped to maxLevel") {
      const auto x = GENERATE(1E-3F, 0.01, 0.1, 1., 10., 100.);

      REQUIRE(unit.quantizeNormDisp(x, 100) <= maxLevel);
    }
  }

  SECTION("Quantize normalized disparity [m^-1] to a level, PLS method") {
    const auto normDispLow = GENERATE(-2.F, 3.F);
    const auto normDispHigh = GENERATE(5.F, 7.F);

    auto dq = DepthQuantization{};
    dq.dq_norm_disp_low(normDispLow);
    dq.dq_norm_disp_high(normDispHigh);

    dq.dq_quantization_law(2);
    dq.dq_pivot_count_minus1(0);

    float normDispMap0 = normDispLow;
    float normDispMap1 = normDispLow + (normDispHigh - normDispLow) /
                                           static_cast<float>(dq.dq_pivot_count_minus1() + 2);
    dq.dq_pivot_norm_disp(0, normDispMap0);
    dq.dq_pivot_norm_disp(1, normDispMap1);

    constexpr auto bits = 14U;
    constexpr auto maxLevel = TMIV::Common::maxLevel(bits);

    const auto unit = DepthTransform{dq, bits};

    REQUIRE(unit.quantizeNormDisp(normDispLow, 0) == 0);
    REQUIRE(unit.quantizeNormDisp(normDispHigh, 0) == maxLevel);
    REQUIRE(unit.quantizeNormDisp(normDispMap0, 0) == 0);
    REQUIRE(unit.quantizeNormDisp(normDispMap1, 0) == maxLevel);

    SECTION("Invalid depth values are set to zero") {
      REQUIRE(unit.quantizeNormDisp(-1.F, 100) == 0);
      REQUIRE(unit.quantizeNormDisp(0.F, 200) == 0);
      REQUIRE(unit.quantizeNormDisp(NAN, 300) == 0);
    }

    SECTION("Valid depth values are clamped to [minLevel, maxLevel]") {
      const auto x = GENERATE(1E-3F, 0.01, 0.1, 1., 10., 100.);
      const auto minLevel = GENERATE(0U, 1U, 10U, 100U, 1000U);

      REQUIRE(minLevel <= unit.quantizeNormDisp(x, minLevel));
    }

    SECTION("Valid depth values are clamped to maxLevel") {
      const auto x = GENERATE(1E-3F, 0.01, 0.1, 1., 10., 100.);

      REQUIRE(unit.quantizeNormDisp(x, 100) <= maxLevel);
    }
  }

  SECTION("(Implementation-defined) minimum normalized disparity") {
    const auto normDispLow = GENERATE(-2.F, 1E-4F, 1E-3F, 0.01F, 3.F);
    const auto normDispHigh = GENERATE(-3.F, 5.F, 7.F);

    if (normDispLow != normDispHigh && 0 < std::max(normDispLow, normDispHigh)) {
      auto dq = DepthQuantization{};
      dq.dq_norm_disp_low(normDispLow);
      dq.dq_norm_disp_high(normDispHigh);

      const uint32_t bits = GENERATE(1, 4, 13, 16);
      const auto maxLevel = TMIV::Common::maxLevel(bits);

      const auto unit = DepthTransform{dq, bits};

      REQUIRE(0 < unit.minNormDisp());

      if (0 < normDispLow && 0 < normDispHigh) {
        REQUIRE(unit.minNormDisp() <= normDispLow);
        REQUIRE(unit.minNormDisp() <= normDispHigh);
      }

      SECTION(
          "The minimum normalized disparity is less or equal than the lowest positive normalized "
          "disparity that can be formed") {
        for (uint32_t i = 0; i <= maxLevel; ++i) {
          const auto far = std::min(normDispLow, normDispHigh);
          const auto near = std::max(normDispLow, normDispHigh);
          const auto normDisp = far + (near - far) * TMIV::Common::expandValue(i, bits);
          if (0 < normDisp) {
            REQUIRE(unit.minNormDisp() <= normDisp);
            break;
          }
        }
      }
    }
  }
}
