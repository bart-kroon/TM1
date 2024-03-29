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

#include "../src/PushPull.h"

#include <TMIV/Common/Bitstream.h>

namespace {
const auto encodeCoordinates = [](uint16_t w, uint16_t h) {
  auto frame = TMIV::Common::RendererFrame{TMIV::Common::Frame<>::yuv444({w, h}, 10),
                                           TMIV::Common::Frame<>::lumaOnly({w, h}, 16)};

  for (uint16_t y = 0; y < h; ++y) {
    for (uint16_t x = 0; x < w; ++x) {
      frame.texture.getPlane(0)(y, x) = x;
      frame.texture.getPlane(1)(y, x) = y;
    }
  }
  return frame;
};

using YUVD = std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>;
} // namespace

TEST_CASE("inplacePush") {
  SECTION("inplacePush invokes the filter once for each output pixel") {
    const auto wi = GENERATE(0, 1, 2, 13);
    const auto hi = GENERATE(0, 1, 2, 32);

    auto count = 0;
    auto in = TMIV::Common::RendererFrame{TMIV::Common::Frame<>::yuv444({wi, hi}, 10),
                                          TMIV::Common::Frame<>::lumaOnly({wi, hi}, 16)};
    auto out = TMIV::Common::RendererFrame{};

    TMIV::Renderer::PushPull::inplacePushFrame(in, out, [&count](const auto & /* unused */) {
      ++count;
      return YUVD{};
    });

    REQUIRE(count == out.texture.getWidth() * out.texture.getHeight());
  }

  SECTION("inplacePush resizes the output frame") {
    const auto wi = GENERATE(0, 1, 2, 13);
    const auto hi = GENERATE(0, 1, 2, 32);

    auto in = TMIV::Common::RendererFrame{TMIV::Common::Frame<>::yuv444({wi, hi}, 10),
                                          TMIV::Common::Frame<>::lumaOnly({wi, hi}, 16)};
    auto out = TMIV::Common::RendererFrame{};

    TMIV::Renderer::PushPull::inplacePushFrame(in, out,
                                               [](const auto & /* unused */) { return YUVD{}; });

    const auto wo = (wi + 1) / 2;
    const auto ho = (hi + 1) / 2;
    REQUIRE(out.texture.getWidth() == wo);
    REQUIRE(out.texture.getHeight() == ho);
  }

  SECTION("inplacePush filters blocks of 2 x 2 pixels down to 1 x 1 pixel") {
    auto out = TMIV::Common::RendererFrame{};
    TMIV::Renderer::PushPull::inplacePushFrame(
        encodeCoordinates(4, 6), out, [](const std::array<YUVD, 4> &pixelValues) {
          // In this test case, pixel positions are encoded in the pixel values, and the checks test
          // if positions are odd or even.

          constexpr auto isEven = [](uint16_t x) { return x % 2 == 0; };
          constexpr auto isOdd = [](uint16_t x) { return x % 2 != 0; };

          REQUIRE(isEven(std::get<0>(pixelValues[0])));
          REQUIRE(isOdd(std::get<0>(pixelValues[1])));
          REQUIRE(isEven(std::get<0>(pixelValues[2])));
          REQUIRE(isOdd(std::get<0>(pixelValues[3])));

          REQUIRE(isEven(std::get<1>(pixelValues[0])));
          REQUIRE(isEven(std::get<1>(pixelValues[1])));
          REQUIRE(isOdd(std::get<1>(pixelValues[2])));
          REQUIRE(isOdd(std::get<1>(pixelValues[3])));

          return YUVD{};
        });
  }

  SECTION("inplacePush repeats the right border when the input has odd width") {
    auto out = TMIV::Common::RendererFrame{};
    TMIV::Renderer::PushPull::inplacePushFrame(
        encodeCoordinates(5, 4), out, [](const std::array<YUVD, 4> &pixelValues) {
          REQUIRE((std::get<0>(pixelValues[0]) < 4 || std::get<0>(pixelValues[1]) == 4));
          return YUVD{};
        });
  }

  SECTION("inplacePush repeats the bottom border when the input has odd height") {
    auto out = TMIV::Common::RendererFrame{};
    TMIV::Renderer::PushPull::inplacePushFrame(
        encodeCoordinates(4, 5), out, [](const std::array<YUVD, 4> &pixelValues) {
          REQUIRE((std::get<1>(pixelValues[0]) < 4 || std::get<1>(pixelValues[2]) == 4));
          return YUVD{};
        });
  }

  SECTION("inplacePush writes the output of the filter into the output frame") {
    auto out = TMIV::Common::RendererFrame{};
    TMIV::Renderer::PushPull::inplacePushFrame(
        encodeCoordinates(4, 4), out,
        [](const std::array<YUVD, 4> &pixelValues) { return pixelValues[3]; });

    REQUIRE(out.texture.getPlane(0)(0, 0) == 1);
    REQUIRE(out.texture.getPlane(0)(0, 1) == 3);
    REQUIRE(out.texture.getPlane(0)(1, 0) == 1);
    REQUIRE(out.texture.getPlane(0)(1, 1) == 3);

    REQUIRE(out.texture.getPlane(1)(0, 0) == 1);
    REQUIRE(out.texture.getPlane(1)(0, 1) == 1);
    REQUIRE(out.texture.getPlane(1)(1, 0) == 3);
    REQUIRE(out.texture.getPlane(1)(1, 1) == 3);
  }
}

TEST_CASE("inplacePull") {
  SECTION("inplacePull invokes the filter once for each output pixel") {
    const auto w = GENERATE(0, 1, 2, 13);
    const auto h = GENERATE(0, 1, 2, 32);

    auto count = 0;
    auto in = TMIV::Common::RendererFrame{
        TMIV::Common::Frame<>::yuv444({(w + 1) / 2, (h + 1) / 2}, 10),
        TMIV::Common::Frame<>::lumaOnly({(w + 1) / 2, (h + 1) / 2}, 10)};
    auto out = TMIV::Common::RendererFrame{TMIV::Common::Frame<>::yuv444({w, h}, 10),
                                           TMIV::Common::Frame<>::lumaOnly({w, h}, 10)};
    TMIV::Renderer::PushPull::inplacePullFrame(in, out, [&count](const auto &.../* unused */) {
      ++count;
      return YUVD{};
    });

    REQUIRE(count == out.texture.getWidth() * out.texture.getHeight());
  }

  SECTION("inplacePull allows for bilinear interpolation and border repeat") {
    //  in     0       1       2
    // out   |   |   |   |   |   |
    // pos  0,0 0,1 1,0 1,2 2,1 2,2
    auto countX = std::array<std::array<int32_t, 3>, 3>{};
    auto countY = std::array<std::array<int32_t, 3>, 3>{};

    const auto in = encodeCoordinates(3, 3);
    auto out = TMIV::Common::RendererFrame{TMIV::Common::Frame<>::yuv444({6, 6}, 10),
                                           TMIV::Common::Frame<>::lumaOnly({6, 6}, 16)};
    TMIV::Renderer::PushPull::inplacePullFrame(
        in, out, [&](const std::array<YUVD, 4> &pixelValues, const auto & /* unused */) {
          ++TMIV::Common::at(countX, std::get<0>(pixelValues[0]), std::get<0>(pixelValues[1]));
          ++TMIV::Common::at(countY, std::get<1>(pixelValues[0]), std::get<1>(pixelValues[2]));
          return YUVD{};
        });

    for (int32_t i : {0, 1, 2}) {
      for (int32_t j : {0, 1, 2}) {
        if (std::abs(i - j) == 1) {
          REQUIRE(TMIV::Common::at(countX, i, j) == 6);
          REQUIRE(TMIV::Common::at(countY, i, j) == 6);
        } else if (i == j && (i == 0 || i == 2)) {
          REQUIRE(TMIV::Common::at(countX, i, j) == 6);
          REQUIRE(TMIV::Common::at(countY, i, j) == 6);
        } else {
          REQUIRE(TMIV::Common::at(countX, i, j) == 0);
          REQUIRE(TMIV::Common::at(countY, i, j) == 0);
        }
      }
    }
  }
}

TEST_CASE("PushPull") {
  SECTION("default construction") { const auto pushPull = TMIV::Renderer::PushPull{}; }

  GIVEN("A push-pull object and frame") {
    auto pushPull = TMIV::Renderer::PushPull{};

    const auto w = GENERATE(uint16_t{}, uint16_t{1}, uint16_t{13}, uint16_t{12});
    const auto h = GENERATE(uint16_t{}, uint16_t{1}, uint16_t{7}, uint16_t{8});
    CAPTURE(w, h);

    const auto in = encodeCoordinates(w, h);

    WHEN("Filtering the frame") {
      const auto &out = pushPull.filter(
          in, [](const auto & /*unused */) { return YUVD{}; },
          [](const auto & /*unused */, const auto & /*unused */) { return YUVD{}; });

      THEN("The filtered image has the same size as the input") {
        REQUIRE(in.texture.getWidth() == out.texture.getWidth());
        REQUIRE(in.texture.getHeight() == out.texture.getHeight());
        REQUIRE(in.geometry.getWidth() == out.geometry.getWidth());
        REQUIRE(in.geometry.getHeight() == out.geometry.getHeight());
      }

      THEN("The number of layers in the pyramid is accessible") {
        REQUIRE(pushPull.numLayers() == 1U + TMIV::Common::ceilLog2(std::max(w, h)));
      }

      THEN("Each filtered layer is accessible") {
        int32_t ww = w;
        int32_t hh = h;

        for (size_t i = 0; i < pushPull.numLayers(); ++i) {
          REQUIRE(pushPull.layer(i).texture.getWidth() == ww);
          REQUIRE(pushPull.layer(i).texture.getHeight() == hh);
          ww = (ww + 1) / 2;
          hh = (hh + 1) / 2;
        }
      }
    }
  }
}
