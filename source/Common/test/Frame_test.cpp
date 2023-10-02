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

#include <TMIV/Common/Frame.h>

namespace TMIV::Common {
namespace {
template <typename Element>
void checkLuminancePlaneContents(const Frame<Element> &unit, uint32_t expectedPixelValue = 0) {
  const auto &yPlane = unit.getPlane(0);
  REQUIRE(!yPlane.empty());
  REQUIRE(yPlane.width() == static_cast<size_t>(unit.getWidth()));
  REQUIRE(yPlane.height() == static_cast<size_t>(unit.getHeight()));
  REQUIRE(yPlane.size() == static_cast<size_t>(unit.getWidth() * unit.getHeight()));
  for (const auto pixel : yPlane) {
    REQUIRE(pixel == expectedPixelValue);
  }
}

template <typename Element>
void checkChrominancePlaneContents(const Frame<Element> &unit, int32_t planeIdx,
                                   uint32_t expectedPixelValue = 0) {
  const auto &uPlane = unit.getPlane(planeIdx);
  REQUIRE(!uPlane.empty());
  REQUIRE(uPlane.width() == static_cast<size_t>(unit.getWidth() / 2));
  REQUIRE(uPlane.height() == static_cast<size_t>(unit.getHeight() / 2));
  REQUIRE(uPlane.size() == static_cast<size_t>(unit.getWidth() * unit.getHeight() / 4));
  for (const auto pixel : uPlane) {
    REQUIRE(pixel == expectedPixelValue);
  }
}

template <typename Element>
void checkIfPlanesContainColor(const Frame<Element> &unit, uint32_t expectedColor) {
  checkLuminancePlaneContents(unit, expectedColor);
  checkChrominancePlaneContents(unit, 1, expectedColor);
  checkChrominancePlaneContents(unit, 2, expectedColor);
}

// NOTE(#397): Cannot use TEMPLATE_TEST_CASE because of clang-tidy warnings. At the time of writing
// this is an open issue on Catch2: https://github.com/catchorg/Catch2/issues/2095

template <typename F> void forEachElement(F &&f) {
  f(uint8_t{});
  f(uint16_t{});
  f(uint32_t{});
}
} // namespace

TEST_CASE("0x0 frame through default construction") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    const Frame<TestType> unit{};

    REQUIRE(unit.empty());

    REQUIRE(unit.getNumberOfPlanes() == 0);
    REQUIRE(unit.getPlanes().empty());
  });
}

TEST_CASE("4x2 frame") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    const Frame<TestType> unit{{4, 2}, 8, ColorFormat::YUV420};

    REQUIRE(!unit.empty());
    REQUIRE(unit.getHeight() == 2);
    REQUIRE(unit.getWidth() == 4);
    REQUIRE(unit.getSize() == Vec2i{4, 2});
    REQUIRE(unit.getByteCount() == 12 * sizeof(TestType));
    REQUIRE(unit.getBitDepth() == 8);
    REQUIRE(unit.neutralValue() == 0x80);

    REQUIRE(unit.getNumberOfPlanes() == 3);

    checkLuminancePlaneContents(unit);
    checkChrominancePlaneContents(unit, 1);
    checkChrominancePlaneContents(unit, 2);
  });
}

TEST_CASE("4x2 frame with neutral color value") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> unit{{4, 2}, 8, ColorFormat::YUV420};
    unit.fillNeutral();
    checkIfPlanesContainColor(unit, unit.neutralValue());
  });
}

TEST_CASE("4x2 frame with max color value") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> unit{{4, 2}, 8, ColorFormat::YUV420};
    unit.fillMax();

    checkIfPlanesContainColor(unit, 0xFF);
  });
}

TEST_CASE("4x2 frame reset to zero") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> unit{{4, 2}, 8, ColorFormat::YUV420};
    unit.fillNeutral();
    unit.fillZero();

    checkIfPlanesContainColor(unit, 0);
  });
}

TEST_CASE("4x2 frame fillInvalidWithNeutral") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> unit{{4, 2}, 6, ColorFormat::YUV444};

    Frame<uint8_t> depthFrame{{4, 2}, 8, ColorFormat::YUV420};
    depthFrame.fillOne();
    auto &yPlane = depthFrame.getPlane(0);
    yPlane[3] = 0;

    unit.fillInvalidWithNeutral(depthFrame);

    for (const auto &plane : unit.getPlanes()) {
      for (uint32_t pixel_index = 0; pixel_index < plane.size(); ++pixel_index) {
        if (pixel_index == 3) {
          REQUIRE(plane[pixel_index] == unit.neutralValue());
        } else {
          REQUIRE(plane[pixel_index] == 0);
        }
      }
    }
  });
}

TEST_CASE("Resizing 0x0 frame to 4x2") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> unit{};
    REQUIRE(unit.empty());

    unit.createYuv420({4, 2});
    REQUIRE(unit.getWidth() == 4);
    REQUIRE(unit.getHeight() == 2);
    REQUIRE(unit.getNumberOfPlanes() == 3);
    REQUIRE(unit.getBitDepth() == std::numeric_limits<TestType>::digits);
    REQUIRE(unit.getColorFormat() == ColorFormat::YUV420);
  });
}

TEST_CASE("Convert Yuv444P10 to YUV420P10") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> sourceFrame{{2, 2}, 6, ColorFormat::YUV444};
    sourceFrame.fillNeutral();
    auto offset = TestType{};
    for (auto &plane : sourceFrame.getPlanes()) {
      for (auto &pixel : plane) {
        pixel += offset++;
      }
    }

    const auto destinationFrame = yuv420(sourceFrame);

    const auto &yPlane = destinationFrame.getPlane(0);
    REQUIRE(yPlane[0] == 32);
    REQUIRE(yPlane[1] == 33);
    REQUIRE(yPlane[2] == 34);
    REQUIRE(yPlane[3] == 35);
    const auto &uPlane = destinationFrame.getPlane(1);
    REQUIRE(uPlane[0] == 38);
    const auto &vPlane = destinationFrame.getPlane(2);
    REQUIRE(vPlane[0] == 42);
  });
}

TEST_CASE("Convert YUV420P10 to Yuv444P10 ") {
  forEachElement([](auto zero) {
    using TestType = decltype(zero);

    Frame<TestType> sourceFrame{{2, 2}, 6, ColorFormat::YUV420};
    sourceFrame.fillNeutral();
    auto offset = TestType{};
    for (auto &plane : sourceFrame.getPlanes()) {
      for (auto &pixel : plane) {
        pixel += offset++;
      }
    }

    const auto destinationFrame = yuv444(sourceFrame);

    const auto &yPlane = destinationFrame.getPlane(0);
    REQUIRE(yPlane[0] == 32);
    REQUIRE(yPlane[1] == 33);
    REQUIRE(yPlane[2] == 34);
    REQUIRE(yPlane[3] == 35);
    for (const auto pixel : destinationFrame.getPlane(1)) {
      REQUIRE(pixel == 36);
    }
    for (const auto pixel : destinationFrame.getPlane(2)) {
      REQUIRE(pixel == 37);
    }
  });
}

TEST_CASE("Expand texture with zeroes") {
  const auto result = expandTexture(Frame<uint16_t>{{4, 2}, 10, ColorFormat::YUV444});

  REQUIRE(result.height() == 2);
  REQUIRE(result.width() == 4);
  for (const auto &pixel : result) {
    REQUIRE(pixel == Vec3f{0.F, 0.F, 0.F});
  }
}

TEST_CASE("Expand texture with ones") {
  Frame<uint16_t> frame{{4, 2}, 10, ColorFormat::YUV444};
  frame.fillOne();

  const auto result = expandTexture(frame);

  for (const auto &pixel : result) {
    for (const auto color_channel : pixel) {
      REQUIRE(color_channel == Catch::Approx(0.000977517F).epsilon(1e-5F));
    }
  }
}

TEST_CASE("Expand luma with ones") {
  Frame<uint16_t> frame{{4, 2}, 10, ColorFormat::YUV420};
  frame.fillOne();

  const auto result = expandLuma(frame);

  REQUIRE(result.height() == 2);
  REQUIRE(result.width() == 4);
  for (const auto &pixel : result) {
    REQUIRE(pixel == Catch::Approx(98e-5F).epsilon(1e-2F));
  }
}

TEST_CASE("Quantize texture") {
  Mat<Vec3f> flatFrame{};
  flatFrame.resize(2, 4);
  float offset = 0.F;
  for (uint32_t row_index = 0; row_index < flatFrame.height(); ++row_index) {
    for (uint32_t col_index = 0; col_index < flatFrame.width(); ++col_index) {
      offset += 0.1F;
      flatFrame(row_index, col_index) = Vec3f{offset, offset + 0.02F, offset + 0.04F};
    }
  }

  const auto result = quantizeTexture(flatFrame, 10);

  REQUIRE(result.getPlane(0)(0, 0) == 102);
  REQUIRE(result.getPlane(1)(0, 0) == 123);
  REQUIRE(result.getPlane(2)(0, 0) == 143);

  REQUIRE(result.getPlane(0)(1, 1) == 614);
  REQUIRE(result.getPlane(1)(1, 1) == 634);
  REQUIRE(result.getPlane(2)(1, 1) == 655);

  REQUIRE(result.getPlane(0)(1, 3) == 818);
  REQUIRE(result.getPlane(1)(1, 3) == 839);
  REQUIRE(result.getPlane(2)(1, 3) == 859);
}
} // namespace TMIV::Common
