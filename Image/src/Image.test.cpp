/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <TMIV/Common/Common.h>
#include <TMIV/Image/Image.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Image;

TEST_CASE("maxlevel", "[quantize_and_expand]") {
  REQUIRE(maxLevel(8U) == 255U);
  REQUIRE(maxLevel(10U) == 1023U);
  REQUIRE(maxLevel(16U) == 65535U);
}

TEST_CASE("expandValue", "[quantize_and_expand]") {
  REQUIRE(expandValue<10>(0) == 0.F);
  REQUIRE(expandValue<8>(128) == 128.F / 255.F);
  REQUIRE(expandValue<10>(1023) == 1.F);
  REQUIRE(expandValue<16>(40000) == 40000.F / 65535.F);
}

TEST_CASE("quantizeValue", "[quantize_and_expand]") {
  REQUIRE(quantizeValue<10>(NaN) == 0U);
  REQUIRE(quantizeValue<10>(inf) == 1023U);
  REQUIRE(quantizeValue<10>(1e20F) == 1023U);
}

SCENARIO("Expand YUV 4:2:0 10-bit texture", "[quantize_and_expand]") {
  GIVEN("Such a texture with known values") {
    Frame<YUV420P10> texture{10, 6};
    for (int i = 0; i != 6; ++i) {
      for (int j = 0; j != 10; ++j) {
        texture.getPlane(0)(i, j) = uint16_t(100 * i + j);
      }
    }
    for (int k = 1; k < 3; ++k) {
      for (int i = 0; i != 3; ++i) {
        for (int j = 0; j != 5; ++j) {
          texture.getPlane(k)(i, j) = uint16_t(i + 150 * j);
        }
      }
    }

    WHEN("The texture is expanded") {
      auto expanded = expandTexture(texture);

      THEN("Luma plane samples are expanded") {
        REQUIRE(expanded(5, 9).x() == expandValue<10>(509));
      }

      THEN("Chroma plane samples are upsampled using nearest interpolation") {
        REQUIRE(expanded(4, 8).y() == expandValue<10>(602));
        REQUIRE(expanded(4, 9).z() == expandValue<10>(602));
        REQUIRE(expanded(5, 8).z() == expandValue<10>(602));
        REQUIRE(expanded(5, 9).y() == expandValue<10>(602));
      }
    }
  }
}

SCENARIO("Quantize planar 4:4:4 float to YUV 4:2:0 10-bit texture",
         "[quantize_and_expand]") {
  GIVEN("Such a texture with known values") {
    Mat<Vec3f> texture({6, 10});
    for (int i = 0; i != 6; ++i) {
      for (int j = 0; j != 10; ++j) {
        texture(i, j) = Vec3f{0.1F * float(i) + 0.01F * float(j),
                              0.2F * float(i) + 0.001F * float(j),
                              0.3F * float(i) + 0.1F * float(j)};
      }
    }

    WHEN("The texture is quantized") {
      auto quantized = quantizeTexture(texture);

      THEN("Luma plane samples are quantized") {
        REQUIRE(quantized.getPlane(0)(5, 9) == quantizeValue<10>(0.59F));
      }
    }
  }
}

TEST_CASE("requantizeValue", "[quantize_and_expand]") {
  REQUIRE(0U == requantizeValue<uint16_t, uint32_t>(0U, 16, 16));
  REQUIRE(0U == requantizeValue<uint16_t, uint32_t>(0U, 16, 13));
  REQUIRE(0U == requantizeValue<uint16_t, uint32_t>(0U, 9, 15));
  REQUIRE(0xFFFFU == requantizeValue<uint16_t, uint32_t>(0xFFFFU, 16, 16));
  REQUIRE(0xFFFFU == requantizeValue<uint16_t, uint64_t>(0xFFFFU, 16, 16));
  REQUIRE(0xFFFFU == requantizeValue<uint16_t, uint32_t>(0xFFFU, 12, 16));
  REQUIRE(0x1FU == requantizeValue<uint8_t, uint16_t>(0x7FU, 7, 5));
  REQUIRE(0x800 == requantizeValue<uint16_t, uint32_t>(0x8000, 16, 12));
  REQUIRE(0x400 == requantizeValue<uint16_t, uint32_t>(0x800, 12, 11));
}

TEST_CASE("requantize", "[quantize_and_expand]") {
  // Input is a multiview depth frame with a single view
  auto inFrame =
      MVD10Frame{TextureDepth10Frame{TextureFrame{5, 7}, Depth10Frame{32, 16}}};

  // Put a 9-bit ramp in the depth map
  auto i = 0U;
  for (auto &x : inFrame.front().second.getPlane(0)) {
    x = i++;
  }

  // Convert from YUV 4:0:0 9-bit (within 10-bit type) to YUV 4:2:0 16-bit
  auto outFrame = requantize<YUV420P16>(inFrame, 9);

  // Output has the same size
  REQUIRE(outFrame.size() == 1);
  REQUIRE(outFrame.front().first.getSize() == Vec2i{5, 7});
  REQUIRE(outFrame.front().second.getSize() == Vec2i{32, 16});
  const auto &Y = outFrame.front().second.getPlane(0);
  const auto &Cb = outFrame.front().second.getPlane(1);
  const auto &Cr = outFrame.front().second.getPlane(2);

  // The conversion maps 0 --> 0 and 511 --> 65535 with rounding to the nearest
  // integer
  REQUIRE(0U == Y(0, 0));
  REQUIRE(513U == Y(0, 4)); // 4 * 65535 / 511 == 512.994 (round up)
  REQUIRE(641U == Y(0, 5)); // 5 * 65535 / 511 == 641.243 (round down)

  REQUIRE(0xFFFFU == Y(0xF, 0x1F));

  // Chroma is all zero
  REQUIRE(0U == Cb(7, 13));
  REQUIRE(0U == Cr(0, 5));
}

TEST_CASE("Trivial use of compressRangeValue", "[compress_depth_range]") {
	REQUIRE(0x00u == compressRangeValue<uint8_t, uint32_t>(0x00u, 8u, 8u, 0x00u));
	REQUIRE(0x01u == compressRangeValue<uint8_t, uint32_t>(0x01u, 8u, 8u, 0x00u));
	REQUIRE(0xFEu == compressRangeValue<uint8_t, uint32_t>(0xFEu, 8u, 8u, 0x00u));
	REQUIRE(0xFFu == compressRangeValue<uint8_t, uint32_t>(0xFFu, 8u, 8u, 0x00u));
	REQUIRE(0x0000u == compressRangeValue<uint16_t, uint32_t>(0x0000u, 16u, 16u, 0x0000u));
	REQUIRE(0x0001u == compressRangeValue<uint16_t, uint32_t>(0x0001u, 16u, 16u, 0x0000u));
	REQUIRE(0xFFFEu == compressRangeValue<uint16_t, uint32_t>(0xFFFEu, 16u, 16u, 0x0000u));
	REQUIRE(0xFFFFu == compressRangeValue<uint16_t, uint32_t>(0xFFFFu, 16u, 16u, 0x0000u));
	REQUIRE(0x000u == compressRangeValue<uint16_t, uint32_t>(0x0000u, 16u, 12u, 0x000u));
	REQUIRE(0x000u == compressRangeValue<uint16_t, uint32_t>(0x0001u, 16u, 12u, 0x000u));
	REQUIRE(0xFFFu == compressRangeValue<uint16_t, uint32_t>(0xFFFEu, 16u, 12u, 0x000u));
	REQUIRE(0xFFFu == compressRangeValue<uint16_t, uint32_t>(0xFFFFu, 16u, 12u, 0x000u));
}

TEST_CASE("Non-trivial use of compressRangeValue", "[compress_depth_range]") {
	REQUIRE(0x10u == compressRangeValue<uint8_t, uint32_t>(0x00u, 8u, 8u, 0x10u));
	REQUIRE(0x21u == compressRangeValue<uint8_t, uint32_t>(0x01u, 8u, 8u, 0x20u));
	REQUIRE(0xFEu == compressRangeValue<uint8_t, uint32_t>(0xFEu, 8u, 8u, 0x30u));
	REQUIRE(0xFFu == compressRangeValue<uint8_t, uint32_t>(0xFFu, 8u, 8u, 0x40u));
	REQUIRE(0x0001u == compressRangeValue<uint16_t, uint32_t>(0x0000u, 16u, 16u, 0x0001u));
	REQUIRE(0x0021u == compressRangeValue<uint16_t, uint32_t>(0x0001u, 16u, 16u, 0x0020u));
	REQUIRE(0xFFFEu == compressRangeValue<uint16_t, uint32_t>(0xFFFEu, 16u, 16u, 0x0300u));
	REQUIRE(0xFFFFu == compressRangeValue<uint16_t, uint32_t>(0xFFFFu, 16u, 16u, 0x4000u));
	REQUIRE(0x100u == compressRangeValue<uint16_t, uint32_t>(0x0000u, 16u, 12u, 0x100u));
	REQUIRE(0x200u == compressRangeValue<uint16_t, uint32_t>(0x0001u, 16u, 12u, 0x200u));
	REQUIRE(0xFFFu == compressRangeValue<uint16_t, uint32_t>(0xFFFEu, 16u, 12u, 0x300u));
	REQUIRE(0xFFFu == compressRangeValue<uint16_t, uint32_t>(0xFFFFu, 16u, 12u, 0x400u));
}

TEST_CASE("Trivial use of decompressRangeValue", "[compress_depth_range]") {
	REQUIRE(0x00u == decompressRangeValue<uint8_t, uint32_t>(0x00u, 8u, 8u, 0x00u));
	REQUIRE(0x01u == decompressRangeValue<uint8_t, uint32_t>(0x01u, 8u, 8u, 0x00u));
	REQUIRE(0xFEu == decompressRangeValue<uint8_t, uint32_t>(0xFEu, 8u, 8u, 0x00u));
	REQUIRE(0xFFu == decompressRangeValue<uint8_t, uint32_t>(0xFFu, 8u, 8u, 0x00u));
	REQUIRE(0x0000u == decompressRangeValue<uint16_t, uint32_t>(0x0000u, 16u, 16u, 0x0000u));
	REQUIRE(0x0001u == decompressRangeValue<uint16_t, uint32_t>(0x0001u, 16u, 16u, 0x0000u));
	REQUIRE(0xFFFEu == decompressRangeValue<uint16_t, uint32_t>(0xFFFEu, 16u, 16u, 0x0000u));
	REQUIRE(0xFFFFu == decompressRangeValue<uint16_t, uint32_t>(0xFFFFu, 16u, 16u, 0x0000u));
	REQUIRE(0x0000u == decompressRangeValue<uint16_t, uint32_t>(0x000u, 12u, 16u, 0x0000u));
	REQUIRE(0x0010u == decompressRangeValue<uint16_t, uint32_t>(0x001u, 12u, 16u, 0x0000u));
	REQUIRE(0xFFEFu == decompressRangeValue<uint16_t, uint32_t>(0xFFEu, 12u, 16u, 0x0000u));
	REQUIRE(0xFFFFu == decompressRangeValue<uint16_t, uint32_t>(0xFFFu, 12u, 16u, 0x0000u));
}

TEST_CASE("Non-trivial use of decompressRangeValue", "[compress_depth_range]") {
	REQUIRE(0x00u == decompressRangeValue<uint8_t, uint32_t>(0x10u, 8u, 8u, 0x10u));
	REQUIRE(0x01u == decompressRangeValue<uint8_t, uint32_t>(0x21u, 8u, 8u, 0x20u));
	REQUIRE(0xFEu == decompressRangeValue<uint8_t, uint32_t>(0xFEu, 8u, 8u, 0x30u));
	REQUIRE(0xFFu == decompressRangeValue<uint8_t, uint32_t>(0xFFu, 8u, 8u, 0x40u));
	REQUIRE(0x0000u == decompressRangeValue<uint16_t, uint32_t>(0x0001u, 16u, 16u, 0x0001u));
	REQUIRE(0x0001u == decompressRangeValue<uint16_t, uint32_t>(0x0021u, 16u, 16u, 0x0020u));
	REQUIRE(0xFFFEu == decompressRangeValue<uint16_t, uint32_t>(0xFFFEu, 16u, 16u, 0x0300u));
	REQUIRE(0xFFFFu == decompressRangeValue<uint16_t, uint32_t>(0xFFFFu, 16u, 16u, 0x4000u));
	REQUIRE(0x0000u == decompressRangeValue<uint16_t, uint32_t>(0x100u, 12u, 16u, 0x100u));
	REQUIRE(0x0012u == decompressRangeValue<uint16_t, uint32_t>(0x201u, 12u, 16u, 0x200u));
	REQUIRE(0xFFFFu == decompressRangeValue<uint16_t, uint32_t>(0xFFFu, 12u, 16u, 0x300u));
	REQUIRE(0xFFFFu == decompressRangeValue<uint16_t, uint32_t>(0xFFFu, 12u, 16u, 0x400u));
}