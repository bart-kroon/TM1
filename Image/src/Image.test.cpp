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
  REQUIRE(TMIV::Image::impl::expandValue<10>(0) == 0.F);
  REQUIRE(TMIV::Image::impl::expandValue<8>(128) == 128.F / 255.F);
  REQUIRE(TMIV::Image::impl::expandValue<10>(1023) == 1.F);
  REQUIRE(TMIV::Image::impl::expandValue<16>(40000) == 40000.F / 65535.F);
}

TEST_CASE("quantizeValue", "[quantize_and_expand]") {
  REQUIRE(TMIV::Image::impl::quantizeValue<10>(NaN) == 0U);
  REQUIRE(TMIV::Image::impl::quantizeValue<10>(inf) == 1023U);
  REQUIRE(TMIV::Image::impl::quantizeValue<10>(1e20F) == 1023U);
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
        REQUIRE(expanded(5, 9).x() == TMIV::Image::impl::expandValue<10>(509));
      }

      THEN("Chroma plane samples are upsampled using nearest interpolation") {
        REQUIRE(expanded(4, 8).y() == TMIV::Image::impl::expandValue<10>(602));
        REQUIRE(expanded(4, 9).z() == TMIV::Image::impl::expandValue<10>(602));
        REQUIRE(expanded(5, 8).z() == TMIV::Image::impl::expandValue<10>(602));
        REQUIRE(expanded(5, 9).y() == TMIV::Image::impl::expandValue<10>(602));
      }
    }
  }
}

SCENARIO("Quantize planar 4:4:4 float to YUV 4:2:0 10-bit texture", "[quantize_and_expand]") {
  GIVEN("Such a texture with known values") {
    Mat<Vec3f> texture({6, 10});
    for (int i = 0; i != 6; ++i) {
      for (int j = 0; j != 10; ++j) {
        texture(i, j) =
            Vec3f{0.1F * float(i) + 0.01F * float(j), 0.2F * float(i) + 0.001F * float(j),
                  0.3F * float(i) + 0.1F * float(j)};
      }
    }

    WHEN("The texture is quantized") {
      auto quantized = quantizeTexture(texture);

      THEN("Luma plane samples are quantized") {
        REQUIRE(quantized.getPlane(0)(5, 9) == TMIV::Image::impl::quantizeValue<10>(0.59F));
      }
    }
  }
}
