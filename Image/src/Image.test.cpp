/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include <TMIV/Image/Image.h>
#include <TMIV/Common/Common.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Image;

TEST_CASE("maxlevel", "[quantize_and_expand]") {
  REQUIRE(maxLevel(8u) == 255u);
  REQUIRE(maxLevel(10u) == 1023u);
  REQUIRE(maxLevel(16u) == 65535u);
}

TEST_CASE("expandValue", "[quantize_and_expand]") {
  REQUIRE(expandValue<10>(0) == 0.f);
  REQUIRE(expandValue<8>(128) == 128.f / 255.f);
  REQUIRE(expandValue<10>(1023) == 1.f);
  REQUIRE(expandValue<16>(40000) == 40000.f / 65535.f);
}

TEST_CASE("quantizeValue", "[quantize_and_expand]") {
  REQUIRE(quantizeValue<10>(NaN) == 0u);
  REQUIRE(quantizeValue<10>(inf) == 1023u);
  REQUIRE(quantizeValue<10>(1e20f) == 1023u);
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
        texture(i, j) = Vec3f{0.1f * i + 0.01f * j, 0.2f * i + 0.001f * j,
                              0.3f * i + 0.1f * j};
      }
    }

    WHEN("The texture is quantized") {
      auto quantized = quantizeTexture(texture);

      THEN("Luma plane samples are quantized") {
        REQUIRE(quantized.getPlane(0)(5, 9) == quantizeValue<10>(0.59f));
      }
    }
  }
}
