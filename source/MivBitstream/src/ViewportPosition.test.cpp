/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include "test.h"

#include <TMIV/MivBitstream/ViewportPosition.h>

namespace TMIV::MivBitstream {
namespace examples {
const auto viewportPosition =
    std::array{ViewportPosition{},
               ViewportPosition{10, false, {}, true},
               ViewportPosition{18, true, 69, true},
               ViewportPosition{18, true, 69, false},
               ViewportPosition{18, true, 69, false, true, Common::Vec3f{1.1F, 2.2F, 3.3F},
                                static_cast<int16_t>(128), static_cast<int16_t>(3037),
                                static_cast<int16_t>(32767)},
               ViewportPosition{18, true, 69, false, true, Common::Vec3f{1.1F, 2.2F, 3.3F},
                                static_cast<int16_t>(128), static_cast<int16_t>(3037),
                                static_cast<int16_t>(-32768), true},
               ViewportPosition{18, true, 69, false, true, Common::Vec3f{1.1F, 2.2F, 3.3F},
                                static_cast<int16_t>(128), static_cast<int16_t>(3037),
                                static_cast<int16_t>(32767), false, true},
               ViewportPosition{0},
               ViewportPosition{2},
               ViewportPosition{4},
               ViewportPosition{10},
               ViewportPosition{100},
               ViewportPosition{{},
                                false,
                                {},
                                false,
                                true,
                                Common::Vec3f{1.1F, 2.2F, 3.3F},
                                static_cast<int16_t>(4),
                                static_cast<int16_t>(5),
                                static_cast<int16_t>(6),
                                true}};
} // namespace examples

TEST_CASE("Viewport position coding") {
  REQUIRE(bitCodingTest(examples::viewportPosition[0], 150));
  REQUIRE(bitCodingTest(examples::viewportPosition[1], 9));
  REQUIRE(bitCodingTest(examples::viewportPosition[2], 21));
  REQUIRE(bitCodingTest(examples::viewportPosition[3], 168));
  REQUIRE(bitCodingTest(examples::viewportPosition[4], 168));
  REQUIRE(bitCodingTest(examples::viewportPosition[5], 167));
  REQUIRE(bitCodingTest(examples::viewportPosition[6], 168));
  REQUIRE(bitCodingTest(examples::viewportPosition[7], 150));
  REQUIRE(bitCodingTest(examples::viewportPosition[8], 152));
  REQUIRE(bitCodingTest(examples::viewportPosition[9], 154));
  REQUIRE(bitCodingTest(examples::viewportPosition[10], 156));
  REQUIRE(bitCodingTest(examples::viewportPosition[11], 162));
  REQUIRE(bitCodingTest(examples::viewportPosition[12], 149));
}

TEST_CASE("Viewport position from view params") {
  const auto json = TMIV::Common::Json::parse(R"(
{
    "Depth_range": [ 0.1, 500.0 ],
    "Hor_range": [ -90.0, 90.0 ],
    "Name": "v2",
    "Position": [ -0.2878679633140564, -0.0878679633140564, 1.0 ],
    "Projection": "Equirectangular",
    "Resolution": [ 2048, 1048 ],
    "Rotation": [ 45.00000125223908, 19.3, 4.3 ],
    "Ver_range": [ -90.0, 90.0 ]
})");
  auto vp = TMIV::MivBitstream::ViewParams(json);
  ViewportPosition unit = ViewportPosition::fromViewParams(vp);
  REQUIRE(bitCodingTest(unit, 149));
}
} // namespace TMIV::MivBitstream
