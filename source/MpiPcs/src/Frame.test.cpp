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

#include <catch2/catch.hpp>

#include <TMIV/MpiPcs/Frame.h>

namespace TMIV::MpiPcs {
TEST_CASE("MpiPcs : Frame") {
  MpiPcs::Frame unit({6, 6});
  auto mpiLayer1 = TextureTransparency8Frame{Common::Frame<>::yuv420({6, 6}, 10),
                                             Common::Frame<uint8_t>::lumaOnly({6, 6})};
  auto mpiLayer2 = TextureTransparency8Frame{Common::Frame<>::yuv420({6, 6}, 10),
                                             Common::Frame<uint8_t>::lumaOnly({6, 6})};

  mpiLayer1.transparency.getPlane(0)(0, 2) = 255;
  mpiLayer2.transparency.getPlane(0)(0, 2) = 255;
  mpiLayer2.transparency.getPlane(0)(3, 4) = 255;
  unit.appendLayer(1, mpiLayer1);
  unit.appendLayer(2, mpiLayer2);

  SECTION("construction") {
    REQUIRE(unit(0, 2).size() == 2);
    REQUIRE(unit(3, 4).size() == 1);
    REQUIRE(unit.getPixelList()[1].empty());
  }

  MpiPcs::Frame unit_copy{unit};
  SECTION("constructor from MpiPcs frame") {
    REQUIRE(unit_copy(0, 2).size() == 2);
    REQUIRE(unit_copy(3, 4).size() == 1);
    REQUIRE(unit_copy.getPixelList()[1].empty());
  }

  SECTION("layer equal operator") {
    auto mpiLayer2Reconstructed = unit.getLayer(2);
    REQUIRE(mpiLayer2.transparency.getPlane(0) == mpiLayer2Reconstructed.transparency.getPlane(0));
  }

  SECTION("layer (not) equal operator") {
    REQUIRE_FALSE(mpiLayer1.transparency.getPlane(0) == mpiLayer2.transparency.getPlane(0));
  }

  SECTION("Frame equal operator") {
    MpiPcs::Frame unit_1{};
    MpiPcs::Frame unit_2{};
    REQUIRE(unit_1 == unit_2);
  }

  SECTION("Frame (not) equal operator") {
    MpiPcs::Frame unit_1{};
    REQUIRE_FALSE(unit == unit_1);
  }
}

TEST_CASE("MpiPcs : Attribute") {
  MpiPcs::Attribute unit{};
  unit.geometry = 3;
  unit.texture[0] = 1;
  unit.texture[1] = 10;
  unit.texture[2] = 100;
  unit.transparency = 77;

  SECTION("copy constructor") {
    MpiPcs::Attribute attr{unit};
    REQUIRE(unit == attr);
  }

  SECTION("equal operator") {
    MpiPcs::Attribute attr{};
    REQUIRE_FALSE(unit == attr);
    REQUIRE(attr == attr);
    REQUIRE(unit == unit);
  }

  SECTION("construction from attribute") {
    const auto t = MpiPcs::Attribute::TextureValue{1, 10, 100};
    const auto g = MpiPcs::Attribute::GeometryValue{3};
    const auto a = MpiPcs::Attribute::TransparencyValue{77};
    MpiPcs::Attribute unit_1{t, g, a};
    REQUIRE(unit_1.texture == t);
    REQUIRE(unit_1.geometry == g);
    REQUIRE(unit_1.transparency == a);
  }
}
} // namespace TMIV::MpiPcs
