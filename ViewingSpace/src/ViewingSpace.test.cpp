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

#include <TMIV/ViewingSpace/SignedDistance.h>

using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::ViewingSpace;

TEST_CASE("Signed distance functions") {
  SECTION("Distance sign") {
    REQUIRE(SignedDistance(-1.F).isInside());
    REQUIRE(SignedDistance(1.F).isOutside());
  }
  const Cuboid cuboid{{1.F, 2.F, 3.F}, {11.F, 13.F, 17.F}};
  const Spheroid spheroid{{5.F, 4.F, 3.F}, {2.F, 3.F, 4.F}};
  SECTION("Cuboid") {
    REQUIRE(signedDistance(cuboid, EulerAngles({}), cuboid.center).isInside());
    REQUIRE(
        signedDistance(cuboid, EulerAngles({}), cuboid.center + 0.49F * cuboid.size).isInside());
    REQUIRE(
        signedDistance(cuboid, EulerAngles({180.F, 0.F, 0.F}), cuboid.center - 0.49F * cuboid.size)
            .isInside());
    REQUIRE(
        signedDistance(cuboid, EulerAngles({}), cuboid.center + 0.51F * cuboid.size).isOutside());
    REQUIRE(signedDistance(cuboid, EulerAngles({180.F, 180.F, 180.F}),
                           cuboid.center - 0.51F * cuboid.size)
                .isOutside());
  }
  SECTION("Spheroid") {
    REQUIRE(signedDistance(spheroid, EulerAngles({}), spheroid.center).isInside());
    REQUIRE(signedDistance(spheroid, EulerAngles({}),
                           spheroid.center + Vec3f({0.49F * spheroid.radius.x(), 0.F, 0.F}))
                .isInside());
    REQUIRE(signedDistance(spheroid, EulerAngles({0.F, 90.F, 0.F}),
                           spheroid.center + Vec3f({0.F, 0.99F * spheroid.radius.y(), 0.F}))
                .isInside());
    REQUIRE(signedDistance(spheroid, EulerAngles({0.F, 90.F, 0.F}),
                           spheroid.center + Vec3f({0.F, 1.01F * spheroid.radius.y(), 0.F}))
                .isOutside());
    REQUIRE(signedDistance(spheroid, EulerAngles({}), spheroid.center + 1.01F * spheroid.radius)
                .isOutside());
    REQUIRE(signedDistance(spheroid, EulerAngles({}), spheroid.center - 1.01F * spheroid.radius)
                .isOutside());
  }
  SECTION("Halfspace") {
    const Halfspace hs1({{1.F, 0.F, 0.F}, 10.F});
    REQUIRE(signedDistance(hs1, EulerAngles({}), Vec3f({9.F, 10.F, 1.0e6F})).isInside());
    REQUIRE(signedDistance(hs1, EulerAngles({}), Vec3f({11.F, -100.F, 0.F})).isOutside());
    const Halfspace hs2({{0.F, 0.707F, -0.707F}, 0.F});
    REQUIRE(signedDistance(hs1, EulerAngles({}), 0.99F * hs1.distance * hs1.normal).isInside());
    REQUIRE(signedDistance(hs2, EulerAngles({}), -0.1F * hs2.normal).isInside());
    REQUIRE(signedDistance(hs1, EulerAngles({}), 1.01F * hs1.distance * hs1.normal).isOutside());
    REQUIRE(signedDistance(hs2, EulerAngles({}), 0.1F * hs2.normal).isOutside());
  }

  SECTION("Addition and subtraction") {
    {
      const auto point = cuboid.center - 0.49F * cuboid.size;
      const auto sd1 = signedDistance(cuboid, EulerAngles({}), point);
      const auto sd2 = signedDistance(spheroid, EulerAngles({}), point);
      REQUIRE(sd1.isInside());
      REQUIRE(sd2.isOutside());
      REQUIRE((sd1 + sd2).isInside());
      REQUIRE((sd1 - sd2).isInside());
      REQUIRE((sd2 - sd1).isOutside());
    }
    {
      const auto point = spheroid.center;
      const auto sd1 = signedDistance(cuboid, EulerAngles({}), point);
      const auto sd2 = signedDistance(spheroid, EulerAngles({}), point);
      REQUIRE(sd1.isInside());
      REQUIRE(sd2.isInside());
      REQUIRE((sd1 + sd2).isInside());
      REQUIRE((sd1 - sd2).isOutside());
      REQUIRE((sd2 - sd1).isOutside());
    }
  }
}
