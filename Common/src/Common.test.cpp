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

#include <TMIV/Common/Common.h>
#include <TMIV/Common/LinAlg.h>

#include <iostream>

namespace TMIV::Common {

TEST_CASE("Array, Vector, Matrix, LinAlg") {
  const float EPS = 1e-5;

  heap::Array<3, float> A({3, 5, 10});
  Mat3x3f m1 = {1., 2., 3., 4., 5., 6., 7., 8., -9.};
  Mat<float> m2(
      {4, 3}, {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f, 11.f, 12.f});
  Vec3f v1({1.f, 2.f, 3.f}), v2({3.f, -1.f, 6.f});

  SECTION("Array")
  REQUIRE(A.size() == 150);

  SECTION("Vector norm")
  REQUIRE(fabs(norm(v1) - 3.7416573f) < EPS);

  SECTION("Unit vector")
  REQUIRE(fabs(norm(unit(v1)) - 1.f) < EPS);

  SECTION("Vector dot product")
  REQUIRE(fabs(dot(v1, v1) - 14.f) < EPS);

  SECTION("Vector cross product")
  REQUIRE(norm_inf(cross(v1, v2) - Vec3f({15.f, 3.f, -7.f})) < EPS);

  SECTION("Matrix trace")
  REQUIRE(fabs(trace(m1) - (-3.f)) < EPS);

  SECTION("Matrix transpose")
  REQUIRE(norm_inf(transpose(m2) -
                   Mat<float>({3, 4}, {1.f, 4.f, 7.f, 10.f, 2.f, 5.f, 8.f, 11.f,
                                       3.f, 6.f, 9.f, 12.f})) < EPS);

  SECTION("Matrix / Vector product")
  REQUIRE(norm_inf((m1 * v1) - Vec3f({14.f, 32.f, -4.f})) < EPS);

  SECTION("Matrix / Matrix product")
  REQUIRE(
      norm_inf((m2 * m1) -
               Mat<float>({4, 3}, {30.f, 36.f, -12.f, 66.f, 81.f, -12.f, 102.f,
                                   126.f, -12.f, 138.f, 171.f, -12.f})) < EPS);

  SECTION("Matrix inverse")
  REQUIRE(norm_inf((m1 * inv(m1)) - Mat3x3f::eye()) < EPS);

  SECTION("Linear system (right)")
  REQUIRE(norm_inf(mrdivide(m2, m1) -
                   Mat<float>({4, 3}, {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, -1.f, 2.f,
                                       0.f, -2.f, 3.f, 0.f})) < EPS);

  SECTION("Linear system (left)")
  REQUIRE(norm_inf(mldivide(m1, v1) - Vec3f({-1.f, 2.f, 0.f}) / 3.f) < EPS);

  SECTION("Matrix determinant")
  REQUIRE(fabs(det(m1) - 54.f) < EPS);

  std::fill(m1.diag_begin(), m1.diag_end(), 0.f);
  SECTION("Matrix iterator")
  REQUIRE(fabs(trace(m1)) < EPS);
}

} // namespace TMIV::Common
