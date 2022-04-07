/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/Common/Quaternion.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/LinAlg.h>

#include <cmath>

namespace TMIV::Common {
TEST_CASE("Quanternion<T>", "[quaternion]") {
  static_assert(std::is_same_v<QuatF, Quaternion<float>>);
  static_assert(std::is_same_v<QuatD, Quaternion<double>>);
  static_assert(std::is_same_v<QuatF, Vec4f>);
  static_assert(std::is_same_v<QuatD, Vec4d>);

  const auto p = QuatF{1.F, -2.F, 3.F, -4.F};     // some quaternion
  const auto q = QuatF{1.F, 3.F, 4.F, 7.F};       // another quaternion
  const auto r = QuatF{-0.2F, 0.4F, 0.4F, -0.8F}; // some versor
  const auto u = neutralOrientationF;             // zero rotation versor

  SECTION("Quaternion convention (x, y, z, w)") {
    // p = w + ix + jz + kz
    REQUIRE(p.x() == 1.F);
    REQUIRE(p.y() == -2.F);
    REQUIRE(p.z() == 3.F);
    REQUIRE(p.w() == -4.F);
  }

  SECTION("Quaternion norm") {
    REQUIRE(norm2(p) == 30.);
    REQUIRE(norm2(q) == 75.);
    REQUIRE(norm(u) == 1.F);
    REQUIRE(norm(r) == 1.F);
  }

  SECTION("Quaternion multiplication") {
    REQUIRE(p * conj(p) == QuatF{0.F, 0.F, 0.F, norm2(p)});
    REQUIRE(q * conj(q) == QuatF{0.F, 0.F, 0.F, norm2(q)});
    REQUIRE(p * q == QuatF{-14.F, -27.F, 10.F, -35.F});
    REQUIRE(q * p == QuatF{20.F, -25.F, 0.F, -35.F});
  }

  SECTION("Vector quaternion") { REQUIRE(quat(Vec3d{1., 2., 3.}) == QuatD{1., 2., 3., 0.}); }

  const auto eps = 1e-5F;

  SECTION("Rotate a vector with a versor") {
    const auto v = Vec3f{6.F, 3.F, 2.F};
    REQUIRE(rotate(v, u) == Vec3f{6.F, 3.F, 2.F});
    REQUIRE(norm(rotate(v, r) - Vec3f{2.F, -3.F, 6.F}) < eps);
    REQUIRE(norm(rotate(v, conj(r)) - Vec3f{0.72F, 5.96F, -3.6F}) < eps);
  }

  SECTION("Convert Euler angles to quaternion") {
    // Below examples are derived from the equation at
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion

    REQUIRE(norm(euler2quat(Vec3d{2., 0., 0.}) - QuatD{0., 0., std::sin(1.), std::cos(1.)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{0., 2., 0.}) - QuatD{0., std::sin(1.), 0., std::cos(1.)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{0., 0., 2.}) - QuatD{std::sin(1.), 0., 0., std::cos(1.)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{halfPi<double>, halfPi<double>, halfPi<double>}) -
                 QuatD{0., std::sqrt(0.5), 0., std::sqrt(0.5)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{halfPi<double>, -halfPi<double>, halfPi<double>}) -
                 QuatD{std::sqrt(0.5), 0., std::sqrt(0.5), 0.}) < eps);
  }

  SECTION("Convert Euler angles to quaternion (2)") {
    const auto yaw_rad = -0.4764713951;
    const auto pitch_rad = 0.0344346480;
    const auto roll_rad = 0.0204419943;
    const auto quat = euler2quat(Vec3d{yaw_rad, pitch_rad, roll_rad});

    // Mathematica 12.1 is the source of truth for below four numbers:
    // << Quaternions`
    // Euler2Quat[y_, p_, r_] :=
    //    Quaternion[Cos[y/2], 0, 0, Sin[y/2]] **
    //    Quaternion[Cos[p/2], 0, Sin[p/2], 0] **
    //    Quaternion[Cos[r/2], Sin[r/2], 0, 0]
    // SetPrecision[
    //    Euler2Quat[-0.4764713951, 0.0344346480, 0.0204419943], 16]

    REQUIRE(quat.x() == Approx(0.0139933465964437));
    REQUIRE(quat.y() == Approx(0.01431769616180822));
    REQUIRE(quat.z() == Approx(-0.2361122181339085));
    REQUIRE(quat.w() == Approx(0.9715195367443717));
  }

  SECTION("Convert quaternion to Euler angles") {
    const auto euler = quat2euler(
        QuatD{0.0139933465964437, 0.01431769616180822, -0.2361122181339085, 0.9715195367443717});

    CHECK(euler.x() == Approx(-0.4764713951)); // yaw [rad]
    CHECK(euler.y() == Approx(0.0344346480));  // pitch [rad]
    CHECK(euler.z() == Approx(0.0204419943));  // roll [rad]

    const auto euler2 = quat2euler(QuatD{-0.5, 0.5, 0.5, 0.5});

    CHECK(euler2.x() == Approx(1.570796326794896558)); // yaw [rad]
    CHECK(euler2.y() == Approx(1.570796326794896558)); // pitch [rad]
    CHECK(euler2.z() == Approx(0));                    // roll [rad]
  }

  SECTION("Convert quaternion to rotation matrix") {
    REQUIRE(rotationMatrix(u) == Mat3x3f::eye());

    const auto R = rotationMatrix(r);

    SECTION("Check matrix cells using another method") {
      for (int32_t i = 0; i < 3; ++i) {
        const auto p2 = Vec3d{static_cast<double>(i == 0), static_cast<double>(i == 1),
                              static_cast<double>(i == 2)};
        const auto rotate_p_by_r = rotate(p2, conj(r));
        for (int32_t j = 0; j < 3; ++j) {
          REQUIRE(R(i, j) == Approx(rotate_p_by_r[j]).margin(1E-6));
        }
      }
    }

    SECTION("Trial rotation") {
      const auto v = Vec3f{12.F, -7.F, 9.F};

      REQUIRE(norm(R * v - rotate(v, r)) < eps);
    }
  }
}

static auto directAveragingOfOrientationsDegrees(const std::vector<Vec3d> &orientationsDegrees)
    -> Vec3d {
  std::vector<Common::QuatD> orientations;
  orientations.reserve(orientationsDegrees.size());
  for (const auto &eulerd : orientationsDegrees) {
    orientations.push_back(Common::eulerDeg2quat(eulerd));
  }
  auto orientationAverage = directAveragingOfOrientations(orientations);
  auto orientationAverageDegrees = Common::quat2eulerDeg(orientationAverage);

  return orientationAverageDegrees;
}

// This test evaluates the precision of the implemented direct method for averaging orientations
// against the more accurate and more complex maximum likelihood method.
// The expected values are generated using Matlab meanrot() function
TEST_CASE("DirectAveragingOfOrientations") {
  const double smallAngleDegrees = 1.0;
  const double verySmallAngleDegrees = 0.1;

  SECTION("Average orientation example") {
    std::vector<Vec3d> orientationsDegrees = {{40, 20, 10}, {50, 10, 5}, {45, 70, 1}};
    auto orientationAverageDegreesExpected = Vec3d{45.7876, 32.6452, 6.0407};
    auto orientationAverageDegreesActual =
        directAveragingOfOrientationsDegrees(orientationsDegrees);
    auto difference =
        Common::norm(orientationAverageDegreesExpected - orientationAverageDegreesActual);

    REQUIRE(difference < smallAngleDegrees);
  }

  SECTION("Average orientation example") {
    std::vector<Vec3d> orientationsDegrees = {{40, 20, 10}, {50, 350, 5}, {45, 70, 1}};
    auto orientationAverageDegreesExpected = Vec3d{45.7158, 25.9763, 6.3364};
    auto orientationAverageDegreesActual =
        directAveragingOfOrientationsDegrees(orientationsDegrees);
    auto difference =
        Common::norm(orientationAverageDegreesExpected - orientationAverageDegreesActual);

    REQUIRE(difference < smallAngleDegrees);
  }

  SECTION("Average orientation of real camera rig") {
    // Use the camera setup of test sequence 'S' as a realistic example
    std::vector<Vec3d> orientationsDegrees = {
        {40.2684, -3.81838, 132.56},    {38.5782, -5.7702, 129.004},
        {37.1499, -8.23096, 126.987},   {35.0126, -10.0534, 122.19},
        {34.1317, -13.3612, 121.78},    {32.1488, -15.4979, 117.9},
        {29.9645, -18.9061, 118.087},   {29.165, -21.9032, 116.952},
        {27.6615, -25.2889, 114.787},   {26.2221, -29.6156, 112.98},
        {23.7379, -31.5641, 110.607},   {21.5129, -35.3238, 110.244},
        {19.9639, -37.8383, 108.283},   {17.4955, -41.5014, 108.827},
        {15.5377, -44.2564, 103.857},   {14.2657, -48.203, 104.231},
        {11.0674, -54.3679, 104.882},   {11.2173, -54.3154, 105.835},
        {8.73715, -58.845, 100.842},    {6.50795, -62.8814, 102.931},
        {4.50941, -67.5603, 104.599},   {3.49002, -70.983, 101.682},
        {2.3553, -74.5453, 99.9127},    {4.02029, -78.5012, 97.4278},
        {2.06328, -81.5449, 94.4807},   {2.22225, -85.9019, 94.0953},
        {-128.387, -89.5978, -135.637}, {-173.897, -87.9799, -96.0767},
        {-178.197, -84.3768, -92.716},  {-175.99, -83.3803, -100.826}};

    auto orientationAverageDegreesExpected = Vec3d{15.8680, -47.5520, 105.7264};
    auto orientationAverageDegreesActual =
        directAveragingOfOrientationsDegrees(orientationsDegrees);
    auto difference =
        Common::norm(orientationAverageDegreesExpected - orientationAverageDegreesActual);

    REQUIRE(difference < verySmallAngleDegrees);
  }
}

} // namespace TMIV::Common
