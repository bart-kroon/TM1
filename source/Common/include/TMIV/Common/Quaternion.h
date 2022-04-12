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

#ifndef TMIV_COMMON_QUATERNION_H
#define TMIV_COMMON_QUATERNION_H

#include "Math.h"
#include "Matrix.h"
#include "Vector.h"
#include "verify.h"

#include <cassert>
#include <type_traits>

namespace TMIV::Common {
// Quaternion: q = w + ix + jy + kz with i^2 = j^2 = k^2 = ijk = -1
template <typename Float> using Quaternion = stack::Vec4<Float>;
using QuatF = Quaternion<float>;
using QuatD = Quaternion<double>;
const auto neutralOrientationF = QuatF{0.F, 0.F, 0.F, 1.F};
const auto neutralOrientationD = QuatD{0., 0., 0., 1.};

// Quaternion product: a b
template <typename Float1, typename Float2>
auto operator*(const Quaternion<Float1> &a, const Quaternion<Float2> &b) {
  using R = std::common_type_t<Float1, Float2>;
  return Quaternion<R>{
      a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(), // x
      a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(), // y
      a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w(), // z
      a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z()  // w
  };
}

// Quaternion conjugent: q -> w - ix - jy - kz
template <typename Float> auto conj(const Quaternion<Float> &q) {
  return Quaternion<Float>{-q.x(), -q.y(), -q.z(), q.w()};
}

// Normalize a quaternion
//
// NOTE(#335): The new design principle is to assume quaternions are not normalized, and normalize
// them at the point where this is a requirement (rotate, quat2euler, rotationMatrix, encoding).
template <typename Float> auto normalize(Quaternion<Float> q) noexcept {
  static_assert(std::is_floating_point_v<Float>);
  const auto q2 = norm2(q);
  PRECONDITION(Float{} < q2);
  return q / std::sqrt(q2);
}

// Vector quaternion: v = (x, y, z) -> ix + jy + kz
template <typename Float> auto quat(const stack::Vec3<Float> &v) {
  return Quaternion<Float>{v.x(), v.y(), v.z(), 0.F};
}

// Conjugate quaternion p by unit quaternion q: qpq^-1 = qpq* / qq*
template <typename Float1, typename Float2>
auto rotate(const Quaternion<Float1> &p, const Quaternion<Float2> &q) {
  static_assert(std::is_floating_point_v<Float1> && std::is_floating_point_v<Float2>);
  return (q * p * conj(q)) / norm2(q);
}

// Rotate a vector by a unit quaternion
template <typename Float1, typename Float2>
auto rotate(const stack::Vec3<Float1> &v, const Quaternion<Float2> &q) {
  const auto p = rotate(quat(v), q);
  using Float3 = typename decltype(p)::value_type;
  return stack::Vec3<Float3>{p.x(), p.y(), p.z()};
}

// Euler angles (yaw, pitch, roll) [rad] to quaternion conversion
//
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
template <typename Float> auto euler2quat(const stack::Vec3<Float> &eulerAngles) {
  static_assert(std::is_floating_point_v<Float>);

  using Q = Quaternion<Float>;
  using std::cos;
  using std::sin;

  const auto y = eulerAngles[0]; // yaw rotation [rad]
  const auto p = eulerAngles[1]; // pitch rotation [rad]
  const auto r = eulerAngles[2]; // roll rotation [rad]

  const auto qy = Q{0.F, 0.F, sin(0.5F * y), cos(0.5F * y)};
  const auto qp = Q{0.F, sin(0.5F * p), 0.F, cos(0.5F * p)};
  const auto qr = Q{sin(0.5F * r), 0.F, 0.F, cos(0.5F * r)};

  return qy * qp * qr;
}

template <typename Float> auto quat2euler(Quaternion<Float> q) {
  q = normalize(q);

  const auto cYaw = sqr(q.x()) - sqr(q.y()) - sqr(q.z()) + sqr(q.w());
  const auto sYaw = 2.F * (q.w() * q.z() + q.x() * q.y());
  auto yaw = std::atan2(sYaw, cYaw);
  if (std::abs(sYaw) < 1e-6 && std::abs(cYaw) < 1e-6) {
    yaw = std::atan2(q.w() + q.x() + q.y() + q.z(), 0.0);
  }

  const auto sPitch = 2.F * (q.w() * q.y() - q.z() * q.x());
  const auto pitch =
      std::abs(sPitch) < 1.F ? std::asin(sPitch) : std::copysign(halfPi<Float>, sPitch);

  const auto cRoll = -sqr(q.x()) - sqr(q.y()) + sqr(q.z()) + sqr(q.w());
  const auto sRoll = 2.F * (q.w() * q.x() + q.y() * q.z());
  auto roll = std::atan2(sRoll, cRoll);
  if (std::abs(sRoll) < 1e-6 && std::abs(cRoll) < 1e-6) {
    roll = 0.0;
  }

  return stack::Vec3<Float>{yaw, pitch, roll};
}

// Euler angles (yaw, pitch, roll) [deg] to quaternion conversion
//
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
template <typename Float> auto eulerDeg2quat(stack::Vec3<Float> eulerAngles) {
  for (auto &component : eulerAngles) {
    component = deg2rad(component);
  }
  return euler2quat(eulerAngles);
}

template <typename Float> auto quat2eulerDeg(Quaternion<Float> q) {
  auto eulerAngles = quat2euler(q);
  for (auto &component : eulerAngles) {
    component = rad2deg(component);
  }
  return eulerAngles;
}

// Unit quaternion (q) to rotation matrix (R) conversion
//
// The matrix R has the following property:
//
//    quat(Rv) == rotate(v, q))
template <typename Float> auto rotationMatrix(Quaternion<Float> q) {
  q = normalize(q);

  return stack::Mat3x3<Float>{1.F - 2.F * (q.y() * q.y() + q.z() * q.z()),  // R_xx
                              2.F * (q.x() * q.y() - q.z() * q.w()),        // R_xy
                              2.F * (q.z() * q.x() + q.y() * q.w()),        // R_xz
                              2.F * (q.x() * q.y() + q.z() * q.w()),        // R_yx
                              1.F - 2.F * (q.z() * q.z() + q.x() * q.x()),  // R_yy
                              2.F * (q.y() * q.z() - q.x() * q.w()),        // R_yz
                              2.F * (q.z() * q.x() - q.y() * q.w()),        // R_zx
                              2.F * (q.y() * q.z() + q.x() * q.w()),        // R_zy
                              1.F - 2.F * (q.x() * q.x() + q.y() * q.y())}; // R_zz
}

// An aligned (negated) quaternion represents the same orientation
inline auto alignQuaternion(const Common::QuatD &quatRef, const Common::QuatD &quat)
    -> Common::QuatD {
  auto quadsAreAligned = Common::dot_product(quatRef.begin(), quatRef.end(), quat.begin()) >= 0.0;

  return quadsAreAligned ? quat : Common::QuatD{-quat.x(), -quat.y(), -quat.z(), -quat.w()};
}

// Note that this simple approximation has limited accuracy
// Quaternions are directly averaged while a correction is applied for the double-cover problem
// where -q and q represent the same orientation.
// See https://math.stackexchange.com/questions/61146/averaging-quaternions
inline auto directAveragingOfOrientations(const std::vector<Common::QuatD> &normalizedQuats)
    -> Common::QuatD {
  PRECONDITION(!normalizedQuats.empty());

  auto result = normalizedQuats[0];

  for (auto k = 1U; k < normalizedQuats.size(); k++) {
    auto quatAligned = alignQuaternion(normalizedQuats[0], normalizedQuats[k]);
    result += quatAligned;
  }

  result = result / static_cast<double>(normalizedQuats.size());
  result = Common::normalize(result);

  return result;
}

} // namespace TMIV::Common

#endif
