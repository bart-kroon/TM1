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

#ifndef _TMIV_COMMON_MATH_H_
#define _TMIV_COMMON_MATH_H_

#include <cmath>
#include <complex>
#include <limits>

namespace TMIV::Common {

#ifndef M_PI
constexpr double M_PI = 3.141592653589793238462643383279;
#endif
constexpr double M_2PI = M_PI * 2.0;

inline double deg2rad(double x) { return x * M_PI / 180.; }
inline double rad2deg(double x) { return x * 180. / M_PI; }

template <typename T> T sqr(T val) { return val * val; }
template <typename T> T cube(T val) { return val * val * val; }
template <typename T> int sgn(T val) {
  return int(T(0) < val) - int(val < T(0));
}
template <typename T> T clamp(T val, T min, T max) {
  if (val < min) {
    return min;
  }
  if (max < val) {
    return max;
  }
  return val;
}
template <typename T> bool inRange(T val, T min, T max) {
  return ((min <= val) && (val <= max));
}
template <typename T> T is_zero(T val) {
  return (std::abs(val) < std::numeric_limits<T>::epsilon());
}
inline double squash(double a) {
  while (M_PI < a) {
    a -= M_2PI;
  }
  while (a < -M_PI) {
    a += M_2PI;
  }
  return a;
}
inline int ipow(int base, int exp) {
  int result = 1;
  for (;;) {
    if ((exp & 1) != 0) {
      result *= base;
    }
    exp >>= 1;
    if (exp == 0) {
      break;
    }
    base *= base;
  }
  return result;
}

inline int gcd(int a, int b) { return (b == 0) ? a : gcd(b, a % b); }
inline int lcm(int a, int b) { return std::abs(a * b) / gcd(a, b); }

inline double ppd2pps(double ppd) { return sqr(180. * ppd / M_PI); }
inline double pps2ppd(double pps) { return sqrt(pps) * M_PI / 180.; }

template <typename T,
          typename std::enable_if<std::is_integral<T>::value ||
                                      std::is_floating_point<T>::value,
                                  int>::type = 0>
T conjugate(T v) {
  return v;
}
template <typename T,
          typename std::enable_if<!(std::is_integral<T>::value ||
                                    std::is_floating_point<T>::value),
                                  int>::type = 0>
T conjugate(T v) {
  return std::conj(v);
}

template <typename T> T align(T value, T alignment) {
  T misalignment = value % alignment;
  return (misalignment != 0) ? (value + (alignment - misalignment)) : value;
}

} // namespace TMIV::Common

#endif
