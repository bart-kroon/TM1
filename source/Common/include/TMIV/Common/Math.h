/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_COMMON_MATH_H
#define TMIV_COMMON_MATH_H

#include <cmath>
#include <complex>
#include <limits>

namespace TMIV::Common {
template <typename Float, typename = std::enable_if_t<std::is_floating_point_v<Float>>>
constexpr Float pi = static_cast<Float>(3.1415926535897932384626433832795);
template <typename Float> constexpr Float halfPi = Float{0.5} * pi<Float>;
template <typename Float> constexpr Float twoPi = Float{2} * pi<Float>;

template <typename T> constexpr auto deg2rad(T x) noexcept {
  return x * static_cast<T>(pi<double> / 180.);
}

template <typename T> constexpr auto rad2deg(T x) noexcept {
  return x * static_cast<T>(180. / pi<double>);
}

template <typename T> constexpr auto sqr(T val) noexcept { return val * val; }

template <typename T> constexpr auto inRange(T val, T min, T max) noexcept {
  return min <= val && val <= max;
}

template <typename T> constexpr auto pps2ppd(T pps) noexcept { return deg2rad(std::sqrt(pps)); }

template <typename T> constexpr auto align(T value, T alignment) noexcept {
  T misalignment = value % alignment;
  return (misalignment != 0) ? (value + (alignment - misalignment)) : value;
}

} // namespace TMIV::Common

#endif
