/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_COMMON_NUMBER_H
#define TMIV_COMMON_NUMBER_H

#include "verify.h"

#include <cmath>
#include <limits>
#include <type_traits>

namespace TMIV::Common {
template <typename Integral, typename FloatingPoint,
          typename = std::enable_if_t<std::is_integral_v<Integral> &&
                                      std::is_floating_point_v<FloatingPoint>>>
[[nodiscard]] auto floatingPointIsWithinIntegralRange(FloatingPoint x) {
  static constexpr auto min = static_cast<FloatingPoint>(std::numeric_limits<Integral>::min());
  static constexpr auto max = static_cast<FloatingPoint>(std::numeric_limits<Integral>::max());

  return min <= x && x <= max;
}

// TODO(mpeg143-ce2): Apply in the entire code base (search for std::.*round)
template <typename Integral, typename FloatingPoint,
          typename = std::enable_if_t<std::is_integral_v<Integral> &&
                                      std::is_floating_point_v<FloatingPoint>>>
[[nodiscard]] auto iround(FloatingPoint x) {
  PRECONDITION(floatingPointIsWithinIntegralRange<Integral>(x));

  return static_cast<Integral>(std::llround(x));
}

// TODO(mpeg143-ce2): Apply in the entire code base (search for std::floor)
template <typename Integral, typename FloatingPoint,
          typename = std::enable_if_t<std::is_integral_v<Integral> &&
                                      std::is_floating_point_v<FloatingPoint>>>
[[nodiscard]] auto ifloor(FloatingPoint x) {
  PRECONDITION(floatingPointIsWithinIntegralRange<Integral>(x));

  return static_cast<Integral>(std::llround(std::floor(x)));
}

// TODO(mpeg143-ce2): Apply in the entire code base (search for std::ceil)
template <typename Integral, typename FloatingPoint,
          typename = std::enable_if_t<std::is_integral_v<Integral> &&
                                      std::is_floating_point_v<FloatingPoint>>>
[[nodiscard]] auto iceil(FloatingPoint x) {
  PRECONDITION(floatingPointIsWithinIntegralRange<Integral>(x));

  return static_cast<Integral>(std::llround(std::ceil(x)));
}
} // namespace TMIV::Common

#endif
