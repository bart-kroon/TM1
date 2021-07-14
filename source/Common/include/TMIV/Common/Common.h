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

#ifndef TMIV_COMMON_COMMON_H
#define TMIV_COMMON_COMMON_H

// Common data types and functions that are often used and do not need a separate header file

#include <TMIV/Common/verify.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

namespace TMIV::Common {
// http://open-std.org/JTC1/SC22/WG21/docs/papers/2018/p0051r3.pdf
template <typename... Ts> struct Overload : public Ts... {
  template <typename... Us> Overload(Us &&...values) : Ts{std::forward<Us>(values)}... {}
  using Ts::operator()...;
};
template <typename... Ts>
auto overload(Ts &&...values) -> Overload<std::remove_reference_t<Ts>...> {
  return {std::forward<Ts>(values)...};
}

// ISO/IEC 23090-5 supports upto 32-bit video data (depending on video codec support)
using SampleValue = uint_fast32_t;

// The maximum level for an unsigned integer of the specified number of bits
//
// The default return type is SampleValue. Override to avoid static_cast's at the call site.
template <typename UnsignedResult = SampleValue>
constexpr auto maxLevel(unsigned bits) noexcept -> UnsignedResult {
  static_assert(std::is_unsigned_v<UnsignedResult>);
  ASSERT(bits <= std::numeric_limits<UnsignedResult>::digits);
  return static_cast<UnsignedResult>((UnsignedResult{1} << bits) - 1U);
}

// Expand an integral value to floating-point using a linear transfer function
constexpr auto expandValue(SampleValue x, unsigned bits) -> float {
  ASSERT(x <= maxLevel(bits));
  ASSERT(0 < bits);
  return static_cast<float>(x) / static_cast<float>(maxLevel(bits));
}

// Quantize a value using a linear transfer function
template <typename UnsignedResult = SampleValue>
constexpr auto quantizeValue(float x, unsigned bits) -> UnsignedResult {
  const auto maxLevel_ = maxLevel<UnsignedResult>(bits);
  if (x >= 0.F && x < 1.F) {
    return static_cast<UnsignedResult>(std::llround(x * static_cast<float>(maxLevel_)));
  }
  if (1.F <= x) {
    return maxLevel_;
  }
  return {};
}

// Does a collection contain a specified value?
template <typename Collection, typename Value>
auto contains(const Collection &collection, Value &&value) -> bool {
  using std::cbegin;
  using std::cend;
  return std::any_of(cbegin(collection), cend(collection),
                     [&value](const auto &x) { return x == value; });
}
} // namespace TMIV::Common

#endif
