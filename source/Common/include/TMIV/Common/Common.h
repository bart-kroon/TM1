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

// ISO/IEC 23090-5 supports upto 32-bit video data (depending on video codec support) but to save on
// memory, the maximum bit depth is 16.
using DefaultElement = uint16_t;
static constexpr auto sampleBitDepth = std::numeric_limits<DefaultElement>::digits;

// The unsigned integer for performing calculations on sample values
using SampleValue = unsigned;
static_assert(sampleBitDepth <= std::numeric_limits<SampleValue>::digits);

// The maximum level for an uint32_t integer of the specified number of bits
//
// The default return type is SampleValue. Override to avoid static_cast's at the call site.
template <typename Integer = SampleValue>
constexpr auto maxLevel(uint32_t bits) noexcept -> Integer {
  static_assert(std::is_integral_v<Integer>);
  ASSERT(bits <= std::numeric_limits<Integer>::digits);
  constexpr auto one = std::make_unsigned_t<Integer>{1};
  return static_cast<Integer>((one << bits) - one);
}

// The medium level for an uint32_t integer of the specified number of bits
//
// The default return type is SampleValue. Override to avoid static_cast's at the call site.
template <typename Integer = SampleValue>
constexpr auto medLevel(uint32_t bits) noexcept -> Integer {
  static_assert(std::is_integral_v<Integer>);
  ASSERT(bits <= std::numeric_limits<Integer>::digits);
  constexpr auto one = std::make_unsigned_t<Integer>{1};
  return static_cast<Integer>(one << (bits - 1U));
}

// Expand an integral value to floating-point using a linear transfer function
constexpr auto expandValue(SampleValue x, uint32_t bits) -> float {
  ASSERT(x <= maxLevel(bits));
  ASSERT(0 < bits);
  return static_cast<float>(x) / static_cast<float>(maxLevel(bits));
}

// Quantize a value using a linear transfer function
template <typename UnsignedResult = SampleValue>
constexpr auto quantizeValue(float x, uint32_t bits) -> UnsignedResult {
  const auto maxLevel_ = maxLevel<UnsignedResult>(bits);
  if (x >= 0.F && x < 1.F) {
    return static_cast<UnsignedResult>(std::llround(x * static_cast<float>(maxLevel_)));
  }
  if (1.F <= x) {
    return maxLevel_;
  }
  return {};
}

// Shift x by |n| bits to the left (0 <= n) or right (n <= 0)
template <typename Integer, typename = std::enable_if_t<std::is_integral_v<Integer>>>
constexpr auto shift(Integer x, int32_t n) noexcept {
  if (n >= 0) {
    return x << n;
  }
  return x >> (-n);
}

static_assert(shift(5, 2) == 20);
static_assert(shift(5, 0) == 5);
static_assert(shift(20, -2) == 5);

// Specialize on element type: uint8_t, uint16_t or uint32_t (not exceeding SampleValue)
//
// The signature of the lambda is `f(auto zero)` with decltype(zero) the element type
template <typename F> constexpr auto withElement(uint32_t bitDepth, F f) -> decltype(auto) {
  PRECONDITION(0 < bitDepth);
  LIMITATION(bitDepth <= sampleBitDepth);

  if constexpr (16 < sampleBitDepth) {
    if (16 < bitDepth) {
      return f(uint32_t{});
    }
  }

  if constexpr (8 < sampleBitDepth) {
    if (8 < bitDepth) {
      return f(uint16_t{});
    }
  }

  return f(uint8_t{});
}

// Return the number of bytes used to represent a sample value of given bit depth in uncompressed
// video format, e.g. yuv420p -> 1, yuv420p10le -> 2
constexpr auto elementSize(uint32_t bitDepth) -> size_t {
  return withElement(bitDepth, [](auto zero) { return sizeof(zero); });
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
