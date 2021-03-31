/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#ifndef TMIV_COMMON_ALGORITHM_H
#define TMIV_COMMON_ALGORITHM_H

#include <algorithm>

// This header provides commonly used algorithms like in <algorithm> and <numeric> but with
// `constexpr` and `noexcept` guarantees.

namespace TMIV::Common::cx {
// https://en.cppreference.com/w/cpp/algorithm/fill (1)
template <typename ForwardIt, typename T>
constexpr void fill(ForwardIt first, ForwardIt last, const T &value) noexcept {
  for (; first != last; ++first) {
    *first = value;
  }
}

// https://en.cppreference.com/w/cpp/algorithm/copy (1)
template <typename InputIt, typename OutputIt>
constexpr auto copy(InputIt first, InputIt last, OutputIt out) noexcept {
  while (first != last) {
    *out++ = *first++;
  }
  return out;
}

// https://en.cppreference.com/w/cpp/algorithm/transform (1)
template <typename InputIt, typename OutputIt, typename UnaryOperation>
constexpr auto transform(InputIt first, InputIt last, OutputIt out, UnaryOperation op) noexcept {
  while (first != last) {
    *out++ = op(*first++);
  }
  return out;
}

// https://en.cppreference.com/w/cpp/algorithm/transform (3)
template <typename InputIt1, typename InputIt2, typename OutputIt, typename BinaryOperation>
constexpr auto transform(InputIt1 first1, InputIt1 last1, InputIt2 first2, OutputIt out,
                         BinaryOperation op) noexcept {
  while (first1 != last1) {
    *out++ = op(*first1++, *first2++);
  }
  return out;
}

// https://en.cppreference.com/w/cpp/algorithm/equal (1)
template <typename InputIt1, typename InputIt2>
constexpr auto equal(InputIt1 first1, InputIt1 last1, InputIt2 first2) noexcept {
  for (; first1 != last1; ++first1, ++first2) {
    if (!(*first1 == *first2)) {
      return false;
    }
  }
  return true;
}

// https://en.cppreference.com/w/cpp/algorithm/swap (1)
template <typename T> constexpr void swap(T &a, T &b) noexcept {
  auto c = std::move(a);
  a = std::move(b);
  b = std::move(c);
}
} // namespace TMIV::Common::cx

#endif
