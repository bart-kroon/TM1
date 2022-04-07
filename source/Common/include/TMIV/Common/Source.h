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

#ifndef TMIV_COMMON_SOURCE_H
#define TMIV_COMMON_SOURCE_H

#include <functional>
#include <iterator>
#include <optional>
#include <type_traits>
#include <utility>

#include <TMIV/Common/verify.h>

namespace TMIV::Common {
// An infinite stream of T's (stream comonad)
// There is a null state (maybe monad)
template <typename T> using Stream = std::function<T()>;

// A source of T's is a finite stream of T's coded as: { {T}, ..., {T}, nullopt, ... }
template <typename T> using Source = Stream<std::optional<T>>;

// emptySource() returns a source oF T's of length zero: { nullopt, ... }
template <typename T> [[nodiscard]] constexpr auto emptySource() noexcept -> Source<T> {
  return []() -> std::optional<T> { return std::nullopt; };
}

// uniformSource(size_t n, T x) returns a source of length n: { x, ..., x, nullopt, ... }
template <typename T>
[[nodiscard]] constexpr auto uniformSource(size_t n, const T &x) -> Source<T> {
  // NOLINTNEXTLINE(bugprone-exception-escape)
  return [n, x]() mutable -> std::optional<T> {
    if (n == 0) {
      return std::nullopt;
    }
    --n;
    return x;
  };
}

template <typename Iterator, typename T = typename std::iterator_traits<Iterator>::value_type>
[[nodiscard]] constexpr auto sourceFromIteratorPair(Iterator first, Iterator last) noexcept
    -> Source<T> {
  return [=]() mutable -> std::optional<T> {
    if (first == last) {
      return std::nullopt;
    }
    return *first++;
  };
}

// ReadAhead adapts a non-null stream to memorize the head element
template <typename T> class ReadAhead {
public:
  constexpr explicit ReadAhead(Stream<T> stream) : m_head{stream()}, m_tail{std::move(stream)} {}

  [[nodiscard]] constexpr auto head() const noexcept -> const auto & { return m_head; }

  constexpr auto operator()() -> T { return std::exchange(m_head, m_tail()); }

private:
  T m_head;
  Stream<T> m_tail;
};

// test(source) returns an isomorphic stream or null if the source is null or empty
template <typename T> auto test(Source<T> source) -> Source<T> {
  if (source == nullptr) {
    return nullptr;
  }

  auto readAhead = std::make_shared<ReadAhead<std::optional<T>>>(std::move(source));

  if (readAhead->head()) {
    return [readAhead]() mutable { return (*readAhead)(); };
  }
  return nullptr;
}
} // namespace TMIV::Common

#endif
