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

#ifndef TMIV_COMMON_ARRAY_H
#define TMIV_COMMON_ARRAY_H

#include <TMIV/Common/Algorithm.h>
#include <TMIV/Common/Traits.h>
#include <TMIV/Common/verify.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <iterator>
#include <numeric>
#include <ostream>
#include <vector>

namespace TMIV::Common {
namespace Array {
template <typename Iter> class SteppedIterator : public std::iterator_traits<Iter> {
private:
  static_assert(
      std::is_same_v<typename SteppedIterator::iterator_category, std::random_access_iterator_tag>);

  Iter m_begin;
  ptrdiff_t m_offset;
  ptrdiff_t m_step;

public:
  constexpr explicit SteppedIterator(Iter begin, ptrdiff_t offset, ptrdiff_t step) noexcept
      : m_begin{begin}, m_offset{offset}, m_step{step} {
    assert(0 < step);
  }

  [[deprecated]] [[nodiscard]] auto n() const noexcept { return m_step; }

  [[nodiscard]] constexpr auto operator==(const SteppedIterator &rhs) const noexcept {
    assert(m_begin == rhs.m_begin);
    return m_offset == rhs.m_offset;
  }

  [[nodiscard]] constexpr auto operator!=(const SteppedIterator &rhs) const noexcept {
    return !this->operator==(rhs);
  }

  [[nodiscard]] constexpr auto operator<(const SteppedIterator &rhs) const noexcept {
    assert(m_begin == rhs.m_begin);
    return m_offset < rhs.m_offset;
  }

  [[nodiscard]] constexpr auto operator>(const SteppedIterator &rhs) const noexcept {
    return rhs < *this;
  }

  [[nodiscard]] constexpr auto operator<=(const SteppedIterator &rhs) const noexcept {
    return !(*this > rhs);
  }

  [[nodiscard]] constexpr auto operator>=(const SteppedIterator &rhs) const noexcept {
    return !(*this < rhs);
  }

  [[nodiscard]] constexpr auto operator*() const noexcept -> decltype(auto) {
    return m_begin[m_offset];
  }

  [[nodiscard]] constexpr auto operator->() const noexcept -> decltype(auto) {
    return &m_begin[m_offset];
  }

  constexpr auto operator++() noexcept -> decltype(auto) {
    m_offset += m_step;
    return *this;
  }

  constexpr auto operator++(int) noexcept {
    SteppedIterator out(*this);
    operator++();
    return out;
  }

  constexpr auto operator--() noexcept -> decltype(auto) {
    m_offset -= m_step;
    return *this;
  }

  constexpr auto operator--(int) noexcept {
    SteppedIterator out(*this);
    operator--();
    return out;
  }

  [[nodiscard]] constexpr auto operator+(ptrdiff_t a) const noexcept {
    return SteppedIterator(m_begin, m_offset + a * m_step, this->m_step);
  }

  constexpr auto operator+=(ptrdiff_t a) noexcept -> decltype(auto) {
    m_offset += a * m_step;
    return *this;
  }

  [[nodiscard]] constexpr auto operator-(const SteppedIterator &iter) const noexcept {
    assert(0 < m_step);
    return (m_offset - iter.m_offset) / m_step;
  }

  [[nodiscard]] constexpr auto operator-(ptrdiff_t a) const noexcept -> decltype(auto) {
    return SteppedIterator(m_begin, m_offset - a * m_step, this->m_step);
  }

  constexpr auto operator-=(ptrdiff_t a) noexcept -> decltype(auto) {
    m_offset -= a * m_step;
    return *this;
  }

  [[nodiscard]] constexpr auto operator[](ptrdiff_t a) const noexcept -> decltype(auto) {
    return m_begin[m_offset + a * m_step];
  }

  constexpr void swap(SteppedIterator &a, SteppedIterator &b) noexcept { std::swap(a, b); }
};

template <typename Iter>
constexpr auto operator+(ptrdiff_t a, const SteppedIterator<Iter> &rhs) noexcept {
  return rhs + a;
}
} // namespace Array

namespace stack {
template <typename T, size_t M, size_t... N> struct Array {
private:
  static constexpr auto m_dim = 1 + sizeof...(N);
  static constexpr auto m_size = (M * ... * N);
  static constexpr auto m_sizes = std::array<size_t, m_dim>{M, N...};
  static constexpr auto m_minSize = std::min({M, N...});

  static constexpr auto m_step = []() {
    auto result = std::array<size_t, m_dim>{};

    result[m_dim - 1] = 1;

    for (ptrdiff_t i = m_dim - 1; 0 < i; --i) {
      result[i - 1] = result[i] * m_sizes[i];
    }
    return result;
  }();

  static constexpr auto m_diagStep = []() {
    auto result = size_t{};

    for (size_t i = 0; i < m_dim; ++i) {
      result += m_step[i];
    }
    return result;
  }();

  template <size_t L> static constexpr auto position(const std::array<size_t, L> &index) noexcept {
    auto result = size_t{};

    for (size_t i = 0; i < m_dim && i < L; ++i) {
      result += m_step[i] * index[i];
    }
#ifndef NDEBUG
    for (size_t i = m_dim; i < L; ++i) {
      assert(index[i] == 0);
    }
#endif
    return result;
  }

  // Like position() but the K'th element is at the front
  template <size_t K>
  [[nodiscard]] static constexpr auto offset(const std::array<size_t, m_dim> &index) noexcept {
    auto result = m_step[K] * index.front();

    if constexpr (0 < K) {
      for (size_t i = 0; i < K; ++i) {
        result += m_step[i] * index[1 + i];
      }
    }
    for (size_t i = K + 1; i < m_dim; ++i) {
      result += m_step[i] * index[i];
    }
    return result;
  }

  using InternalArray = std::array<T, (M * ... * N)>;

  InternalArray m_v{};

  template <typename OtherArray> constexpr auto isEquallyShaped(const OtherArray &that) noexcept {
    const auto minDim = std::min(dim(), that.dim());

    for (size_t i = 0; i < minDim; ++i) {
      if (m_sizes[i] != that.size(i)) {
        return false;
      }
    }
    for (size_t i = minDim; i < m_dim; ++i) {
      if (m_sizes[i] != 1) {
        return false;
      }
    }
    for (size_t i = minDim; i < that.dim(); ++i) {
      if (that.size(i) != 1) {
        return false;
      }
    }
    return true;
  }

  struct Convert {
    template <typename U> auto operator()(const U &x) noexcept {
      static_assert(std::is_convertible_v<decltype(x), T>);

      if constexpr (std::is_arithmetic_v<decltype(x)> && std::is_integral_v<T>) {
        return Common::assertDownCast<T>(x);
      } else {
        return static_cast<T>(x);
      }
    }
  };

public:
  using value_type = typename InternalArray::value_type;
  using iterator = typename InternalArray::iterator;
  using const_iterator = typename InternalArray::const_iterator;
  using dim_iterator = TMIV::Common::Array::SteppedIterator<iterator>;
  using const_dim_iterator = TMIV::Common::Array::SteppedIterator<const_iterator>;
  using diag_iterator = dim_iterator;
  using const_diag_iterator = const_dim_iterator;
  using container_type = Array<T, M, N...>;
  using tuple_type = std::array<size_t, m_dim>;
  template <typename U> using promoted_type = Array<std::common_type_t<T, U>, M, N...>;

  Array() = default;

  explicit constexpr Array(const T &t) noexcept { cx::fill(begin(), end(), t); }

  constexpr Array(std::initializer_list<T> init) noexcept {
    cx::copy(init.begin(), init.end(), begin());
  }

  // TODO(#488): Make promotion implicit and create another function for casting
  template <typename OtherArray, typename = typename OtherArray::dim_iterator,
            typename = std::enable_if_t<!std::is_same_v<Array, OtherArray>>>
  explicit Array(const OtherArray &that) noexcept : Array{} {
    assert(isEquallyShaped(that));
    cx::transform(that.cbegin(), that.cend(), begin(), Convert{});
  }

  constexpr auto operator=(const T &t) noexcept -> auto & {
    cx::fill(begin(), end(), t);
    return *this;
  }

  constexpr auto operator=(std::initializer_list<T> init) noexcept -> auto & {
    cx::copy(init.begin(), init.end(), begin());
    return *this;
  }

  template <typename OtherArray, typename = typename OtherArray::dim_iterator,
            typename = std::enable_if_t<!std::is_same_v<Array, OtherArray>>>
  constexpr auto operator=(const OtherArray &that) noexcept -> auto & {
    assert(isEquallyShaped(that));
    cx::transform(that.cbegin(), that.cend(), begin(), Convert{});
    return *this;
  }

  [[nodiscard]] constexpr auto operator==(const Array &that) const noexcept {
    return cx::equal(cbegin(), cend(), that.cbegin());
  }

  [[nodiscard]] constexpr auto operator!=(const Array &that) const noexcept {
    return !operator==(that);
  }

  constexpr void swap(Array &that) noexcept { return cx::swap(*this, that); }

  static constexpr void resize([[maybe_unused]] const tuple_type &newSize) noexcept {
    assert(newSize == m_sizes);
  }

  static constexpr void reshape([[maybe_unused]] const tuple_type &newSize) noexcept {
    assert(newSize == m_sizes);
  }

  [[nodiscard]] static constexpr auto dim() noexcept { return m_dim; }

  [[nodiscard]] static constexpr auto size() noexcept { return m_size; }
  [[nodiscard]] static constexpr auto size(size_t i) noexcept { return m_sizes[i]; }

  [[nodiscard]] static constexpr auto sizes() noexcept { return m_sizes; }

  // Returns the gap between two consecutive elements on the ith dimension
  [[nodiscard]] static constexpr auto step(size_t i) noexcept {
    auto result = size_t{1};

    for (size_t j = m_dim - 1; i < j; --j) {
      result *= m_sizes[j];
    }
    return result;
  }

  [[nodiscard]] constexpr auto data() noexcept -> auto * { return m_v.data(); }
  [[nodiscard]] constexpr auto data() const noexcept -> const auto * { return m_v.data(); }

  [[nodiscard]] constexpr auto operator[](size_t k) noexcept -> decltype(auto) { return m_v[k]; }
  [[nodiscard]] constexpr auto operator[](size_t k) const noexcept -> decltype(auto) {
    return m_v[k];
  }

  [[nodiscard]] constexpr auto begin() noexcept { return m_v.begin(); }
  [[nodiscard]] constexpr auto begin() const noexcept { return m_v.begin(); }
  [[nodiscard]] constexpr auto cbegin() const noexcept { return m_v.cbegin(); }
  [[nodiscard]] constexpr auto end() noexcept { return m_v.end(); }
  [[nodiscard]] constexpr auto end() const noexcept { return m_v.end(); }
  [[nodiscard]] constexpr auto cend() const noexcept { return m_v.cend(); }

  // Returns an iterator along the Kth dimension to the element of the hyperplane defined by i
  template <size_t K, typename... SizeT>
  [[nodiscard]] constexpr auto dim_begin(SizeT... i) const noexcept {
    return const_dim_iterator(m_v.begin(), offset<K>({0, i...}), m_step[K]);
  }

  // Returns an iterator along the Kth dimension to the element of the hyperplane defined by i
  template <size_t K, typename... SizeT>
  [[nodiscard]] constexpr auto dim_begin(SizeT... i) noexcept {
    return dim_iterator(m_v.begin(), offset<K>({0, i...}), m_step[K]);
  }

  // Returns an iterator along the Kth dimension to the element of the hyperplane defined by i
  template <size_t K, typename... SizeT>
  [[nodiscard]] constexpr auto cdim_begin(SizeT... i) const noexcept {
    return dim_begin<K>(i...);
  }

  // Returns an iterator along the Kth dimension to the end of the hyperplane
  template <size_t K, typename... SizeT>
  [[nodiscard]] constexpr auto dim_end(SizeT... i) const noexcept {
    return const_dim_iterator(m_v.begin(), offset<K>({m_sizes[K], i...}), m_step[K]);
  }

  // Returns an iterator along the Kth dimension to the end of the hyperplane
  template <size_t K, typename... SizeT> [[nodiscard]] constexpr auto dim_end(SizeT... i) noexcept {
    return dim_iterator(m_v.begin(), offset<K>({m_sizes[K], i...}), m_step[K]);
  }

  // Returns an iterator along the Kth dimension to the end of the hyperplane
  template <size_t K, typename... SizeT>
  [[nodiscard]] constexpr auto cdim_end(SizeT... i) const noexcept {
    return dim_end<K>(i...);
  }

  // Returns an iterator to the first diagonal element
  [[nodiscard]] constexpr auto diag_begin() const noexcept {
    return const_diag_iterator(m_v.begin(), 0, m_diagStep);
  }

  // Returns an iterator to the first diagonal element
  [[nodiscard]] constexpr auto diag_begin() noexcept {
    return diag_iterator(m_v.begin(), 0, m_diagStep);
  }

  // Returns a const iterator to the first diagonal element
  [[nodiscard]] constexpr auto cdiag_begin() const noexcept { return diag_begin(); }

  // Returns an iterator to the first element afer the last diagonal element
  [[nodiscard]] constexpr auto diag_end() const noexcept {
    return const_diag_iterator(m_v.begin(), m_minSize * m_diagStep, m_diagStep);
  }

  // Returns an iterator to the first element afer the last diagonal element
  [[nodiscard]] constexpr auto diag_end() noexcept {
    return diag_iterator(m_v.begin(), m_minSize * m_diagStep, m_diagStep);
  }

  // Returns a const iterator to the first element afer the last diagonal element
  [[nodiscard]] constexpr auto cdiag_end() const noexcept { return diag_end(); }

  template <size_t L>
  [[nodiscard]] constexpr auto operator()(const std::array<size_t, L> &index) const noexcept
      -> decltype(auto) {
    return m_v[position(index)];
  }

  template <size_t L>
  [[nodiscard]] constexpr auto operator()(const std::array<size_t, L> &index) noexcept
      -> decltype(auto) {
    return m_v[position(index)];
  }

  template <typename... SizeT>
  [[nodiscard]] constexpr auto operator()(SizeT... i) const noexcept -> decltype(auto) {
    static_assert(std::conjunction_v<std::is_integral<SizeT>...>);
    return operator()(std::array<size_t, sizeof...(SizeT)>{static_cast<size_t>(i)...});
  }

  template <typename... SizeT>
  [[nodiscard]] constexpr auto operator()(SizeT... i) noexcept -> decltype(auto) {
    static_assert(std::conjunction_v<std::is_integral<SizeT>...>);
    return operator()(std::array<size_t, sizeof...(SizeT)>{static_cast<size_t>(i)...});
  }

  [[nodiscard]] constexpr auto operator-() const noexcept {
    auto result = Array{};
    cx::transform(cbegin(), cend(), result.begin(), std::negate<>{});
    return result;
  }

  constexpr auto operator+=(const T &v) noexcept -> decltype(auto) {
    cx::transform(cbegin(), cend(), begin(), [&v](auto x) { return x + v; });
    return *this;
  }

  constexpr auto operator-=(const T &v) noexcept -> decltype(auto) {
    cx::transform(cbegin(), cend(), begin(), [&v](auto x) { return x - v; });
    return *this;
  }

  constexpr auto operator/=(const T &v) noexcept -> decltype(auto) {
    cx::transform(cbegin(), cend(), begin(), [&v](auto x) { return x / v; });
    return *this;
  }

  constexpr auto operator*=(const T &v) noexcept -> decltype(auto) {
    cx::transform(cbegin(), cend(), begin(), [&v](auto x) { return x * v; });
    return *this;
  }

  template <typename OtherArray, typename = typename OtherArray::const_iterator>
  constexpr auto operator+=(const OtherArray &that) noexcept -> decltype(auto) {
    cx::transform(cbegin(), cend(), that.cbegin(), begin(), std::plus<>{});
    return *this;
  }

  template <typename OtherArray, typename = typename OtherArray::const_iterator>
  constexpr auto operator-=(const OtherArray &that) noexcept -> decltype(auto) {
    cx::transform(cbegin(), cend(), that.cbegin(), begin(), std::minus<>{});
    return *this;
  }

  // Return array with diagonal filled with 1
  [[nodiscard]] static constexpr auto eye() noexcept {
    auto result = Array{};
    cx::fill(result.diag_begin(), result.diag_end(), value_type{1});
    return result;
  }

  template <typename OtherArray>
  [[nodiscard]] static constexpr auto from(const OtherArray &that) noexcept {
    return Array{that};
  }
};
} // namespace stack

namespace heap {
template <size_t D, typename T> class Array {
private:
  using InternalArray = std::vector<T>;

  std::array<size_t, D> m_size;
  std::array<size_t, D + 1> m_step;
  InternalArray m_v;

public:
  using value_type = T;
  using reference = T &;
  using const_reference = const T &;
  using iterator = typename InternalArray::iterator;
  using const_iterator = typename InternalArray::const_iterator;
  using dim_iterator = TMIV::Common::Array::SteppedIterator<iterator>;
  using const_dim_iterator = TMIV::Common::Array::SteppedIterator<const_iterator>;
  using diag_iterator = dim_iterator;
  using const_diag_iterator = const_dim_iterator;
  using difference_type = ptrdiff_t;
  using container_type = Array<D, T>;
  using tuple_type = std::array<size_t, D>;
  template <typename U> using promoted_type = Array<D, std::common_type_t<T, U>>;

  // Default constructors
  Array() {
    m_size.fill(0);
    m_step.fill(0);
  }
  explicit Array(const tuple_type &sz) : Array() { this->resize(sz); }

  // Destructor
  ~Array() = default;

  // Copy constructors
  Array(const Array &that) = default;
  Array(const tuple_type &sz, T v) : Array() {
    this->resize(sz);
    std::fill(begin(), end(), v);
  }
  Array(const tuple_type &sz, std::initializer_list<T> v) : Array() {
    this->resize(sz);
    m_v = v;
  }
  template <typename OTHER, typename = typename OTHER::dim_iterator>
  explicit Array(const OTHER &that) : Array() {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->resize(sz);
    std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });
  }

  // Move constructor
  Array(Array &&that) noexcept {
    m_size = that.m_size;
    m_step = that.m_step;
    m_v = std::move(that.m_v);

    that.m_size.fill(0);
    that.m_step.fill(0);
  }

  // Copy assignment
  auto operator=(const Array &that) -> Array & = default;
  template <typename OTHER, typename = typename OTHER::dim_iterator>
  auto operator=(const OTHER &that) -> Array & {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->resize(sz);
    std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });

    return *this;
  }
  auto operator=(T v) -> Array & {
    std::fill(begin(), end(), v);
    return *this;
  }

  // Move assignment
  auto operator=(Array &&that) noexcept -> Array & {
    m_size = that.m_size;
    m_step = that.m_step;
    m_v = std::move(that.m_v);

    that.m_size.fill(0);
    that.m_step.fill(0);

    return *this;
  }

  // Equal operator
  auto operator==(const Array &that) const noexcept {
    return (std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) &&
            std::equal(begin(), end(), that.begin()));
  }

  // Different operator
  auto operator!=(const Array &that) const noexcept {
    return (!std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) ||
            !std::equal(begin(), end(), that.begin()));
  }

  // Swap operator
  void swap(Array &that) {
    std::swap(m_size, that.m_size);
    std::swap(m_step, that.m_step);
    m_v.swap(that.m_v);
  }

  // Resize operator
  void resize(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin())) {
      return;
    }

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_t l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1, [&l](size_t s) {
      l *= s;
      return l;
    });

    // Data
    m_v.resize(m_step.front());
  }

  // Reshape operator
  void reshape(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin())) {
      return;
    }

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_t l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1, [&l](size_t s) {
      l *= s;
      return l;
    });
  }

  // Returns the array dimension
  static constexpr auto dim() noexcept { return D; }

  // Returns the array size along the i-th dimension
  [[nodiscard]] auto size(size_t i) const noexcept { return m_size[i]; }

  // Returns the array sizes
  [[nodiscard]] auto sizes() const noexcept -> decltype(auto) { return m_size; }

  [[nodiscard]] auto size() const noexcept { return m_v.size(); }

  // Returns the gap between 2 consecutive elements on the ith dimension
  [[nodiscard]] auto step(size_t i) const noexcept { return m_step[i + 1]; }

  // Returns the array steps
  [[nodiscard]] auto steps() const noexcept -> decltype(auto) { return m_step; }

  [[nodiscard]] auto empty() const noexcept { return m_v.empty(); }
  [[nodiscard]] auto data() noexcept -> auto * { return m_v.data(); }
  [[nodiscard]] auto data() const noexcept -> const auto * { return m_v.data(); }

  [[nodiscard]] auto operator[](size_t k) noexcept -> decltype(auto) { return m_v[k]; }
  [[nodiscard]] auto operator[](size_t k) const noexcept -> decltype(auto) { return m_v[k]; }

  [[nodiscard]] auto begin() noexcept { return m_v.begin(); }
  [[nodiscard]] auto begin() const noexcept { return m_v.begin(); }
  [[nodiscard]] auto cbegin() const noexcept { return m_v.cbegin(); }
  [[nodiscard]] auto end() noexcept { return m_v.end(); }
  [[nodiscard]] auto end() const noexcept { return m_v.end(); }
  [[nodiscard]] auto cend() const noexcept { return m_v.cend(); }

  // Returns an iterator along the Kth dimension to the first element of the hyperplane defined by
  // next
  template <size_t K, typename... I> [[nodiscard]] auto dim_begin(I... next) const noexcept {
    return const_dim_iterator(begin(), offset<K>(0, next...), m_step[K + 1]);
  }
  template <size_t K, typename... I> auto dim_begin(I... next) noexcept {
    return dim_iterator(begin(), offset<K>(0, next...), m_step[K + 1]);
  }

  // Returns a const iterator along the Kth dimension to the first element of the hyperplane defined
  // by next
  template <size_t K, typename... I>
  [[nodiscard]] [[nodiscard]] auto cdim_begin(I... next) const noexcept {
    return const_dim_iterator(begin(), offset<K>(0, next...), m_step[K + 1]);
  }

  // Returns an iterator along the Kth dimension to the first element after the end of the
  // hyperplane defined by next
  template <size_t K, typename... I> [[nodiscard]] auto dim_end(I... next) const noexcept {
    return const_dim_iterator(begin(), offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  template <size_t K, typename... I> auto dim_end(I... next) noexcept {
    return dim_iterator(begin(), offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }

  // Returns a const iterator along the Kth dimension to the first element after the end of the
  // hyperplane defined by next
  template <size_t K, typename... I> [[nodiscard]] auto cdim_end(I... next) const noexcept {
    return const_dim_iterator(begin(), offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }

  // Returns an iterator to the first diagonal element
  [[nodiscard]] auto diag_begin() const noexcept {
    return const_diag_iterator(begin(), 0,
                               std::accumulate(m_step.begin() + 1, m_step.end(), size_t{}));
  }
  auto diag_begin() noexcept {
    return diag_iterator(begin(), 0, std::accumulate(m_step.begin() + 1, m_step.end(), size_t{}));
  }

  // Returns a const iterator to the first diagonal element
  [[nodiscard]] auto cdiag_begin() const noexcept {
    return const_diag_iterator(begin(), 0,
                               std::accumulate(m_step.begin() + 1, m_step.end(), size_t{}));
  }

  // Returns an iterator to the first element afer the last diagonal element
  [[nodiscard]] auto diag_end() const noexcept {
    size_t d = std::accumulate(m_step.begin() + 1, m_step.end(), size_t{});
    return const_diag_iterator(begin(), *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  auto diag_end() noexcept {
    size_t d = std::accumulate(m_step.begin() + 1, m_step.end(), size_t{});
    return diag_iterator(begin(), *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }

  // Returns a const iterator to the first element after the last diagonal element
  [[nodiscard]] auto cdiag_end() const noexcept { return diag_end(); }

  // Returns m(i, j, k, ..)
  template <typename... I>
  auto operator()(size_t first, I... next) const noexcept -> decltype(auto) {
    return m_v[pos(1, first, next...)];
  }
  template <typename... I> auto operator()(size_t first, I... next) noexcept -> decltype(auto) {
    return m_v[pos(1, first, next...)];
  }

  // Returns distance of m(i, j, k, ...) from m(0, 0, 0, ..)
  template <typename... I> auto distance(size_t first, I... next) const noexcept {
    return pos(1, first, next...);
  }

  // Unary - operator
  auto operator-() const {
    Array v(sizes());
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }

  // += scalar operator
  auto operator+=(T v) noexcept -> decltype(auto) {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
    return *this;
  }

  // -= scalar operator
  auto operator-=(T v) noexcept -> decltype(auto) {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
    return *this;
  }

  // /= scalar operator
  auto operator/=(T v) noexcept -> decltype(auto) {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
    return *this;
  }

  // *= scalar operator
  auto operator*=(T v) noexcept -> decltype(auto) {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
    return *this;
  }

  // += operator
  template <typename OTHER, typename = typename OTHER::const_iterator>
  auto operator+=(const OTHER &that) noexcept -> decltype(auto) {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }

  // -= operator
  template <typename OTHER, typename = typename OTHER::const_iterator>
  auto operator-=(const OTHER &that) noexcept -> decltype(auto) {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }

  // Return array with diagonal filled with 1
  static auto eye(const tuple_type &sz) {
    Array out(sz);

    std::fill(out.begin(), out.end(), T{0});
    std::fill(out.diag_begin(), out.diag_end(), T{1});

    return out;
  }

private:
  template <size_t K> [[nodiscard]] auto offset(size_t i, size_t first = 0) const noexcept {
    return (i == K) ? first : first * m_step[i + 1];
  }
  template <size_t K, typename... I>
  [[nodiscard]] auto offset(size_t i, size_t first, I... next) const noexcept {
    return (i == K) ? offset<K>(i + 1, first, next...)
                    : first * m_step[i + 1] + offset<K>(i + 1, next...);
  }
  [[nodiscard]] auto pos(size_t /*unused*/, size_t first) const noexcept { return first; }
  template <typename... I>
  [[nodiscard]] auto pos(size_t i, size_t first, I... next) const noexcept {
    return first * m_step[i] + pos(i + 1, next...);
  }
};
} // namespace heap
} // namespace TMIV::Common

// Send the array a to the stream os
template <typename A, typename = typename A::dim_iterator>
auto operator<<(std::ostream &os, const A &a) -> std::ostream & {
  typename A::size_t step = a.size(a.dim() - 1);
  typename A::const_iterator iter;
  typename A::const_iterator iter1 = a.begin();
  typename A::const_iterator iter2 = a.end();

  for (iter = iter1; iter != iter2; iter += step) {
    std::for_each(iter, iter + step, [&os](typename A::value_type v) { os << v << " "; });
    if ((iter + step) != iter2) {
      os << '\n';
    }
  }

  return os;
}

// Load the array a from the stream is
template <typename A, typename = typename A::dim_iterator>
auto operator>>(std::istream &is, A &a) -> std::istream & {
  for (auto &e : a) {
    is >> e;
  }
  return is;
}

// Return true if a1 and a2 have the same size
template <typename A1, typename A2> auto same_size(const A1 &a1, const A2 &a2) -> bool {
  if (a1.dim() != a2.dim()) {
    return false;
  }

  for (typename A1::size_t i = 0; i < a1.dim(); i++) {
    if (a1.size(i) != a2.size(i)) {
      return false;
    }
  }

  return true;
}

// array/scalar + operator
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void add(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v + u); });
}
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void add(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (u + v); });
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator+(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  add(m, u, out);
  return out;
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator+(U u, const A1 &m) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  add(u, m, out);
  return out;
}

// array/scalar - operator
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void sub(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v - u); });
}
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void sub(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (u - v); });
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator-(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  sub(m, u, out);
  return out;
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator-(U u, const A1 &m) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  sub(u, m, out);
  return out;
}

// array/scalar * operator
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void mult(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v * u); });
}
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void mult(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (u * v); });
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator*(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  mult(m, u, out);
  return out;
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator*(U u, const A1 &m) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  mult(u, m, out);
  return out;
}

// array/scalar / operator
template <typename A1, typename U, typename A2, typename = typename A1::dim_iterator,
          typename = typename TMIV::Common::NumericChecker<U>, typename = typename A2::dim_iterator>
void div(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v / u); });
}
template <typename A1, typename U, typename = typename A1::diag_iterator,
          typename = typename TMIV::Common::NumericChecker<U>>
auto operator/(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  div(m, u, out);
  return out;
}

// array/array + operator
template <typename A1, typename A2, typename A3, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator, typename = typename A3::dim_iterator>
void add(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 + v2; });
}
template <typename A1, typename A2, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator>
auto operator+(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  add(m1, m2, out);
  return out;
}

// array/array - operator
template <typename A1, typename A2, typename A3, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator, typename = typename A3::dim_iterator>
void sub(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 - v2; });
}
template <typename A1, typename A2, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator>
auto operator-(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  sub(m1, m2, out);
  return out;
}

// Element-by-element multiplication operator
template <typename A1, typename A2, typename A3, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator, typename = typename A3::dim_iterator>
void mult(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 * v2; });
}
template <typename A1, typename A2, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator>
auto mult(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  mult(m1, m2, out);
  return out;
}

// Element-by-element division operator
template <typename A1, typename A2, typename A3, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator, typename = typename A3::dim_iterator>
void div(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 / v2; });
}
template <typename A1, typename A2, typename = typename A1::dim_iterator,
          typename = typename A2::dim_iterator>
auto div(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  div(m1, m2, out);
  return out;
}

#endif
