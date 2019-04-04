/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifndef _TMIV_COMMON_ARRAY_H_
#define _TMIV_COMMON_ARRAY_H_

#include <algorithm>
#include <array>
#include <numeric>
#include <ostream>
#include <vector>

#include "Traits.h"

namespace TMIV::Common {
namespace Array {
using size_type = unsigned int;

template <typename T>
class const_iterator
    : public std::iterator<std::random_access_iterator_tag, T> {
protected:
  T *m_p;

public:
  const_iterator(const T *x = nullptr) : m_p((T *)x) {}
  const_iterator(const const_iterator &iter) = default;
  const_iterator(const_iterator &&iter) = default;
  ~const_iterator() = default;
  const_iterator &operator=(const const_iterator &rhs) = default;
  const_iterator &operator=(const_iterator &&rhs) = default;
  bool operator==(const const_iterator &rhs) const { return m_p == rhs.m_p; }
  bool operator!=(const const_iterator &rhs) const { return m_p != rhs.m_p; }
  const T &operator*() const { return *m_p; }
  const T *operator->() const { return m_p; }
  const_iterator &operator++() {
    ++m_p;
    return *this;
  }
  const_iterator operator++(int) {
    const_iterator tmp(*this);
    operator++();
    return tmp;
  }
  const_iterator &operator--() {
    --m_p;
    return *this;
  }
  const_iterator operator--(int) {
    const_iterator tmp(*this);
    operator--();
    return tmp;
  }
  const_iterator operator+(std::ptrdiff_t n) const {
    return const_iterator(m_p + n);
  }
  const_iterator &operator+=(std::ptrdiff_t n) {
    m_p += n;
    return *this;
  }
  std::ptrdiff_t operator-(const const_iterator &iter) const {
    return m_p - iter.m_p;
  }
  const_iterator operator-(std::ptrdiff_t n) const {
    return const_iterator(m_p - n);
  }
  const_iterator &operator-=(std::ptrdiff_t n) {
    m_p -= n;
    return *this;
  }
  const T &operator[](std::ptrdiff_t n) const { return m_p[n]; }
  bool operator<(const const_iterator &rhs) const { return m_p < rhs.m_p; }
  bool operator<=(const const_iterator &rhs) const { return m_p <= rhs.m_p; }
  bool operator>(const const_iterator &rhs) const { return m_p > rhs.m_p; }
  bool operator>=(const const_iterator &rhs) const { return m_p >= rhs.m_p; }
  void swap(const_iterator &a, const_iterator &b) { std::swap(a, b); }
};

template <typename T>
const_iterator<T> operator+(std::ptrdiff_t n, const const_iterator<T> &rhs) {
  return rhs + n;
}

template <typename T> class iterator : public const_iterator<T> {
public:
  iterator(T *x = nullptr) : const_iterator<T>(x) {}
  T &operator*() { return *(this->m_p); }
  T *operator->() { return (this->m_p); }
  iterator &operator++() {
    ++this->m_p;
    return *this;
  }
  iterator operator++(int) {
    iterator tmp(*this);
    operator++();
    return tmp;
  }
  iterator &operator--() {
    --this->m_p;
    return *this;
  }
  iterator operator--(int) {
    iterator tmp(*this);
    operator--();
    return tmp;
  }
  iterator operator+(std::ptrdiff_t n) const { return iterator(this->m_p + n); }
  iterator &operator+=(std::ptrdiff_t n) {
    this->m_p += n;
    return *this;
  }
  std::ptrdiff_t operator-(const iterator &iter) const {
    return this->m_p - iter.m_p;
  }
  iterator operator-(std::ptrdiff_t n) const { return iterator(this->m_p - n); }
  iterator &operator-=(std::ptrdiff_t n) {
    this->m_p -= n;
    return *this;
  }
  T &operator[](std::ptrdiff_t n) { return (this->m_p)[n]; }
};

template <typename T>
iterator<T> operator+(std::ptrdiff_t n, const iterator<T> &rhs) {
  return rhs + n;
}

template <typename T>
class const_dim_iterator
    : public std::iterator<std::random_access_iterator_tag, T> {
protected:
  T *m_p;
  std::ptrdiff_t m_step;

public:
  const_dim_iterator(const T *x = nullptr, std::ptrdiff_t s = 0)
      : m_p((T *)x), m_step(s) {}
  const_dim_iterator(const const_dim_iterator &iter) = default;
  const_dim_iterator(const_dim_iterator &&iter) = default;
  ~const_dim_iterator() = default;
  std::ptrdiff_t n() const { return m_step; }
  const_dim_iterator &operator=(const const_dim_iterator &rhs) = default;
  const_dim_iterator &operator=(const_dim_iterator &&rhs) = default;
  bool operator==(const const_dim_iterator &rhs) const {
    return m_p == rhs.m_p;
  }
  bool operator!=(const const_dim_iterator &rhs) const {
    return m_p != rhs.m_p;
  }
  const T &operator*() const { return *m_p; }
  const T *operator->() const { return m_p; }
  const_dim_iterator &operator++() {
    m_p += m_step;
    return *this;
  }
  const_dim_iterator operator++(int) {
    const_dim_iterator out(*this);
    operator++();
    return out;
  }
  const_dim_iterator &operator--() {
    m_p -= m_step;
    return *this;
  }
  const_dim_iterator operator--(int) {
    const_dim_iterator out(*this);
    operator--();
    return out;
  }
  const_dim_iterator operator+(std::ptrdiff_t a) const {
    return const_dim_iterator(m_p + a * m_step, this->m_step);
  }
  const_dim_iterator &operator+=(std::ptrdiff_t a) {
    m_p += (a * m_step);
    return *this;
  }
  std::ptrdiff_t operator-(const const_dim_iterator &iter) const {
    return (m_p - iter.m_p) / m_step;
  }
  const_dim_iterator operator-(std::ptrdiff_t a) const {
    return const_dim_iterator(m_p - a * m_step, this->m_step);
  }
  const_dim_iterator &operator-=(std::ptrdiff_t a) {
    m_p -= (a * m_step);
    return *this;
  }
  const T &operator[](std::ptrdiff_t a) const { return m_p[a * m_step]; }
  bool operator<(const const_dim_iterator &rhs) const { return m_p < rhs.m_p; }
  bool operator<=(const const_dim_iterator &rhs) const {
    return m_p <= rhs.m_p;
  }
  bool operator>(const const_dim_iterator &rhs) const { return m_p > rhs.m_p; }
  bool operator>=(const const_dim_iterator &rhs) const {
    return m_p >= rhs.m_p;
  }
  void swap(const_dim_iterator &a, const_dim_iterator &b) { std::swap(a, b); }
};

template <typename T>
const_dim_iterator<T> operator+(std::ptrdiff_t a,
                                const const_dim_iterator<T> &rhs) {
  return rhs + a;
}

template <typename T> class dim_iterator : public const_dim_iterator<T> {
public:
  dim_iterator(T *x = nullptr, std::ptrdiff_t s = 0)
      : const_dim_iterator<T>(x, s) {}
  T &operator*() { return *this->m_p; }
  T *operator->() { return this->m_p; }
  dim_iterator &operator++() {
    this->m_p += this->m_step;
    return *this;
  }
  dim_iterator operator++(int) {
    dim_iterator out(*this);
    operator++();
    return out;
  }
  dim_iterator &operator--() {
    this->m_p -= this->m_step;
    return *this;
  }
  dim_iterator operator--(int) {
    dim_iterator out(*this);
    operator--();
    return out;
  }
  dim_iterator operator+(std::ptrdiff_t a) const {
    return dim_iterator(this->m_p + (a * this->m_step), this->m_step);
  }
  dim_iterator &operator+=(std::ptrdiff_t a) {
    this->m_p += (a * this->m_step);
    return *this;
  }
  std::ptrdiff_t operator-(const dim_iterator &iter) const {
    return (this->m_p - iter.m_p) / this->m_step;
  }
  dim_iterator operator-(std::ptrdiff_t a) const {
    return dim_iterator(this->m_p - a * this->m_step, this->m_step);
  }
  dim_iterator &operator-=(std::ptrdiff_t a) {
    this->m_p -= (a * this->m_step);
    return *this;
  }
  T &operator[](std::ptrdiff_t a) { return this->m_p[a * this->m_step]; }
};

template <typename T>
dim_iterator<T> operator+(std::ptrdiff_t a, const dim_iterator<T> &rhs) {
  return rhs + a;
}

} // namespace Array

namespace stack {

using size_type = TMIV::Common::Array::size_type;

template <size_type D, typename T, size_type... I> struct _Array {};

template <typename T, size_type M> struct _Array<1, T, M> {
protected:
  std::array<T, M> m_v;

public:
  static constexpr size_type size(size_type) { return M; }
  static void sizes(size_type *iter) { *iter = M; }
  static constexpr size_type size() { return M; }
  static constexpr size_type min_size() { return M; }
  T *data() { return reinterpret_cast<T *>(m_v.data()); }
  const T *data() const { return reinterpret_cast<const T *>(m_v.data()); }
  template <size_type K>
  static constexpr size_type offset(size_type i, size_type first = 0,
                                    size_type second = 0) {
    return (i == K) ? second : first;
  }
  static constexpr size_type step(size_type i) { return i ? 1 : M; }
  static constexpr size_type diag_step() { return 1; }
  T get(size_type first) const { return m_v[first]; }
  T &get(size_type first) { return m_v[first]; }
};

template <size_type D, typename T, size_type M, size_type N, size_type... I>
struct _Array<D, T, M, N, I...> {
protected:
  std::array<_Array<D - 1, T, N, I...>, M> m_v;

public:
  static constexpr size_type size(size_type i) {
    return i ? _Array<D - 1, T, N, I...>::size(i - 1) : M;
  }
  static void sizes(size_type *iter) {
    *iter = M;
    _Array<D - 1, T, N, I...>::sizes(++iter);
  }
  static constexpr size_type size() {
    return M * _Array<D - 1, T, N, I...>::size();
  }
  static size_type min_size() {
    return (std::min)(M, _Array<D - 1, T, N, I...>::min_size());
  }
  T *data() { return reinterpret_cast<T *>(m_v.data()); }
  const T *data() const { return reinterpret_cast<const T *>(m_v.data()); }
  template <size_type K, typename... J>
  static constexpr size_type offset(size_type i, size_type first, J... next) {
    return (i == K)
               ? _Array<D - 1, T, N, I...>::template offset<K>(i + 1, first,
                                                               next...)
               : first * _Array<D - 1, T, N, I...>::size() +
                     _Array<D - 1, T, N, I...>::template offset<K>(i + 1,
                                                                   next...);
  }
  static constexpr size_type step(size_type i) {
    return i ? _Array<D - 1, T, N, I...>::step(i - 1)
             : M * _Array<D - 1, T, N, I...>::step(i);
  }
  static constexpr size_type diag_step() {
    return step(1) + _Array<D - 1, T, N, I...>::diag_step();
  }
  template <typename... J> T get(size_type first, J... next) const {
    return m_v[first].get(next...);
  }
  template <typename... J> T &get(size_type first, J... next) {
    return m_v[first].get(next...);
  }
};

template <typename T, size_type... I> class Array {
public:
  typedef T value_type;
  typedef T &reference;
  typedef const T &const_reference;
  typedef TMIV::Common::Array::iterator<T> iterator;
  typedef TMIV::Common::Array::const_iterator<T> const_iterator;
  typedef TMIV::Common::Array::dim_iterator<T> dim_iterator;
  typedef TMIV::Common::Array::const_dim_iterator<T> const_dim_iterator;
  typedef TMIV::Common::Array::dim_iterator<T> diag_iterator;
  typedef TMIV::Common::Array::const_dim_iterator<T> const_diag_iterator;
  typedef std::ptrdiff_t difference_type;
  typedef stack::size_type size_type;
  typedef Array<T, I...> container_type;
  typedef std::array<stack::size_type, sizeof...(I)> tuple_type;

protected:
  class Helper {
  protected:
    tuple_type m_sizes;

  public:
    Helper() { _Array<sizeof...(I), T, I...>::sizes(m_sizes.data()); };
    const tuple_type &sizes() const { return m_sizes; }
  };

protected:
  static Helper m_helper;
  _Array<sizeof...(I), T, I...> m_v;

public:
  //! \brief Default constructor
  Array() = default;
  //! \brief Destructor.
  ~Array() = default;
  //! \brief Copy constructors.
  Array(const container_type &that) = default;
  Array(T t) { std::fill(begin(), end(), t); }
  Array(std::initializer_list<T> v) {
    std::copy(v.begin(), v.begin() + size(), begin());
  }
  template <typename OTHER, class = typename OTHER::dim_iterator>
  Array(const OTHER &that) : Array() {
    if ((dim() == that.dim()) &&
        std::equal(that.sizes().begin(), that.sizes().end(), sizes().begin()))
      std::copy(that.begin(), that.end(), begin());
  }
  //! \brief Move constructor.
  Array(container_type &&that) = default;
  //! \brief Copy assignment.
  container_type &operator=(const container_type &that) = default;
  container_type &operator=(T t) {
    std::fill(begin(), end(), t);
    return *this;
  }
  container_type &operator=(std::initializer_list<T> v) {
    std::copy(v.begin(), v.begin() + size(), begin());
    return *this;
  }
  template <typename OTHER, class = typename OTHER::dim_iterator>
  container_type &operator=(const OTHER &that) {
    if ((dim() == that.dim()) &&
        std::equal(that.sizes().begin(), that.sizes().end(), sizes().begin()))
      std::copy(that.begin(), that.end(), begin());

    return *this;
  }
  //! \brief Move assignment.
  container_type &operator=(container_type &&that) = default;
  //! \brief Equal operator.
  bool operator==(const container_type &that) const {
    return std::equal(begin(), end(), that.begin());
  }
  //! \brief Different operator.
  bool operator!=(const container_type &that) const {
    return !std::equal(begin(), end(), that.begin());
  }
  //! \brief Swap operator.
  void swap(container_type &that) { std::swap(m_v, that.m_v); }
  //! \brief Resize operator.
  void resize(const tuple_type &) {}
  //! \brief Reshape operator.
  void reshape(const tuple_type &) {}
  //! \brief Returns the array dimension.
  static constexpr size_type dim() { return sizeof...(I); }
  //! \brief Returns the array size along the i-th dimension.
  static constexpr size_type size(size_type i) {
    return _Array<sizeof...(I), T, I...>::size(i);
  }
  //! \brief Returns the array sizes.
  static const tuple_type &sizes() {
    return m_helper.sizes();
  } // tuple_type out; _Array<sizeof...(I), T, I...>::sizes(out.data()); return
    // out; }
  //! \brief Returns the array total length
  static constexpr size_type size() {
    return _Array<sizeof...(I), T, I...>::size();
  }
  //! \brief Returns the gap between 2 consecutive elements on the ith
  //! dimension.
  static constexpr size_type step(size_type i) {
    return _Array<sizeof...(I), T, I...>::step(i + 1);
  }
  //! \brief Returns true if the array is empty.
  static constexpr bool empty() { return (size() == 0); }
  //! \brief Data access, returns a pointer to the first element of the array.
  T *data() { return m_v.data(); }
  const T *data() const { return m_v.data(); }
  //! \brief [] operator, returns the kth element of the array viewed as a one
  //! dimensional array.
  T operator[](size_type k) const { return data()[k]; }
  T &operator[](size_type k) { return data()[k]; }
  //! \brief Return the property of the array
  int getProperty() const { return -1; }
  //! \brief Returns an iterator to the first element of the array.
  iterator begin() { return iterator(data()); }
  const_iterator begin() const { return const_iterator((T *)data()); }
  //! \brief Returns a const iterator to the first element of the array.
  const_iterator cbegin() const { return const_iterator((T *)data()); }
  //! \brief Returns an iterator to the first element after the end of the
  //! array.
  iterator end() { return iterator(data() + size()); }
  const_iterator end() const { return const_iterator((T *)data() + size()); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! array.
  const_iterator cend() const { return const_iterator((T *)data() + size()); }
  //! \brief Returns an iterator along the Kth dimension to the first element of
  //! the hyperplane defined by next.
  template <size_type K, typename... J>
  const_dim_iterator dim_begin(J... next) const {
    return const_dim_iterator(
        (T *)data() +
            _Array<sizeof...(I), T, I...>::template offset<K>(0, next...),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  template <size_type K, typename... J> dim_iterator dim_begin(J... next) {
    return dim_iterator(
        data() + _Array<sizeof...(I), T, I...>::template offset<K>(0, next...),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element of the hyperplane defined by next.
  template <size_type K, typename... J>
  const_dim_iterator cdim_begin(J... next) const {
    return const_dim_iterator(
        (T *)data() +
            _Array<sizeof...(I), T, I...>::template offset<K>(0, next...),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns an iterator along the Kth dimension to the first element
  //! after the end of the hyperplane defined by next.
  template <size_type K, typename... J>
  const_dim_iterator dim_end(J... next) const {
    return const_dim_iterator(
        (T *)data() +
            _Array<sizeof...(I), T, I...>::template offset<K>(0, next...) +
            _Array<sizeof...(I), T, I...>::step(K),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  template <size_type K, typename... J> dim_iterator dim_end(J... next) {
    return dim_iterator(
        data() + _Array<sizeof...(I), T, I...>::template offset<K>(0, next...) +
            _Array<sizeof...(I), T, I...>::step(K),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element after the end of the hyperplane defined by next.
  template <size_type K, typename... J>
  const_dim_iterator cdim_end(J... next) const {
    return const_dim_iterator(
        (T *)data() +
            _Array<sizeof...(I), T, I...>::template offset<K>(next...) +
            _Array<sizeof...(I), T, I...>::step(K),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns an iterator to the first diagonal element.
  const_diag_iterator diag_begin() const {
    return const_diag_iterator((T *)data(),
                               _Array<sizeof...(I), T, I...>::diag_step());
  }
  diag_iterator diag_begin() {
    return diag_iterator(data(), _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns a const iterator to the first diagonal element.
  const_diag_iterator cdiag_begin() const {
    return const_diag_iterator((T *)data(),
                               _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns an iterator to the first element afer the last diagonal
  //! element.
  const_diag_iterator diag_end() const {
    return const_diag_iterator(
        (T *)data() + _Array<sizeof...(I), T, I...>::min_size() *
                          _Array<sizeof...(I), T, I...>::diag_step(),
        _Array<sizeof...(I), T, I...>::diag_step());
  }
  diag_iterator diag_end() {
    return diag_iterator(data() +
                             _Array<sizeof...(I), T, I...>::min_size() *
                                 _Array<sizeof...(I), T, I...>::diag_step(),
                         _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns a const iterator to the first element afer the last
  //! diagonal element.
  const_diag_iterator cdiag_end() const {
    return const_diag_iterator(
        (T *)data() + _Array<sizeof...(I), T, I...>::min_size() *
                          _Array<sizeof...(I), T, I...>::diag_step(),
        _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns m(i, j, k, ...)
  template <typename... J> T operator()(J... idx) const {
    return m_v.get(idx...);
  }
  template <typename... J> T &operator()(J... idx) { return m_v.get(idx...); }
  //! \brief Unary - operator.
  container_type operator-() const {
    container_type v;
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }
  //! \brief += scalar operator.
  container_type &operator+=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
    return *this;
  }
  //! \brief -= scalar operator.
  container_type &operator-=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
    return *this;
  }
  //! \brief /= scalar operator.
  container_type &operator/=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
    return *this;
  }
  //! \brief *= scalar operator.
  container_type &operator*=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
    return *this;
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  container_type &operator+=(const OTHER &that) {
    std::transform(
        begin(), end(), that.begin(), begin(),
        [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  container_type &operator-=(const OTHER &that) {
    std::transform(
        begin(), end(), that.begin(), begin(),
        [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }
  //! \brief Return array filled with 0
  static container_type zeros() {
    container_type out;
    std::fill(out.begin(), out.end(), 0);
    return out;
  }
  //! \brief Return array with diagonal filled with 1
  static container_type eye() {
    container_type out;

    std::fill(out.begin(), out.end(), 0);
    std::fill(out.diag_begin(), out.diag_end(), 1);

    return out;
  }
  template <typename OTHER> static container_type from(const OTHER &other) {
    container_type out;
    std::copy(other.begin(), other.begin() + out.size(), out.begin());
    return out;
  }
};

template <typename T, size_type... I>
typename Array<T, I...>::Helper Array<T, I...>::m_helper;

} // namespace stack

namespace heap {

using size_type = TMIV::Common::Array::size_type;

template <size_type D, typename T> class Array {
public:
  typedef T value_type;
  typedef T &reference;
  typedef const T &const_reference;
  typedef TMIV::Common::Array::iterator<T> iterator;
  typedef TMIV::Common::Array::const_iterator<T> const_iterator;
  typedef TMIV::Common::Array::dim_iterator<T> dim_iterator;
  typedef TMIV::Common::Array::const_dim_iterator<T> const_dim_iterator;
  typedef TMIV::Common::Array::dim_iterator<T> diag_iterator;
  typedef TMIV::Common::Array::const_dim_iterator<T> const_diag_iterator;
  typedef std::ptrdiff_t difference_type;
  typedef heap::size_type size_type;
  typedef Array<D, T> container_type;
  typedef std::array<heap::size_type, D> tuple_type;

protected:
  std::array<size_type, D> m_size;
  std::array<size_type, D + 1> m_step;
  std::vector<T> m_v;
  int m_property = -1;

public:
  //! \brief Default constructors.
  Array() {
    m_size.fill(0);
    m_step.fill(0);
  }
  Array(const tuple_type &sz) : Array() { this->resize(sz); }
  //! \brief Destructor.
  ~Array() = default;
  //! \brief Copy constructors.
  Array(const container_type &that) = default;
  Array(const tuple_type &sz, T v) : Array() {
    this->resize(sz);
    std::fill(begin(), end(), v);
  }
  Array(const tuple_type &sz, std::initializer_list<T> v) : Array() {
    this->resize(sz);
    m_v = v;
  }
  template <typename OTHER, class = typename OTHER::dim_iterator>
  Array(const OTHER &that) : Array() {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->resize(sz);
    std::copy(that.begin(), that.end(), begin());

    m_property = that.getProperty();
  }
  //! \brief Move constructor.
  Array(container_type &&that) {
    m_size = that.m_size;
    m_step = that.m_step;
    m_v = std::move(that.m_v);
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_property = -1;
  }
  //! \brief Copy assignment.
  container_type &operator=(const container_type &that) = default;
  template <typename OTHER, class = typename OTHER::dim_iterator>
  container_type &operator=(const OTHER &that) {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->resize(sz);
    std::copy(that.begin(), that.end(), begin());

    m_property = that.getProperty();

    return *this;
  }
  container_type &operator=(T v) {
    std::fill(begin(), end(), v);
    return *this;
  }
  //! \brief Move assignment.
  container_type &operator=(container_type &&that) {
    m_size = that.m_size;
    m_step = that.m_step;
    m_v = std::move(that.m_v);
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_property = -1;

    return *this;
  }
  //! \brief Equal operator.
  bool operator==(const container_type &that) const {
    return (std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) &&
            std::equal(begin(), end(), that.begin()));
  }
  //! \brief Different operator.
  bool operator!=(const container_type &that) const {
    return (!std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) ||
            !std::equal(begin(), end(), that.begin()));
  }
  //! \brief Swap operator
  void swap(container_type &that) {
    std::swap(m_size, that.m_size);
    std::swap(m_step, that.m_step);
    m_v.swap(that.m_v);
    std::swap(m_property, that.m_property);
  }
  //! \brief Resize operator.
  void resize(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin()))
      return;

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_type l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1,
                   [&l](size_type s) {
                     l *= s;
                     return l;
                   });

    // Data
    m_v.resize(m_step.front());
  }
  //! \brief Reshape operator.
  void reshape(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin()))
      return;

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_type l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1,
                   [&l](size_type s) {
                     l *= s;
                     return l;
                   });
  }
  //! \brief Returns the array dimension.
  static constexpr size_type dim() { return D; }
  //! \brief Returns the array size along the i-th dimension.
  size_type size(size_type i) const { return m_size[i]; }
  //! \brief Returns the array sizes.
  const tuple_type &sizes() const { return m_size; }
  //! \brief Returns the array total length.
  size_type size() const { return m_step.front(); }
  //! \brief Returns the gap between 2 consecutive elements on the ith
  //! dimension.
  size_type step(size_type i) const { return m_step[i + 1]; }
  //! \brief Returns the array steps
  const std::array<size_type, D + 1> &steps() const { return m_step; }
  //! \brief Returns true if the array is empty.
  bool empty() const { return (size() == 0); }
  //! \brief Data access, returns a pointer to the first element of the array.
  T *data() { return m_v.data(); }
  const T *data() const { return m_v.data(); }
  //! \brief [] operator, returns the kth element of the array viewed as a one
  //! dimensional array.
  T operator[](size_type k) const { return m_v[k]; }
  T &operator[](size_type k) { return m_v[k]; }
  //! \brief Return the property of the array
  int getProperty() const { return m_property; }
  // \brief Set the property of the array
  void setProperty(int v) { m_property = v; }
  //! \brief Returns an iterator to the first element of the array.
  iterator begin() { return iterator(data()); }
  const_iterator begin() const { return const_iterator(data()); }
  //! \brief Returns a const iterator to the first element of the array.
  const_iterator cbegin() const { return const_iterator(data()); }
  //! \brief Returns an iterator to the first element after the end of the
  //! array.
  iterator end() { return iterator(data() + size()); }
  const_iterator end() const { return const_iterator(data() + size()); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! array.
  const_iterator cend() const { return const_iterator(data() + size()); }
  //! \brief Returns an iterator along the Kth dimension to the first element of
  //! the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator dim_begin(I... next) const {
    return const_dim_iterator(data() + offset<K>(0, next...), m_step[K + 1]);
  }
  template <size_type K, typename... I> dim_iterator dim_begin(I... next) {
    return dim_iterator(data() + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element of the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator cdim_begin(I... next) const {
    return const_dim_iterator(data() + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns an iterator along the Kth dimension to the first element
  //! after the end of the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator dim_end(I... next) const {
    return const_dim_iterator(data() + offset<K>(0, next...) + m_step[K],
                              m_step[K + 1]);
  }
  template <size_type K, typename... I> dim_iterator dim_end(I... next) {
    return dim_iterator(data() + offset<K>(0, next...) + m_step[K],
                        m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element after the end of the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator cdim_end(I... next) const {
    return const_dim_iterator(data() + offset<K>(0, next...) + m_step[K],
                              m_step[K + 1]);
  }
  //! \brief Returns an iterator to the first diagonal element.
  const_diag_iterator diag_begin() const {
    return const_diag_iterator(
        data(), std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  diag_iterator diag_begin() {
    return diag_iterator(data(),
                         std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  //! \brief Returns a const iterator to the first diagonal element.
  const_diag_iterator cdiag_begin() const {
    return const_diag_iterator(
        data(), std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  //! \brief Returns an iterator to the first element afer the last diagonal
  //! element.
  const_diag_iterator diag_end() const {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return const_diag_iterator(
        data() + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  diag_iterator diag_end() {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return diag_iterator(
        data() + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns a const iterator to the first element afer the last
  //! diagonal element.
  const_diag_iterator cdiag_end() const {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return const_diag_iterator(
        data() + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns m(i, j, k, ...)
  template <typename... I> T operator()(size_type first, I... next) const {
    return m_v[pos(1, first, next...)];
  }
  template <typename... I> T &operator()(size_type first, I... next) {
    return m_v[pos(1, first, next...)];
  }
  //! \brief Returns distance of m(i, j, k, ...) from m(0, 0, 0, ...)
  template <typename... I>
  size_type distance(size_type first, I... next) const {
    return pos(1, first, next...);
  }
  //! \brief Unary - operator.
  container_type operator-() const {
    container_type v(sizes());
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }
  //! \brief += scalar operator.
  void operator+=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
  }
  //! \brief -= scalar operator.
  void operator-=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
  }
  //! \brief /= scalar operator.
  void operator/=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
  }
  //! \brief *= scalar operator.
  void operator*=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  container_type &operator+=(const OTHER &that) {
    std::transform(
        begin(), end(), that.begin(), begin(),
        [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }
  //! \brief -= operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  container_type &operator-=(const OTHER &that) {
    std::transform(
        begin(), end(), that.begin(), begin(),
        [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }
  //! \brief Return array filled with 0
  static container_type zeros(const tuple_type &sz) {
    container_type out(sz);
    std::fill(out.begin(), out.end(), 0);
    return out;
  }
  //! \brief Return array with diagonal filled with 1
  static container_type eye(const tuple_type &sz) {
    container_type out(sz);

    std::fill(out.begin(), out.end(), 0);
    std::fill(out.diag_begin(), out.diag_end(), 1);

    return out;
  }

protected:
  template <size_type K>
  size_type offset(size_type i, size_type first = 0) const {
    return (i == K) ? first : first * m_step[i + 1];
  }
  template <size_type K, typename... I>
  size_type offset(size_type i, size_type first, I... next) const {
    return (i == K) ? offset<K>(i + 1, first, next...)
                    : first * m_step[i + 1] + offset<K>(i + 1, next...);
  }
  size_type pos(size_type, size_type first) const { return first; }
  template <typename... I>
  size_type pos(size_type i, size_type first, I... next) const {
    return first * m_step[i] + pos(i + 1, next...);
  }
};

} // namespace heap

namespace shallow {

using size_type = TMIV::Common::Array::size_type;

template <size_type D, typename T> class Array {
public:
  typedef T value_type;
  typedef T &reference;
  typedef const T &const_reference;
  typedef TMIV::Common::Array::iterator<T> iterator;
  typedef TMIV::Common::Array::const_iterator<T> const_iterator;
  typedef TMIV::Common::Array::dim_iterator<T> dim_iterator;
  typedef TMIV::Common::Array::const_dim_iterator<T> const_dim_iterator;
  typedef TMIV::Common::Array::dim_iterator<T> diag_iterator;
  typedef TMIV::Common::Array::const_dim_iterator<T> const_diag_iterator;
  typedef std::ptrdiff_t difference_type;
  typedef shallow::size_type size_type;
  typedef Array<D, T> container_type;
  typedef std::array<shallow::size_type, D> tuple_type;

protected:
  std::array<size_type, D> m_size;
  std::array<size_type, D + 1> m_step;
  T *m_data = nullptr;
  int m_property = -1;

public:
  //! \brief Default constructors.
  Array() {
    m_size.fill(0);
    m_step.fill(0);
  }
  //! \brief Destructor.
  ~Array() = default;
  //! \brief Copy constructors.
  Array(const tuple_type &sz, T *src) : Array() {
    this->reshape(sz);
    m_data = src;
  }
  Array(const container_type &that) = default;
  template <typename OTHER, class = typename OTHER::dim_iterator>
  Array(const OTHER &that,
        SameTypeChecker<T, typename OTHER::value_type> * = nullptr)
      : Array() {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->reshape(sz);
    m_data = (T *)that.data();

    m_property = that.getProperty();
  }
  //! \brief Move constructor.
  Array(container_type &&that) {
    m_size = that.m_size;
    m_step = that.m_step;
    m_data = that.m_data;
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_data = nullptr;
    that.m_property = -1;
  }
  //! \brief Copy assignment.
  container_type &operator=(const container_type &that) = default;
  template <typename OTHER, class = typename OTHER::dim_iterator>
  container_type &operator=(const OTHER &that) {
    if (size() == that.size()) {
      tuple_type sz;

      std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
      std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

      this->reshape(sz);
      std::copy(that.begin(), that.end(), begin());

      m_property = that.getProperty();
    }

    return *this;
  }
  container_type &operator=(T v) {
    std::fill(begin(), end(), v);
    return *this;
  }
  //! \brief Move assignment.
  container_type &operator=(container_type &&that) {
    m_size = that.m_size;
    m_step = that.m_step;
    m_data = that.m_data;
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_data = nullptr;
    that.m_property = -1;

    return *this;
  }
  //! \brief Equal operator.
  bool operator==(const container_type &that) const {
    return (std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) &&
            std::equal(begin(), end(), that.begin()));
  }
  //! \brief Different operator.
  bool operator!=(const container_type &that) const {
    return (!std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) ||
            !std::equal(begin(), end(), that.begin()));
  }
  //! \brief Swap operator
  void swap(container_type &that) {
    std::swap(m_size, that.m_size);
    std::swap(m_step, that.m_step);
    std::swap(m_data, that.m_data);
    std::swap(m_property, that.m_property);
  }
  //! \brief Resize operator.
  void resize(const tuple_type &) {}
  //! \brief Reshape operator.
  void reshape(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin()))
      return;

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_type l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1,
                   [&l](size_type s) {
                     l *= s;
                     return l;
                   });
  }
  //! \brief Returns the array dimension.
  static constexpr size_type dim() { return D; }
  //! \brief Returns the array size along the i-th dimension.
  size_type size(size_type i) const { return m_size[i]; }
  //! \brief Returns the array sizes.
  const tuple_type &sizes() const { return m_size; }
  //! \brief Returns the array total length.
  size_type size() const { return m_step.front(); }
  //! \brief Returns the gap between 2 consecutive elements on the ith
  //! dimension.
  size_type step(size_type i) const { return m_step[i + 1]; }
  //! \brief Returns true if the array is empty.
  bool empty() const { return (size() == 0); }
  //! \brief Data access, returns a pointer to the first element of the array.
  T *data() { return m_data; }
  const T *data() const { return m_data; }
  //! \brief [] operator, returns the kth element of the array viewed as a one
  //! dimensional array.
  T operator[](int k) const { return m_data[k]; }
  T &operator[](int k) { return m_data[k]; }
  //! \brief Return the property of the array
  int getProperty() const { return m_property; }
  // \brief Set the property of the array
  void setProperty(int v) { m_property = v; }
  //! \brief Returns an iterator to the first element of the array.
  iterator begin() { return iterator(m_data); }
  const_iterator begin() const { return const_iterator(m_data); }
  //! \brief Returns a const iterator to the first element of the array.
  const_iterator cbegin() const { return const_iterator(m_data); }
  //! \brief Returns an iterator to the first element after the end of the
  //! array.
  iterator end() { return iterator(m_data + size()); }
  const_iterator end() const { return const_iterator(m_data + size()); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! array.
  const_iterator cend() const { return const_iterator(m_data + size()); }
  //! \brief Returns an iterator along the Kth dimension to the first element of
  //! the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator dim_begin(I... next) const {
    return const_dim_iterator(m_data + offset<K>(0, next...), m_step[K + 1]);
  }
  template <size_type K, typename... I> dim_iterator dim_begin(I... next) {
    return dim_iterator(m_data + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element of the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator cdim_begin(I... next) const {
    return const_dim_iterator(m_data + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns an iterator along the Kth dimension to the first element
  //! after the end of the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator dim_end(I... next) const {
    return const_dim_iterator(m_data + offset<K>(0, next...) + m_step[K],
                              m_step[K + 1]);
  }
  template <size_type K, typename... I> dim_iterator dim_end(I... next) {
    return dim_iterator(m_data + offset<K>(0, next...) + m_step[K],
                        m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element after the end of the hyperplane defined by next.
  template <size_type K, typename... I>
  const_dim_iterator cdim_end(I... next) const {
    return const_dim_iterator(m_data + offset<K>(0, next...) + m_step[K],
                              m_step[K + 1]);
  }
  //! \brief Returns an iterator to the first diagonal element.
  const_diag_iterator diag_begin() const {
    return const_diag_iterator(
        m_data, std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  diag_iterator diag_begin() {
    return diag_iterator(m_data,
                         std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  //! \brief Returns a const iterator to the first diagonal element.
  const_diag_iterator cdiag_begin() const {
    return const_diag_iterator(
        m_data, std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  //! \brief Returns an iterator to the first element afer the last diagonal
  //! element.
  const_diag_iterator diag_end() const {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return const_diag_iterator(
        m_data + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  diag_iterator diag_end() {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return diag_iterator(
        m_data + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns a const iterator to the first element afer the last
  //! diagonal element.
  const_diag_iterator cdiag_end() const {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return const_diag_iterator(
        m_data + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns m(i, j, k, ...)
  template <typename... I> T operator()(size_type first, I... next) const {
    return m_data[pos(1, first, next...)];
  }
  template <typename... I> T &operator()(size_type first, I... next) {
    return m_data[pos(1, first, next...)];
  }
  //! \brief Returns distance of m(i, j, k, ...) from m(0, 0, 0, ...)
  template <typename... I>
  size_type distance(size_type first, I... next) const {
    return pos(1, first, next...);
  }
  //! \brief Unary - operator.
  container_type operator-() const {
    container_type v(sizes());
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }
  //! \brief += scalar operator.
  void operator+=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
  }
  //! \brief -= scalar operator.
  void operator-=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
  }
  //! \brief /= scalar operator.
  void operator/=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
  }
  //! \brief *= scalar operator.
  void operator*=(T v) {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  container_type &operator+=(const OTHER &that) {
    std::transform(
        begin(), end(), that.begin(), begin(),
        [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }
  //! \brief -= operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  container_type &operator-=(const OTHER &that) {
    std::transform(
        begin(), end(), that.begin(), begin(),
        [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }

protected:
  template <size_type K>
  size_type offset(size_type i, size_type first = 0) const {
    return (i == K) ? first : first * m_step[i + 1];
  }
  template <size_type K, typename... I>
  size_type offset(size_type i, size_type first, I... next) const {
    return (i == K) ? offset<K>(i + 1, first, next...)
                    : first * m_step[i + 1] + offset<K>(i + 1, next...);
  }
  size_type pos(size_type, size_type first) const { return first; }
  template <typename... I>
  size_type pos(size_type i, size_type first, I... next) const {
    return first * m_step[i] + pos(i + 1, next...);
  }
};

} // namespace shallow

} // namespace TMIV::Common

//! \brief Send the array a to the stream os.
template <typename A, class = typename A::dim_iterator>
std::ostream &operator<<(std::ostream &os, const A &a) {
  typename A::size_type step = a.size(a.dim() - 1);
  typename A::const_iterator iter, iter1 = a.begin(), iter2 = a.end();

  for (iter = iter1; iter != iter2; iter += step) {
    std::for_each(iter, iter + step,
                  [&os](typename A::value_type v) { os << v << " "; });
    if ((iter + step) != iter2)
      os << "\n";
  }

  return os;
}

//! \brief Load the array a from the stream is.
template <typename A, class = typename A::dim_iterator>
std::istream &operator>>(std::istream &is, A &a) {
  for (auto &e : a)
    is >> e;
  return is;
}

//! \brief Return true if a1 and a2 have the same size.
template <typename A1, typename A2> bool same_size(const A1 &a1, const A2 &a2) {
  if (a1.dim() != a2.dim())
    return false;

  for (typename A1::size_type i = 0; i < a1.dim(); i++) {
    if (a1.size(i) != a2.size(i))
      return false;
  }

  return true;
}

//! \brief array/scalar + operator.
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void add(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (v + u);
      });
}
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void add(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (u + v);
      });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator+(const A1 &m, U u) {
  A1 out;
  add(m, u, out);
  return out;
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator+(U u, const A1 &m) {
  A1 out;
  add(u, m, out);
  return out;
}

//! \brief array/scalar - operator.
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void sub(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (v - u);
      });
}
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void sub(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (u - v);
      });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator-(const A1 &m, U u) {
  A1 out;
  sub(m, u, out);
  return out;
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator-(U u, const A1 &m) {
  A1 out;
  sub(u, m, out);
  return out;
}

//! \brief array/scalar * operator.
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void mult(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (v * u);
      });
}
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void mult(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (u * v);
      });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator*(const A1 &m, U u) {
  A1 out;
  mult(m, u, out);
  return out;
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator*(U u, const A1 &m) {
  A1 out;
  mult(u, m, out);
  return out;
}

//! \brief array/scalar / operator.
template <typename A1, typename U, typename A2,
          class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>,
          class = typename A2::dim_iterator>
void div(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(
      m.begin(), m.end(),
      out.begin(), [u](typename A1::value_type v) -> typename A2::value_type {
        return (v / u);
      });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
A1 operator/(const A1 &m, U u) {
  A1 out;
  div(m, u, out);
  return out;
}

//! \brief array/array + operator.
template <typename A1, typename A2, typename A3,
          class = typename A1::dim_iterator, class = typename A2::dim_iterator,
          class = typename A3::dim_iterator>
void add(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 + v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
A1 operator+(const A1 &m1, const A2 &m2) {
  A1 out;
  add(m1, m2, out);
  return out;
}

//! \brief array/array - operator.
template <typename A1, typename A2, typename A3,
          class = typename A1::dim_iterator, class = typename A2::dim_iterator,
          class = typename A3::dim_iterator>
void sub(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 - v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
A1 operator-(const A1 &m1, const A2 &m2) {
  A1 out;
  sub(m1, m2, out);
  return out;
}

//! \brief Element-by-element multiplication operator.
template <typename A1, typename A2, typename A3,
          class = typename A1::dim_iterator, class = typename A2::dim_iterator,
          class = typename A3::dim_iterator>
void mult(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 * v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
A1 mult(const A1 &m1, const A2 &m2) {
  A1 out;
  mult(m1, m2, out);
  return out;
}

//! \brief Element-by-element division operator.
template <typename A1, typename A2, typename A3,
          class = typename A1::dim_iterator, class = typename A2::dim_iterator,
          class = typename A3::dim_iterator>
void div(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 / v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
A1 div(const A1 &m1, const A2 &m2) {
  A1 out;
  div(m1, m2, out);
  return out;
}

#endif
