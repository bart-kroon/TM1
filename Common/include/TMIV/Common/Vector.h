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

#ifndef _TMIV_COMMON_VECTOR_H_
#define _TMIV_COMMON_VECTOR_H_

#include "Array.h"
#include "Math.h"

#include <ostream>

namespace TMIV::Common {
template <typename A> class VectorInterface : public A {
public:
  using size_type = typename A::size_type;
  using const_row_iterator = typename A::const_dim_iterator;
  using row_iterator = typename A::dim_iterator;
  using const_column_iterator = typename A::const_dim_iterator;
  using column_iterator = typename A::dim_iterator;
  template <typename U>
  using promoted_type = VectorInterface<typename A::template promoted_type<U>>;

public:
  using A::A;
  VectorInterface() : A() {}
  explicit VectorInterface(const A &a) : A(a) {}
  explicit VectorInterface(A &&a) : A(std::move(a)) {}
  using A::operator=;
  VectorInterface &operator=(const A &a) {
    A::operator=(a);
    return *this;
  }
  VectorInterface &operator=(A &&a) {
    A::operator=(std::move(a));
    return *this;
  }
  //! \brief Returns the number of rows of the matrix.
  constexpr size_type m() const { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  constexpr size_type n() const { return 1; }
  //! \brief Overloaded resize operator.
  using A::resize;
  void resize(size_type a, size_type /*unused*/ = 1) { A::resize({a}); }
  //! \brief Returns an iterator to the first element of the ith row.
  const_row_iterator row_begin(size_type i) const { return const_row_iterator(A::data() + i); }
  row_iterator row_begin(size_type i) { return row_iterator(A::data() + i); }
  //! \brief Returns a const iterator to the first element of the ith row.
  const_row_iterator crow_begin(size_type i) const { return const_row_iterator(A::data() + i); }
  //! \brief Returns an iterator to the first element after the end of the ith
  //! row.
  const_row_iterator row_end(size_type i) const { return const_row_iterator(A::data() + (i + 1)); }
  row_iterator row_end(size_type i) { return row_iterator(A::data() + (i + 1)); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! ith row.
  const_row_iterator crow_end(size_type i) const { return const_row_iterator(A::data() + (i + 1)); }
  //! \brief Returns an iterator to the first element of the jth column.
  const_column_iterator col_begin(size_type /*unused*/ = 0) const { return A::begin(); }
  column_iterator col_begin(size_type /*unused*/ = 0) { return A::begin(); }
  //! \brief Returns a const iterator to the first element of the jth column.
  const_column_iterator ccol_begin(size_type /*unused*/ = 0) const { return A::cbegin(); }
  //! \brief Returns an iterator to the first element after the end of the jth
  //! column.
  const_column_iterator col_end(size_type /*unused*/ = 0) const { return A::end(); }
  column_iterator col_end(size_type /*unused*/ = 0) { return A::end(); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! jth column.
  const_column_iterator ccol_end(size_type /*unused*/ = 0) const { return A::cend(); }
  //! \brief Getters.
  typename A::value_type x() const { return A::operator[](0); }
  typename A::value_type y() const { return A::operator[](1); }
  typename A::value_type z() const { return A::operator[](2); }
  typename A::value_type w() const { return A::operator[](3); }
  typename A::value_type &x() { return A::operator[](0); }
  typename A::value_type &y() { return A::operator[](1); }
  typename A::value_type &z() { return A::operator[](2); }
  typename A::value_type &w() { return A::operator[](3); }
};

namespace stack {
template <typename T, size_type M> using Vector = VectorInterface<Array<T, M>>;

template <typename T> using Vec2 = Vector<T, 2>;
template <typename T> using Vec3 = Vector<T, 3>;
template <typename T> using Vec4 = Vector<T, 4>;
template <typename T> using Vec5 = Vector<T, 5>;
template <typename T> using Vec6 = Vector<T, 6>;

// Stream out
template <typename T, size_type M>
std::ostream &operator<<(std::ostream &stream, const Vector<T, M> &v) {
  const char *sep = "[";
  for (const auto &x : v) {
    stream << sep << x;
    sep = ", ";
  }
  return stream << "]";
}

//! \brief Returns the cross-product of a and b.
template <typename T, typename U>
Vec3<decltype(T(0) * U(0))> cross(const Vec3<T> &a, const Vec3<U> &b) {
  Vec3<decltype(T(0) * U(0))> out;

  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];

  return out;
}

//! \brief Returns the triple-product of a, b and c (a . (b x c)).
template <typename T, typename U, typename V>
decltype(T(0) * U(0) * V(0)) triple(const Vec3<T> &a, const Vec3<U> &b, const Vec3<V> &c) {
  return dot(a, cross(b, c));
}

//! \brief Returns the solid angle captured by the 3 vertices given as
//! parameters
template <typename T, typename U, typename V>
double solid(const Vec3<T> &a, const Vec3<U> &b, const Vec3<V> &c) {
  using std::abs;
  using std::atan;
  double na = norm(a), nb = norm(b), nc = norm(c);
  double out = 2. * atan(abs(triple(a, b, c)) /
                         (na * nb * nc + na * dot(b, c) + nb * dot(a, c) + nc * dot(a, b)));

  if (out < 0.) {
    return (out + M_PI);
  }
  { return out; }
}
} // namespace stack

namespace heap {
template <typename T> using Vector = VectorInterface<Array<1, T>>;
}

namespace shallow {
template <typename T> using Vector = VectorInterface<Array<1, T>>;
}

// Additional definitions
using Vec2i = stack::Vec2<int>;
using Vec2u = stack::Vec2<unsigned int>;
using Vec2f = stack::Vec2<float>;
using Vec3i = stack::Vec3<int>;
using Vec3f = stack::Vec3<float>;
using Vec4f = stack::Vec4<float>;
using Vec2d = stack::Vec2<double>;
using Vec3d = stack::Vec3<double>;
using Vec4d = stack::Vec4<double>;

using Vec2w = stack::Vec2<uint16_t>;
using Vec3w = stack::Vec3<uint16_t>;
using Vec4w = stack::Vec4<uint16_t>;

using SizeVector = std::vector<Vec2i>;

//! \brief Dot product.
template <typename Iterator1, typename Iterator2,
          typename std::enable_if<std::is_floating_point<typename Iterator1::value_type>::value &&
                                      std::is_floating_point<typename Iterator2::value_type>::value,
                                  int>::type = 0>
auto dot_product(Iterator1 first1, Iterator1 last1, Iterator2 first2) {
  using value_type = typename Iterator1::value_type;
  return std::inner_product(first1, last1, first2, value_type());
}

template <
    typename Iterator1, typename Iterator2,
    typename std::enable_if<!std::is_floating_point<typename Iterator1::value_type>::value &&
                                !std::is_floating_point<typename Iterator2::value_type>::value,
                            int>::type = 0>
auto dot_product(Iterator1 first1, Iterator1 last1, Iterator2 first2) {
  using T1 = typename Iterator1::value_type;
  using T2 = typename Iterator2::value_type;

  return std::inner_product(
      first1, last1, first2, T1(0), [](const T1 &v1, const T2 &v2) { return (v1 + v2); },
      [](const T1 &v1, const T2 &v2) { return (v1 * std::conj(v2)); });
}

template <typename V1, typename V2> auto dot(const V1 &v1, const V2 &v2) {
  return dot_product(v1.begin(), v1.end(), v2.begin());
}

//! \brief Returns ||v||**2.
template <typename V> auto norm2(const V &v) {
  using std::abs;
  return abs(dot(v, v));
}
//! \brief Returns ||v||.
template <typename V> auto norm(const V &v) {
  using std::sqrt;
  return sqrt(norm2(v));
}
//! \brief Returns ||v||inf.
template <typename V> auto norm_inf(const V &v) {
  using std::abs;
  return abs(
      *std::max_element(v.begin(), v.end(), [](auto v1, auto v2) { return abs(v1) < abs(v2); }));
}
//! \brief Returns v / ||v|| and optionally ||v||.
template <typename V, typename U = decltype(std::abs(std::declval<typename V::value_type>()))>
V unit(const V &v, U *n = nullptr) {
  U m = norm(v);
  if (n) {
    *n = m;
  }
  return v / m;
}
//! \brief Normalizes v and optionally returns ||v||.
template <typename V, typename U = decltype(std::abs(std::declval<typename V::value_type>()))>
V &normalize(V &v, U *n = nullptr) {
  U m = norm(v);
  if (n) {
    *n = m;
  }
  v /= m;
  return v;
}
//! \brief Returns the cosine of the angle between the two vectors given as
//! arguments.
//
// This is also known as the normalized inner product of two vectors, or the
// cosine measure.
template <typename V1, typename V2>
decltype(typename V1::value_type(0) * typename V2::value_type(0)) cosAngle(const V1 &v1,
                                                                           const V2 &v2) {
  return dot(v1, v2) / sqrt(norm2(v1) * norm2(v2));
}

//! \brief Returns the angle between the two vectors given as
//! arguments.
template <typename V1, typename V2> auto angle(const V1 &v1, const V2 &v2) {
  using std::acos;
  using std::min;
  return acos(min(1.F, cosAngle(v1, v2)));
}

} // namespace TMIV::Common

#endif
