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

#ifndef TMIV_COMMON_VECTOR_H
#define TMIV_COMMON_VECTOR_H

#include "Array.h"
#include "Math.h"

#include <ostream>
#include <type_traits>

namespace TMIV::Common {
template <typename A> class VectorInterface : public A {
public:
  template <typename U>
  using promoted_type = VectorInterface<typename A::template promoted_type<U>>;

  using A::A;
  VectorInterface() = default;

  using A::operator=;

  auto operator=(const A &a) noexcept -> auto & {
    A::operator=(a);
    return *this;
  }

  auto operator=(A &&a) noexcept -> auto & {
    A::operator=(std::move(a));
    return *this;
  }

  [[nodiscard]] constexpr auto m() const noexcept { return A::size(0); }
  [[nodiscard]] constexpr auto n() const noexcept { return size_t{1}; }

  using A::resize;
  void resize(size_t a, size_t /* b */) noexcept { A::resize({a}); }

  [[nodiscard]] auto row_begin(size_t i) const noexcept { return A::begin() + i; }
  [[nodiscard]] auto row_begin(size_t i) noexcept { return A::begin() + i; }
  [[nodiscard]] auto crow_begin(size_t i) const { return A::begin() + i; }
  [[nodiscard]] auto row_end(size_t i) const noexcept { return A::begin() + (i + 1); }
  [[nodiscard]] auto row_end(size_t i) noexcept { return A::begin() + (i + 1); }
  [[nodiscard]] auto crow_end(size_t i) const noexcept { return A::begin() + (i + 1); }
  [[nodiscard]] auto col_begin(size_t /* j */) const noexcept { return A::begin(); }
  [[nodiscard]] auto col_begin(size_t /* j */) noexcept { return A::begin(); }
  [[nodiscard]] auto ccol_begin(size_t /* j */) const noexcept { return A::cbegin(); }
  [[nodiscard]] auto col_end(size_t /* j */) const noexcept { return A::end(); }
  [[nodiscard]] auto col_end(size_t /* j */) noexcept { return A::end(); }
  [[nodiscard]] auto ccol_end(size_t /* j */) const noexcept { return A::cend(); }

  [[nodiscard]] auto x() const noexcept -> decltype(auto) { return A::operator[](0); }
  [[nodiscard]] auto y() const noexcept -> decltype(auto) { return A::operator[](1); }
  [[nodiscard]] auto z() const noexcept -> decltype(auto) { return A::operator[](2); }
  [[nodiscard]] auto w() const noexcept -> decltype(auto) { return A::operator[](3); }
  [[nodiscard]] auto x() noexcept -> decltype(auto) { return A::operator[](0); }
  [[nodiscard]] auto y() noexcept -> decltype(auto) { return A::operator[](1); }
  [[nodiscard]] auto z() noexcept -> decltype(auto) { return A::operator[](2); }
  [[nodiscard]] auto w() noexcept -> decltype(auto) { return A::operator[](3); }
};

namespace stack {
template <typename T, size_t M> using Vector = VectorInterface<Array<T, M>>;

template <typename T> using Vec2 = Vector<T, 2>;
template <typename T> using Vec3 = Vector<T, 3>;
template <typename T> using Vec4 = Vector<T, 4>;
template <typename T> using Vec5 = Vector<T, 5>;
template <typename T> using Vec6 = Vector<T, 6>;

// Stream out
template <typename T, size_t M>
auto operator<<(std::ostream &stream, const Vector<T, M> &v) -> std::ostream & {
  const char *sep = "[";
  for (const auto &x : v) {
    stream << sep << x;
    sep = ", ";
  }
  return stream << "]";
}

// Returns the cross-product of a and b
template <typename T, typename U>
auto cross(const Vec3<T> &a, const Vec3<U> &b) -> Vec3<std::common_type_t<T, U>> {
  Vec3<std::common_type_t<T, U>> out;

  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];

  return out;
}

// Returns the triple-product of a, b and c (a . (b x c))
template <typename T, typename U, typename V>
auto triple(const Vec3<T> &a, const Vec3<U> &b, const Vec3<V> &c) {
  return dot(a, cross(b, c));
}

// Returns the solid angle captured by the 3 vertices given as parameters
template <typename T, typename U, typename V>
auto solid(const Vec3<T> &a, const Vec3<U> &b, const Vec3<V> &c) -> double {
  using std::abs;
  using std::atan;
  double na = norm(a);
  double nb = norm(b);
  double nc = norm(c);
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

// Additional definitions
using Vec2i = stack::Vec2<int32_t>;
using Vec2u = stack::Vec2<uint32_t>;
using Vec2f = stack::Vec2<float>;
using Vec3i = stack::Vec3<int32_t>;
using Vec4i = stack::Vec4<int32_t>;
using Vec3f = stack::Vec3<float>;
using Vec4f = stack::Vec4<float>;
using Vec2d = stack::Vec2<double>;
using Vec3d = stack::Vec3<double>;
using Vec4d = stack::Vec4<double>;

using Vec2w = stack::Vec2<uint16_t>;
using Vec3w = stack::Vec3<uint16_t>;
using Vec4w = stack::Vec4<uint16_t>;

using SizeVector = std::vector<Vec2i>;

template <typename Iterator1, typename Iterator2>
auto dot_product(Iterator1 first1, Iterator1 last1, Iterator2 first2) {
  using value_type = std::common_type_t<typename std::iterator_traits<Iterator1>::value_type,
                                        typename std::iterator_traits<Iterator2>::value_type>;
  return std::inner_product(
      first1, last1, first2, value_type{}, std::plus<>{}, [](auto v1, auto v2) {
        if constexpr (std::is_arithmetic_v<decltype(v1)> && std::is_arithmetic_v<decltype(v2)>) {
          return v1 * v2;
        } else {
          using std::conj;
          return v1 * conj(v2);
        }
      });
}

template <typename V1, typename V2> auto dot(const V1 &v1, const V2 &v2) {
  return dot_product(v1.begin(), v1.end(), v2.begin());
}

// Returns ||v||**2
template <typename V> auto norm2(const V &v) {
  using std::abs;
  return abs(dot(v, v));
}
// Returns ||v||
template <typename V> auto norm(const V &v) {
  using std::sqrt;
  return sqrt(norm2(v));
}
// Returns ||v||inf
template <typename V> auto norm_inf(const V &v) {
  using std::abs;
  return abs(
      *std::max_element(v.begin(), v.end(), [](auto v1, auto v2) { return abs(v1) < abs(v2); }));
}
// Returns v / ||v|| and optionally ||v||
template <typename V, typename U = typename V::value_type>
auto unit(const V &v, U *n = nullptr) -> V {
  U m = norm(v);
  if (n) {
    *n = m;
  }
  return v / m;
}
// Normalizes v and optionally returns ||v||
template <typename V, typename U = typename V::value_type>
auto normalize(V &v, U *n = nullptr) -> V & {
  U m = norm(v);
  if (n) {
    *n = m;
  }
  v /= m;
  return v;
}
// Returns the cosine of the angle between the two vectors given as arguments
//
// This is also known as the normalized inner product of two vectors, or the cosine measure
template <typename V1, typename V2,
          typename R = std::common_type_t<typename V1::value_type, typename V2::value_type>>
auto cosAngle(const V1 &v1, const V2 &v2) -> R {
  using std::sqrt;
  return static_cast<R>(dot(v1, v2) / sqrt(norm2(v1) * norm2(v2)));
}

// Returns the angle between the two vectors given as arguments
template <typename V1, typename V2,
          typename R = std::common_type_t<typename V1::value_type, typename V2::value_type>>
auto angle(const V1 &v1, const V2 &v2) -> R {
  using std::acos;
  using std::min;
  return acos(min(R{1.F}, cosAngle(v1, v2)));
}
} // namespace TMIV::Common

#endif
