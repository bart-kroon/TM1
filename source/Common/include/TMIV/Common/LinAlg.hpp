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

namespace TMIV::Common {
namespace detail {
template <typename R, typename T, typename X> auto elementwiseProduct(const T &A, X B) -> R {
  auto result = R{A};
  for (auto &y : result) {
    y *= B;
  }
  return result;
}
template <typename T, typename X> auto elementwiseMin(const T &A, X B) -> T {
  auto result = A;
  for (auto &y : result) {
    y = std::min(y, B);
  }
  return result;
}
template <typename T, typename X> auto elementwiseMax(const T &A, X B) -> T {
  auto result = A;
  for (auto &y : result) {
    y = std::max(y, B);
  }
  return result;
}
template <typename T> auto elementwiseMin(const T &A, const T &B) -> T {
  auto result = A;
  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = std::min(result[i], B[i]);
  }
  return result;
}
template <typename T> auto elementwiseMax(const T &A, const T &B) -> T {
  auto result = A;
  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = std::max(result[i], B[i]);
  }
  return result;
}
template <typename T> auto elementwiseAbs(const T &A) -> T {
  auto result = A;
  for (auto &y : result) {
    y = std::abs(y);
  }
  return result;
}
} // namespace detail

template <typename T, typename X, typename>
auto operator*(const heap::Vector<T> &A, X B) -> heap::Vector<std::common_type_t<T, X>> {
  using R = heap::Vector<std::common_type_t<T, X>>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, typename X, typename>
auto operator*(X B, const heap::Vector<T> &A) -> heap::Vector<std::common_type_t<T, X>> {
  using R = heap::Vector<std::common_type_t<T, X>>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, size_t M, typename X, typename>
auto operator*(const stack::Vector<T, M> &A, X B) -> stack::Vector<std::common_type_t<T, X>, M> {
  using R = stack::Vector<std::common_type_t<T, X>, M>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, size_t M, typename X, typename>
auto operator*(X B, const stack::Vector<T, M> &A) -> stack::Vector<std::common_type_t<T, X>, M> {
  using R = stack::Vector<std::common_type_t<T, X>, M>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, typename X, typename>
auto operator*(const heap::Matrix<T> &A, X B) -> heap::Matrix<std::common_type_t<T, X>> {
  using R = heap::Matrix<std::common_type_t<T, X>>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, typename X, typename>
auto operator*(X B, const heap::Matrix<T> &A) -> heap::Matrix<std::common_type_t<T, X>> {
  using R = heap::Matrix<std::common_type_t<T, X>>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, size_t M, size_t N, typename X, typename>
auto operator*(const stack::Matrix<T, M, N> &A, X B)
    -> stack::Matrix<std::common_type_t<T, X>, M, N> {
  using R = stack::Matrix<std::common_type_t<T, X>, M, N>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T, size_t M, size_t N, typename X, typename>
auto operator*(X B, const stack::Matrix<T, M, N> &A)
    -> stack::Matrix<std::common_type_t<T, X>, M, N> {
  using R = stack::Matrix<std::common_type_t<T, X>, M, N>;
  return detail::elementwiseProduct<R>(A, B);
}

template <typename T> auto min(const heap::Vector<T> &A, T B) -> heap::Vector<T> {
  return detail::elementwiseMin(A, B);
}

template <typename T> auto min(T B, const heap::Vector<T> &A) -> heap::Vector<T> {
  return detail::elementwiseMin(A, B);
}

template <typename T, size_t M> auto min(const stack::Vector<T, M> &A, T B) -> stack::Vector<T, M> {
  return detail::elementwiseMin(A, B);
}
template <typename T, size_t M> auto min(T B, const stack::Vector<T, M> &A) -> stack::Vector<T, M> {
  return detail::elementwiseMin(A, B);
}

template <typename T> auto max(const heap::Vector<T> &A, T B) -> heap::Vector<T> {
  return detail::elementwiseMax(A, B);
}

template <typename T> auto max(T B, const heap::Vector<T> &A) -> heap::Vector<T> {
  return detail::elementwiseMax(A, B);
}

template <typename T, size_t M> auto max(const stack::Vector<T, M> &A, T B) -> stack::Vector<T, M> {
  return detail::elementwiseMax(A, B);
}

template <typename T, size_t M> auto max(T B, const stack::Vector<T, M> &A) -> stack::Vector<T, M> {
  return detail::elementwiseMax(A, B);
}

template <typename T>
auto min(const heap::Vector<T> &A, const heap::Vector<T> &B) -> heap::Vector<T> {
  return detail::elementwiseMin(A, B);
}

template <typename T, size_t M>
auto min(const stack::Vector<T, M> &A, const stack::Vector<T, M> &B) -> stack::Vector<T, M> {
  return detail::elementwiseMin(A, B);
}

template <typename T>
auto max(const heap::Vector<T> &A, const heap::Vector<T> &B) -> heap::Vector<T> {
  return detail::elementwiseMax(A, B);
}
template <typename T, size_t M>
auto max(const stack::Vector<T, M> &A, const stack::Vector<T, M> &B) -> stack::Vector<T, M> {
  return detail::elementwiseMax(A, B);
}

template <typename T> auto abs(const heap::Vector<T> &A) -> heap::Vector<T> {
  return detail::elementwiseAbs(A);
}
template <typename T, size_t M> auto abs(const stack::Vector<T, M> &A) -> stack::Vector<T, M> {
  return detail::elementwiseAbs(A);
}

namespace detail {
// Matrix product A_ij B_jk == C_ik
template <typename MAT1, typename MAT2, typename MAT3>
auto matrixProduct(const MAT1 &A, const MAT2 &B, MAT3 &C) -> MAT3 & {
  C.resize(A.m(), B.n());

  using T = typename MAT3::value_type;

  for (size_t i = 0; i < C.m(); i++) {
    for (size_t j = 0; j < C.n(); j++) {
      C(i, j) = std::inner_product(A.crow_begin(i), A.crow_end(i), B.ccol_begin(j), T{});
    }
  }
  return C;
}
} // namespace detail

template <typename T, typename U>
auto operator*(const heap::Matrix<T> &A, const heap::Vector<U> &B)
    -> heap::Vector<std::common_type_t<T, U>> {
  heap::Vector<std::common_type_t<T, U>> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U, size_t M>
auto operator*(const heap::Matrix<T> &A, const stack::Vector<U, M> &B)
    -> heap::Vector<std::common_type_t<T, U>> {
  heap::Vector<std::common_type_t<T, U>> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U>
auto operator*(const heap::Matrix<T> &A, const heap::Matrix<U> &B)
    -> heap::Matrix<std::common_type_t<T, U>> {
  heap::Matrix<std::common_type_t<T, U>> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U, size_t M, size_t N>
auto operator*(const heap::Matrix<T> &A, const stack::Matrix<U, M, N> &B)
    -> heap::Matrix<std::common_type_t<T, U>> {
  heap::Matrix<std::common_type_t<T, U>> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U, size_t M, size_t N>
auto operator*(const stack::Matrix<T, M, N> &A, const stack::Vector<U, N> &B)
    -> stack::Vector<std::common_type_t<T, U>, M> {
  stack::Vector<std::common_type_t<T, U>, M> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U, size_t M, size_t N>
auto operator*(const stack::Matrix<T, M, N> &A, const heap::Vector<U> &B)
    -> heap::Vector<std::common_type_t<T, U>> {
  heap::Vector<std::common_type_t<T, U>> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U, size_t M, size_t N>
auto operator*(const stack::Matrix<T, M, N> &A, const heap::Matrix<U> &B)
    -> heap::Matrix<std::common_type_t<T, U>> {
  heap::Matrix<std::common_type_t<T, U>> out;
  return detail::matrixProduct(A, B, out);
}

template <typename T, typename U, size_t M, size_t N, size_t O>
auto operator*(const stack::Matrix<T, M, N> &A, const stack::Matrix<U, N, O> &B)
    -> stack::Matrix<std::common_type_t<T, U>, M, O> {
  stack::Matrix<std::common_type_t<T, U>, M, O> out;
  return detail::matrixProduct(A, B, out);
}

namespace detail {
template <typename T, size_t M> auto square_type(stack::Vector<T, M>) -> stack::Matrix<T, M, M>;
template <typename T, size_t M, size_t N>
auto square_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, M, M>;
} // namespace detail

template <typename MAT1, typename MAT2> void square(const MAT1 &A, MAT2 &out) {
  out.resize(A.m(), A.m());

  using T = typename MAT2::value_type;

  for (size_t i = 0; i < out.m(); i++) {
    for (size_t j = 0; j < out.n(); j++) {
      out(i, j) = std::inner_product(A.crow_begin(i), A.crow_end(i), A.crow_begin(j), T{});
    }
  }
}

template <typename MAT> auto square(const MAT &A) {
  decltype(detail::square_type(MAT())) out;
  square(A, out);
  return out;
}

namespace detail {
template <typename T, size_t M, size_t N>
auto transquare_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, N, N>;
template <typename T> auto transquare_type(heap::Matrix<T>) -> heap::Matrix<T>;
} // namespace detail

template <typename MAT1, typename MAT2> void transquare(const MAT1 &A, MAT2 &out) {
  out.resize(A.n(), A.n());

  using T = typename MAT2::value_type;

  for (size_t i = 0; i < out.m(); i++) {
    for (size_t j = 0; j < out.n(); j++) {
      out(i, j) = std::inner_product(A.ccol_begin(i), A.ccol_end(i), A.ccol_begin(j), T{});
    }
  }
}

template <typename MAT> auto transquare(const MAT &A) {
  decltype(detail::transquare_type(MAT())) out;
  transquare(A, out);
  return out;
}

namespace detail {
template <typename MatrixA, typename MatrixLU>
auto PLU(MatrixA &A, MatrixLU &LU, ::std::vector<int32_t> &P) -> int32_t {
  using T = typename MatrixLU::value_type;
  using size_type = size_t;
  using std::abs;

  int32_t nb_permutations = 0;
  size_type n = A.m();

  P.resize(A.m());
  std::iota(P.begin(), P.end(), 0);

  std::copy(A.begin(), A.end(), LU.begin());

  for (size_type k = 0; k < n; k++) {
    auto iter = std::max_element(LU.col_begin(k) + k, LU.col_end(k),
                                 [](T x, T y) { return abs(x) < abs(y); });
    T pivot = *iter;

    if (abs(std::numeric_limits<T>::epsilon()) < abs(pivot)) {
      auto p = static_cast<size_type>(iter - LU.col_begin(k));

      if (p != k) {
        std::swap(P[k], P[p]);
        std::swap_ranges(LU.row_begin(k), LU.row_end(k), LU.row_begin(p));
        nb_permutations++;
      }

      for (size_type i = k + 1; i < n; i++) {
        T factor = (LU(i, k) /= pivot);
        std::transform(LU.crow_begin(i) + (k + 1), LU.crow_end(i), LU.crow_begin(k) + (k + 1),
                       LU.row_begin(i) + (k + 1),
                       [factor](T v1, T v2) { return v1 - factor * v2; });
      }
    } else {
      return -1;
    }
  }

  return nb_permutations;
}
} // namespace detail

template <typename MAT1, typename MAT2>
auto PLU(const MAT1 &A, MAT2 &LU, std::vector<int32_t> &P) -> int32_t {
  LU.resize(A.m(), A.n());
  return detail::PLU(A, LU, P);
}

template <typename MAT1, typename MAT2>
auto PLU(const MAT1 &A, MAT2 &L, MAT2 &U, MAT2 &P) -> int32_t {
  using size_type = size_t;
  using T1 = typename MAT2::value_type;
  using T2 = typename MAT2::value_type;

  heap::Matrix<T1> LU;
  std::vector<int32_t> p;

  int32_t ret = PLU(A, LU, p);

  L.resize(A.m(), A.n());
  U.resize(A.m(), A.n());
  P.resize(A.m(), A.n());

  std::fill(P.begin(), P.end(), T2(0));

  for (size_type i = 0; i < A.m(); i++) {
    P(i, p[i]) = 1;

    for (size_type j = 0; j < A.m(); j++) {
      if (j < i) {
        L(i, j) = LU(i, j);
        U(i, j) = T2(0);
      } else if (j == i) {
        L(i, i) = T2(1);
        U(i, j) = LU(i, j);
      } else {
        L(i, j) = T2(0);
        U(i, j) = LU(i, j);
      }
    }
  }

  return ret;
}

namespace detail {
template <typename MatrixA, typename MatrixOut> auto chol(MatrixA &A, MatrixOut &out) -> int32_t {
  using size_type = size_t;
  using T = typename MatrixOut::value_type;
  using std::sqrt;

  size_type n = A.m();
  T x;

  // First column
  out(0, 0) = x = sqrt(A(0, 0));

  for (size_type i = 1; i < n; i++) {
    out(i, 0) = A(i, 0) / x;
  }

  // Remaining lower part
  for (size_type j = 1; j < n; j++) {
    for (size_type i = 0; i < j; i++) {
      out(i, j) = 0;
    }

    out(j, j) = x =
        sqrt(A(j, j) - dot_product(out.crow_begin(j), out.crow_begin(j) + j, out.crow_begin(j)));

    for (size_type i = (j + 1); i < n; i++) {
      out(i, j) =
          (A(i, j) - dot_product(out.crow_begin(i), out.crow_begin(i) + j, out.crow_begin(j))) / x;
    }
  }

  return 0;
}
} // namespace detail

template <typename MAT1, typename MAT2> auto chol(const MAT1 &A, MAT2 &out) -> int32_t {
  out.resize(A.m(), A.m());
  return detail::chol(A, out);
}

template <typename MAT> auto chol(const MAT &A, int32_t *info) -> MAT {
  MAT out;

  int32_t information = chol(A, out);
  if (info) {
    *info = information;
  }

  return out;
}

namespace detail {
template <typename MatrixA> auto det(MatrixA &A, int32_t *info) {
  using T = typename MatrixA::value_type;
  heap::Matrix<T> LU;
  std::vector<int32_t> P;
  int32_t n = PLU(A, LU, P);
  T d = std::accumulate(LU.diag_begin(), LU.diag_end(), T{1}, [](T v1, T v2) { return (v1 * v2); });

  if (n % 2) {
    d = -d;
  }

  if (info) {
    (*info) = 0;
  }

  return d;
}
} // namespace detail

template <typename MAT> auto det(const MAT &A, int32_t *info) -> typename MAT::value_type {
  int32_t information = 0;
  typename MAT::value_type out = 0;

  if (A.isPositive()) {
    MAT L = chol(A, &information);

    if (!information) {
      out = 1;
      std::for_each(L.diag_begin(), L.diag_end(), [&out](typename MAT::value_type t) { out *= t; });
      out = sqr(out);
    }
  } else {
    out = detail::det(A, &information);
  }

  if (info) {
    (*info) = information;
  }

  return out;
}

namespace detail {
template <typename MatrixA, typename MatrixB, typename MatrixOut>
auto mldivide(MatrixA &A, MatrixB &B, MatrixOut &out) -> int32_t {
  using T = typename MatrixOut::value_type;
  using size_type = size_t;

  // PLU decomposition
  heap::Matrix<T> LU;
  std::vector<int32_t> P;

  PLU(A, LU, P);

  // Solve Y = inv(L) * B
  heap::Matrix<T> Y({B.m(), B.n()});

  for (size_type i = 0; i < Y.m(); i++) {
    for (size_type j = 0; j < Y.n(); j++) {
      Y(i, j) = B(P[i], j);

      if (0 < i) {
        Y(i, j) -= std::inner_product(LU.crow_begin(i), LU.crow_begin(i) + i, Y.ccol_begin(j), T{});
      }
    }
  }

  // Solve out = inv(U) * Y
  size_type m = out.m();

  for (size_type i = 0; i < Y.m(); i++) {
    for (size_type j = 0; j < out.n(); j++) {
      size_type k = m - i - 1;

      out(k, j) = Y(k, j) / LU(k, k);

      if (0 < i) {
        out(k, j) -= std::inner_product(LU.crow_begin(k) + m - i, LU.crow_end(k),
                                        out.ccol_begin(j) + m - i, T{}) /
                     LU(k, k);
      }
    }
  }

  return 0;
}
} // namespace detail

template <typename MAT1, typename MAT2, typename MAT3>
auto mldivide(const MAT1 &A, const MAT2 &B, MAT3 &out) -> int32_t {
  out.resize(B.m(), B.n());
  return detail::mldivide(A, B, out);
}

template <typename MAT1, typename MAT2>
auto mldivide(const MAT1 &A, const MAT2 &B, int32_t *info) -> MAT2 {
  MAT2 out;

  int32_t information = mldivide(A, B, out);
  if (info) {
    *info = information;
  }

  return out;
}

namespace detail {
template <typename MatrixA, typename MatrixB, typename MatrixOut>
auto mrdivide(MatrixA &A, MatrixB &B, MatrixOut &out) -> int32_t {
  using T = typename MatrixOut::value_type;
  using size_type = size_t;

  // PLU decomposition
  heap::Matrix<T> LU;
  std::vector<int32_t> P;

  PLU(B, LU, P);

  // Solve Y = A * inv(U)
  heap::Matrix<T> Y({A.m(), A.n()});

  for (size_type i = 0; i < Y.m(); i++) {
    for (size_type j = 0; j < Y.n(); j++) {
      Y(i, j) = A(i, j) / LU(j, j);

      if (0 < j) {
        Y(i, j) -= std::inner_product(Y.crow_begin(i), Y.crow_begin(i) + j, LU.ccol_begin(j), T{}) /
                   LU(j, j);
      }
    }
  }

  // Solve out = Y * inv(L)
  size_type n = out.n();

  for (size_type i = 0; i < Y.m(); i++) {
    for (size_type j = 0; j < n; j++) {
      size_type k = n - j - 1;

      if (0 < j) {
        Y(i, k) -= std::inner_product(Y.crow_begin(i) + n - j, Y.crow_end(i),
                                      LU.ccol_begin(k) + n - j, T{});
      }

      out(i, P[k]) = Y(i, k);
    }
  }

  return 0;
}
} // namespace detail

template <typename MAT1, typename MAT2, typename MAT3>
auto mrdivide(const MAT1 &A, const MAT2 &B, MAT3 &out) -> int32_t {
  out.resize(A.m(), A.n());
  return detail::mrdivide(A, B, out);
}

template <typename MAT1, typename MAT2>
auto mrdivide(const MAT1 &A, const MAT2 &B, int32_t *info) -> MAT1 {
  MAT1 out;

  int32_t information = mrdivide(A, B, out);
  if (info) {
    *info = information;
  }

  return out;
}

namespace detail {
template <typename MatrixA, typename MatrixOut> auto inv(MatrixA &A, MatrixOut &out) -> int32_t {
  using T = typename MatrixA::value_type;

  auto I = A;
  I = 0;
  std::fill(I.diag_begin(), I.diag_end(), T{1});
  return mldivide(A, I, out);
}
} // namespace detail

template <typename MAT1, typename MAT2> auto inv(const MAT1 &A, MAT2 &out) -> int32_t {
  out.resize(A.m(), A.n());
  return detail::inv(A, out);
}

template <typename MAT> auto inv(const MAT &A, int32_t *info) -> MAT {
  MAT out;
  int32_t information = inv(A, out);

  if (info) {
    *info = information;
  }

  return out;
}
} // namespace TMIV::Common
