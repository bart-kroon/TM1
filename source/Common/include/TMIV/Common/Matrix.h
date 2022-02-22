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

#ifndef TMIV_COMMON_MATRIX_H
#define TMIV_COMMON_MATRIX_H

#include <TMIV/Common/Array.h>
#include <TMIV/Common/Common.h>

namespace TMIV::Common {
template <typename A> class MatrixInterface : public A {
public:
  using value_type = typename A::value_type;

  using A::A;
  MatrixInterface() : A() {}
  explicit MatrixInterface(const A &a) : A(a) {}
  explicit MatrixInterface(A &&a) noexcept : A(std::move(a)) {}

  using A::operator=;

  auto operator=(const A &a) noexcept -> auto & {
    A::operator=(a);
    return *this;
  }

  auto operator=(A &&a) noexcept -> auto & {
    A::operator=(std::move(a));
    return *this;
  }

  [[nodiscard]] auto m() const noexcept { return A::size(0); }
  [[nodiscard]] auto n() const noexcept { return A::size(1); }
  [[nodiscard]] auto height() const noexcept { return A::size(0); }
  [[nodiscard]] auto width() const noexcept { return A::size(1); }

  using A::resize;
  void resize(size_t a, size_t b) { A::resize({a, b}); }

  [[nodiscard]] auto row_begin(size_t i) const noexcept { return A::template dim_begin<1>(i); }
  [[nodiscard]] auto row_begin(size_t i) noexcept { return A::template dim_begin<1>(i); }
  [[nodiscard]] auto crow_begin(size_t i) const noexcept { return A::template cdim_begin<1>(i); }
  [[nodiscard]] auto row_end(size_t i) const noexcept { return A::template dim_end<1>(i); }
  [[nodiscard]] auto row_end(size_t i) noexcept { return A::template dim_end<1>(i); }
  [[nodiscard]] auto crow_end(size_t i) const noexcept { return A::template cdim_end<1>(i); }
  [[nodiscard]] auto col_begin(size_t j) const noexcept { return A::template dim_begin<0>(j); }
  [[nodiscard]] auto col_begin(size_t j) noexcept { return A::template dim_begin<0>(j); }
  [[nodiscard]] auto ccol_begin(size_t j) const noexcept { return A::template cdim_begin<0>(j); }
  [[nodiscard]] auto col_end(size_t j) const noexcept { return A::template dim_end<0>(j); }
  [[nodiscard]] auto col_end(size_t j) noexcept { return A::template dim_end<0>(j); }
  [[nodiscard]] auto ccol_end(size_t j) const noexcept { return A::template cdim_end<0>(j); }

  [[nodiscard]] auto isRow() const -> bool { return m() == 1; }
  [[nodiscard]] auto isColumn() const -> bool { return n() == 1; }
  [[nodiscard]] auto isPositive() const -> bool;
};

namespace stack {
template <typename T, size_t M, size_t N> using Matrix = MatrixInterface<Array<T, M, N>>;
template <typename T> using Mat2x2 = Matrix<T, 2, 2>;
template <typename T> using Mat2x3 = Matrix<T, 2, 3>;
template <typename T> using Mat3x3 = Matrix<T, 3, 3>;
} // namespace stack

namespace heap {
template <typename T> using Matrix = MatrixInterface<Array<2, T>>;
}

// Additional definitions
using Mat2x2i = stack::Mat2x2<int32_t>;
using Mat3x3i = stack::Mat3x3<int32_t>;
using Mat2x2f = stack::Mat2x2<float>;
using Mat3x3f = stack::Mat3x3<float>;
template <typename T = DefaultElement> using Mat = heap::Matrix<T>;

// Returns the type of the transpose of the matrix given as input
template <typename T, size_t M, size_t N>
auto transpose_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, N, M>;
template <typename T> auto transpose_type(heap::Matrix<T>) -> heap::Matrix<T>;

// Returns the transpose of the matrix given as input
template <typename Mat1, typename Mat2> auto transpose(const Mat1 &in, Mat2 &out) -> Mat2 &;
template <typename Mat> auto transpose(const Mat &m) -> decltype(transpose_type(Mat()));

} // namespace TMIV::Common

#include "Matrix.hpp"

#endif
