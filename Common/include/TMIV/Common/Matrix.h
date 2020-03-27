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

#ifndef _TMIV_COMMON_MATRIX_H_
#define _TMIV_COMMON_MATRIX_H_

#include "Array.h"
#include "Math.h"

namespace TMIV::Common {

struct Matrix {
  enum Property { None, Symmetric, Hermitian, Positive, Lower, Upper };
};

template <typename A> class MatrixInterface : public A {
public:
  using size_type = typename A::size_type;
  using const_row_iterator = typename A::const_dim_iterator;
  using row_iterator = typename A::dim_iterator;
  using const_column_iterator = typename A::const_dim_iterator;
  using column_iterator = typename A::dim_iterator;
  template <typename U>
  using promoted_type = MatrixInterface<typename A::template promoted_type<U>>;

public:
  using A::A;
  MatrixInterface() : A() {}
  explicit MatrixInterface(const A &a) : A(a) {}
  explicit MatrixInterface(A &&a) : A(std::move(a)) {}
  using A::operator=;
  auto operator=(const A &a) -> MatrixInterface & {
    A::operator=(a);
    return *this;
  }
  auto operator=(A &&a) -> MatrixInterface & {
    A::operator=(std::move(a));
    return *this;
  }
  //! \brief Returns the number of rows of the matrix.
  auto m() const -> size_type { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  auto n() const -> size_type { return A::size(1); }
  //! \brief Returns the number of rows of the matrix.
  auto height() const -> size_type { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  auto width() const -> size_type { return A::size(1); }
  //! \brief Returns the leading dimension of the matrix.
  auto lda() const -> size_type { return A::size(1); }
  //! \brief Overloaded resize operator.
  using A::resize;
  void resize(size_type a, size_type b) { A::resize({a, b}); }
  //! \brief Returns an iterator to the first element of the ith row.
  auto row_begin(size_type i) const -> const_row_iterator { return A::template dim_begin<1>(i); }
  auto row_begin(size_type i) -> row_iterator { return A::template dim_begin<1>(i); }
  //! \brief Returns a const iterator to the first element of the ith row.
  auto crow_begin(size_type i) const -> const_row_iterator { return A::template cdim_begin<1>(i); }
  //! \brief Returns an iterator to the first element after the end of the ith
  //! row.
  auto row_end(size_type i) const -> const_row_iterator { return A::template dim_end<1>(i); }
  auto row_end(size_type i) -> row_iterator { return A::template dim_end<1>(i); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! ith row.
  auto crow_end(size_type i) const -> const_row_iterator { return A::template cdim_end<1>(i); }
  //! \brief Returns an iterator to the first element of the jth column.
  auto col_begin(size_type j) const -> const_column_iterator { return A::template dim_begin<0>(j); }
  auto col_begin(size_type j) -> column_iterator { return A::template dim_begin<0>(j); }
  //! \brief Returns a const iterator to the first element of the jth column.
  auto ccol_begin(size_type j) const -> const_column_iterator {
    return A::template cdim_begin<0>(j);
  }
  //! \brief Returns an iterator to the first element after the end of the jth
  //! column.
  auto col_end(size_type j) const -> const_column_iterator { return A::template dim_end<0>(j); }
  auto col_end(size_type j) -> column_iterator { return A::template dim_end<0>(j); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! jth column.
  auto ccol_end(size_type j) const -> const_column_iterator { return A::template cdim_end<0>(j); }
  //! \brief Returns true if the matrix is a row.
  auto isRow() const -> bool { return (m() == 1); }
  //! \brief Returns true if the matrix is a column.
  auto isColumn() const -> bool { return (n() == 1); }
  //! \brief Returns true if the matrix is symmetric.
  auto isSymmetric() const -> bool {
    return (A::getProperty() == Matrix::Property::Symmetric) ||
           (A::getProperty() == Matrix::Property::Positive);
  }
  //! \brief Returns true if the matrix is hermitian.
  auto isHermitian() const -> bool { return (A::getProperty() == Matrix::Property::Hermitian); }
  //! \brief Returns true if the matrix is positive.
  auto isPositive() const -> bool { return (A::getProperty() == Matrix::Property::Positive); }
  //! \brief Returns true if the matrix is lower.
  auto isLower() const -> bool { return (A::getProperty() == Matrix::Property::Lower); }
  //! \brief Returns true if the matrix is upper.
  auto isUpper() const -> bool { return (A::getProperty() == Matrix::Property::Upper); }
  //! \brief Returns true if the matrix is triangular.
  auto isTriangular() const -> bool {
    return (A::getProperty() == Matrix::Property::Lower) ||
           (A::getProperty() == Matrix::Property::Upper);
  }
  static auto diag(const std::vector<typename A::value_type> &v) -> MatrixInterface {
    MatrixInterface out;

    out.resize(v.size(), v.size());
    std::fill(out.begin(), out.end(), 0);
    std::copy(v.begin(), v.end(), out.diag_begin());

    return out;
  }
};

namespace stack {
template <typename T, size_type M, size_type N> using Matrix = MatrixInterface<Array<T, M, N>>;

template <typename T> using Mat2x2 = Matrix<T, 2, 2>;
template <typename T> using Mat3x3 = Matrix<T, 3, 3>;
template <typename T> using Mat4x4 = Matrix<T, 4, 4>;

} // namespace stack

namespace heap {
template <typename T> using Matrix = MatrixInterface<Array<2, T>>;
}

namespace shallow {
template <typename T> using Matrix = MatrixInterface<Array<2, T>>;
}

// Additional definitions
using Mat2x2f = stack::Mat2x2<float>;
using Mat3x3i = stack::Mat3x3<int>;
using Mat3x3f = stack::Mat3x3<float>;
using Mat4x4f = stack::Mat4x4<float>;
using Mat3x3d = stack::Mat3x3<double>;
using Mat4x4d = stack::Mat4x4<double>;
template <typename T> using Mat = heap::Matrix<T>;

//! \brief Returns the type of the transpose of the matrix given as input.
template <typename T, Array::size_type M, Array::size_type N>
auto transpose_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, N, M>;
template <typename T> auto transpose_type(heap::Matrix<T>) -> heap::Matrix<T>;

//! \brief Returns the transpose of the matrix given as input.
template <typename Mat1, typename Mat2> auto transpose(const Mat1 &in, Mat2 &out) -> Mat2 & {
  out.resize({in.n(), in.m()});

  if (in.isRow() || in.isColumn() || in.isSymmetric()) {
    std::copy(in.begin(), in.end(), out.begin());
  } else {
    for (Array::size_type i = 0; i < out.m(); i++) {
      std::copy(in.col_begin(i), in.col_end(i), out.row_begin(i));
    }
  }

  return out;
}

template <typename Mat> decltype(transpose_type(Mat())) transpose(const Mat &m) {
  decltype(transpose_type(Mat())) out;
  return transpose(m, out);
}

//! \brief Computes and returns the adjoint of the matrix a.
template <typename Mat1, typename Mat2> auto adjoint(const Mat1 &in, Mat2 &out) -> Mat2 & {
  out.resize({in.n(), in.m()});

  if (in.isRow() || in.isColumn()) {
    std::transform(in.begin(), in.end(), out.begin(),
                   [](const typename Mat1::value_type &v) { return conjugate(v); });
  } else if (in.isHermitian()) {
    std::copy(in.begin(), in.end(), out.begin());
  } else {
    for (Array::size_type i = 0; i < out.m(); i++) {
      std::transform(in.col_begin(i), in.col_end(i), out.row_begin(i),
                     [](const typename Mat1::value_type &v) { return conjugate(v); });
    }
  }

  return out;
}

template <typename Mat> decltype(transpose_type(Mat())) adjoint(const Mat &m) {
  decltype(transpose_type(Mat())) out;
  return adjoint(m, out);
}

//! \brief Symmetrizes the matrix A by filling its lower (mode == 'L') or upper
//! (mode == 'U') part.
template <typename Mat> void symmetrize(Mat &A, char mode = 'L') {
  if (mode == 'L') {
    for (Array::size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::copy(ptr2, ptr2 + i, ptr1);
    }
  } else {
    for (Array::size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::copy(ptr1, ptr1 + i, ptr2);
    }
  }
}

//! \brief Hermitianizes the lower (mode == 'L') or upper (mode == 'U') part of
//! the matrix A.
template <typename Mat> void hermitianize(Mat &A, char mode = 'L') {
  if (mode == 'L') {
    for (Array::size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::transform(ptr2, ptr2 + i, ptr1,
                     [](const typename Mat::value_type &v) { return std::conj(v); });
    }
  } else {
    for (Array::size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::transform(ptr1, ptr1 + i, ptr2,
                     [](const typename Mat::value_type &v) { return std::conj(v); });
    }
  }
}

//! \brief Computes and returns the trace of the matrix a.
template <typename Mat> auto trace(const Mat &a) -> typename Mat::value_type {
  return std::accumulate(a.diag_begin(), a.diag_end(), typename Mat::value_type(0));
}

//! \brief Constructs a block matrix from the matrices given as input.
template <typename Mat1, typename Mat2>
auto block(std::initializer_list<std::initializer_list<Mat1>> L, Mat2 &out) -> Mat2 & {
  // Number of rows
  Array::size_type m0 = 0;
  for (auto iter = L.begin(); iter != L.end(); iter++) {
    m0 += iter->begin()->m();
  }

  // Number of columns
  Array::size_type n0 = 0;
  for (auto iter = L.begin()->begin(); iter != L.begin()->end(); iter++) {
    n0 += iter->n();
  }

  out.resize({m0, n0});

  // Building
  Array::size_type i0 = 0, j0 = 0;

  for (auto iter1 = L.begin(); iter1 != L.end(); iter1++) {
    j0 = 0;

    for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++) {
      for (Array::size_type i = 0; i < iter2->m(); i++) {
        std::copy(iter2->row_begin(i), iter2->row_end(i), out.row_begin(i0 + i) + j0);
      }

      j0 += iter2->n();
    }

    i0 += iter1->begin()->m();
  }

  return out;
}

template <typename Mat>
auto block(std::initializer_list<std::initializer_list<Mat>> L)
    -> heap::Matrix<typename Mat::value_type> {
  heap::Matrix<typename Mat::value_type> out;
  block(L, out);
  return out;
}

//! \brief Replicates and tiles matrix a according to the dimension vector dim.
template <typename Mat1, typename Mat2>
auto repmat(const std::array<Array::size_type, 2> &dim, const Mat1 &a, Mat2 &out) -> Mat2 & {
  out.resize({dim[0] * a.m(), dim[1] * a.n()});

  for (Array::size_type i = 0, i0 = 0; i < dim[0]; i++, i0 += a.m()) {
    for (Array::size_type j = 0, j0 = 0; j < dim[1]; j++, j0 += a.n()) {
      for (Array::size_type k = 0; k < a.m(); k++) {
        std::copy(a.row_begin(k), a.row_end(k), out.row_begin(i0 + k) + j0);
      }
    }
  }

  return out;
}

template <typename Mat>
auto repmat(const std::array<Array::size_type, 2> &dim, const Mat &a)
    -> heap::Matrix<typename Mat::value_type> {
  heap::Matrix<typename Mat::value_type> out;
  repmat(dim, a, out);
  return out;
}

} // namespace TMIV::Common

#endif
