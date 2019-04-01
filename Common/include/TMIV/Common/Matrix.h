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

#ifndef _TMIV_COMMON_MATRIX_H_
#define _TMIV_COMMON_MATRIX_H_

#include "Array.h"
#include "Math.h"

namespace TMIV::Common {
template <typename A> class MatrixInterface : public A {
public:
  enum class Property { None, Symmetric, Hermitian, Positive, Lower, Upper };

  typedef typename A::size_type size_type;
  typedef typename A::const_dim_iterator const_row_iterator;
  typedef typename A::dim_iterator row_iterator;
  ;
  typedef typename A::const_dim_iterator const_column_iterator;
  typedef typename A::dim_iterator column_iterator;

public:
  using A::A;
  MatrixInterface() : A() {}
  MatrixInterface(const A &a) : A(a) {}
  MatrixInterface(A &&a) : A(std::move(a)) {}
  using A::operator=;
  MatrixInterface &operator=(const A &a) {
    A::operator=(a);
    return *this;
  }
  MatrixInterface &operator=(A &&a) {
    A::operator=(std::move(a));
    return *this;
  }
  //! \brief Returns the number of rows of the matrix.
  size_type m() const { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  size_type n() const { return A::size(1); }
  //! \brief Returns the number of rows of the matrix.
  size_type height() const { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  size_type width() const { return A::size(1); }
  //! \brief Returns the leading dimension of the matrix.
  size_type lda() const { return A::size(1); }
  //! \brief Overloaded resize operator.
  using A::resize;
  void resize(size_type a, size_type b) { A::resize({a, b}); }
  //! \brief Returns an iterator to the first element of the ith row.
  const_row_iterator row_begin(size_type i) const {
    return A::template dim_begin<1>(i);
  }
  row_iterator row_begin(size_type i) { return A::template dim_begin<1>(i); }
  //! \brief Returns a const iterator to the first element of the ith row.
  const_row_iterator crow_begin(size_type i) const {
    return A::template cdim_begin<1>(i);
  }
  //! \brief Returns an iterator to the first element after the end of the ith
  //! row.
  const_row_iterator row_end(size_type i) const {
    return A::template dim_end<1>(i);
  }
  row_iterator row_end(size_type i) { return A::template dim_end<1>(i); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! ith row.
  const_row_iterator crow_end(size_type i) const {
    return A::template cdim_end<1>(i);
  }
  //! \brief Returns an iterator to the first element of the jth column.
  const_column_iterator col_begin(size_type j) const {
    return A::template dim_begin<0>(j);
  }
  column_iterator col_begin(size_type j) { return A::template dim_begin<0>(j); }
  //! \brief Returns a const iterator to the first element of the jth column.
  const_column_iterator ccol_begin(size_type j) const {
    return A::template cdim_begin<0>(j);
  }
  //! \brief Returns an iterator to the first element after the end of the jth
  //! column.
  const_column_iterator col_end(size_type j) const {
    return A::template dim_end<0>(j);
  }
  column_iterator col_end(size_type j) { return A::template dim_end<0>(j); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! jth column.
  const_column_iterator ccol_end(size_type j) const {
    return A::template cdim_end<0>(j);
  }
  //! \brief Returns true if the matrix is a row.
  bool isRow() const { return (m() == 1); }
  //! \brief Returns true if the matrix is a column.
  bool isColumn() const { return (n() == 1); }
  //! \brief Returns true if the matrix is symmetric.
  bool isSymmetric() const {
    return (A::getProperty() == Property::Symmetric) ||
           (A::getProperty() == Property::Positive);
  }
  //! \brief Returns true if the matrix is hermitian.
  bool isHermitian() const { return (A::getProperty() == Property::Hermitian); }
  //! \brief Returns true if the matrix is positive.
  bool isPositive() const { return (A::getProperty() == Property::Positive); }
  //! \brief Returns true if the matrix is lower.
  bool isLower() const { return (A::getProperty() == Property::Lower); }
  //! \brief Returns true if the matrix is upper.
  bool isUpper() const { return (A::getProperty() == Property::Upper); }
  //! \brief Returns true if the matrix is triangular.
  bool isTriangular() const {
    return (A::getProperty() == Property::Lower) ||
           (A::getProperty() == Property::Upper);
  }
  static MatrixInterface diag(const std::vector<typename A::value_type> &v) {
    MatrixInterface out;

    out.resize(v.size(), v.size());
    std::fill(out.begin(), out.end(), 0);
    std::copy(v.begin(), v.end(), out.diag_begin());

    return out;
  }
};

namespace stack {
template <typename T, size_type M, size_type N>
using Matrix = MatrixInterface<Array<T, M, N>>;

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
using Mat3x3f = stack::Mat3x3<float>;
using Mat4x4f = stack::Mat4x4<float>;
using Mat3x3d = stack::Mat3x3<double>;
using Mat4x4d = stack::Mat4x4<double>;
template <typename T> using Mat = heap::Matrix<T>;

//! \brief Returns the type of the transpose of the matrix given as input.
template <typename T, Array::size_type M, Array::size_type N>
stack::Matrix<T, N, M> transpose_type(stack::Matrix<T, M, N>);
template <typename T> heap::Matrix<T> transpose_type(heap::Matrix<T>);

//! \brief Returns the transpose of the matrix given as input.
template <typename Mat1, typename Mat2>
Mat2 &transpose(const Mat1 &in, Mat2 &out) {
  typedef typename Array::size_type size_type;

  out.resize({in.n(), in.m()});

  if (in.isRow() || in.isColumn() || in.isSymmetric())
    std::copy(in.begin(), in.end(), out.begin());
  else
    for (size_type i = 0; i < out.m(); i++)
      std::copy(in.col_begin(i), in.col_end(i), out.row_begin(i));

  return out;
}

template <typename Mat>
decltype(transpose_type(Mat())) transpose(const Mat &m) {
  decltype(transpose_type(Mat())) out;
  return transpose(m, out);
}

//! \brief Computes and returns the adjoint of the matrix a.
template <typename Mat1, typename Mat2>
Mat2 &adjoint(const Mat1 &in, Mat2 &out) {
  typedef typename Array::size_type size_type;

  out.resize({in.n(), in.m()});

  if (in.isRow() || in.isColumn())
    std::transform(
        in.begin(), in.end(), out.begin(),
        [](const typename Mat1::value_type &v) { return conjugate(v); });
  else if (in.isHermitian())
    std::copy(in.begin(), in.end(), out.begin());
  else
    for (size_type i = 0; i < out.m(); i++)
      std::transform(
          in.col_begin(i), in.col_end(i), out.row_begin(i),
          [](const typename Mat1::value_type &v) { return conjugate(v); });

  return out;
}

template <typename Mat> decltype(transpose_type(Mat())) adjoint(const Mat &m) {
  decltype(transpose_type(Mat())) out;
  return adjoint(m, out);
}

//! \brief Symmetrizes the matrix A by filling its lower (mode == 'L') or upper
//! (mode == 'U') part.
template <typename Mat> void symmetrize(Mat &A, char mode = 'L') {
  typedef typename Array::size_type size_type;

  if (mode == 'L') {
    for (size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::copy(ptr2, ptr2 + i, ptr1);
    }
  } else {
    for (size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::copy(ptr1, ptr1 + i, ptr2);
    }
  }
}

//! \brief Hermitianizes the lower (mode == 'L') or upper (mode == 'U') part of
//! the matrix A.
template <typename Mat> void hermitianize(Mat &A, char mode = 'L') {
  typedef typename Array::size_type size_type;

  if (mode == 'L') {
    for (size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::transform(
          ptr2, ptr2 + i, ptr1,
          [](const typename Mat::value_type &v) { return std::conj(v); });
    }
  } else {
    for (size_type i = 1; i < A.m(); i++) {
      auto ptr1 = A.row_begin(i);
      auto ptr2 = A.col_begin(i);

      std::transform(
          ptr1, ptr1 + i, ptr2,
          [](const typename Mat::value_type &v) { return std::conj(v); });
    }
  }
}

//! \brief Computes and returns the trace of the matrix a.
template <typename Mat> typename Mat::value_type trace(const Mat &a) {
  return std::accumulate(a.diag_begin(), a.diag_end(),
                         typename Mat::value_type(0));
}

//! \brief Constructs a block matrix from the matrices given as input.
template <typename Mat1, typename Mat2>
Mat2 &block(std::initializer_list<std::initializer_list<Mat1>> L, Mat2 &out) {
  typedef typename Array::size_type size_type;

  // Number of rows
  size_type m0 = 0;
  for (auto iter = L.begin(); iter != L.end(); iter++)
    m0 += iter->begin()->m();

  // Number of columns
  size_type n0 = 0;
  for (auto iter = L.begin()->begin(); iter != L.begin()->end(); iter++)
    n0 += iter->n();

  out.resize({m0, n0});

  // Building
  size_type i0 = 0, j0;

  for (auto iter1 = L.begin(); iter1 != L.end(); iter1++) {
    j0 = 0;

    for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++) {
      for (size_type i = 0; i < iter2->m(); i++)
        std::copy(iter2->row_begin(i), iter2->row_end(i),
                  out.row_begin(i0 + i) + j0);

      j0 += iter2->n();
    }

    i0 += iter1->begin()->m();
  }

  return out;
}

template <typename Mat>
heap::Matrix<typename Mat::value_type>
block(std::initializer_list<std::initializer_list<Mat>> L) {
  heap::Matrix<typename Mat::value_type> out;
  block(L, out);
  return out;
}

//! \brief Replicates and tiles matrix a according to the dimension vector dim.
template <typename Mat1, typename Mat2>
Mat2 &repmat(const std::array<Array::size_type, 2> &dim, const Mat1 &a,
             Mat2 &out) {
  typedef typename Array::size_type size_type;

  out.resize({dim[0] * a.m(), dim[1] * a.n()});

  for (size_type i = 0, i0 = 0; i < dim[0]; i++, i0 += a.m()) {
    for (size_type j = 0, j0 = 0; j < dim[1]; j++, j0 += a.n()) {
      for (size_type k = 0; k < a.m(); k++)
        std::copy(a.row_begin(k), a.row_end(k), out.row_begin(i0 + k) + j0);
    }
  }

  return out;
}

template <typename Mat>
heap::Matrix<typename Mat::value_type>
repmat(const std::array<Array::size_type, 2> &dim, const Mat &a) {
  heap::Matrix<typename Mat::value_type> out;
  repmat(dim, a, out);
  return out;
}

} // namespace TMIV::Common

#endif
