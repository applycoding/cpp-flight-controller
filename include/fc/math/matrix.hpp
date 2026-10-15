#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "fc/types.hpp"

namespace fc {

template <int Rows, int Cols>
class Matrix {
 public:
  static Matrix zero() { return {}; }

  static Matrix identity() {
    static_assert(Rows == Cols, "identity requires a square matrix");
    Matrix m;
    for (int i = 0; i < Rows; ++i) {
      m(i, i) = 1.0;
    }
    return m;
  }

  double operator()(int r, int c) const { return data_[static_cast<std::size_t>(r * Cols + c)]; }
  double& operator()(int r, int c) { return data_[static_cast<std::size_t>(r * Cols + c)]; }

  Matrix operator+(const Matrix& rhs) const {
    Matrix out;
    for (std::size_t i = 0; i < data_.size(); ++i) {
      out.data_[i] = data_[i] + rhs.data_[i];
    }
    return out;
  }

  Matrix operator-(const Matrix& rhs) const {
    Matrix out;
    for (std::size_t i = 0; i < data_.size(); ++i) {
      out.data_[i] = data_[i] - rhs.data_[i];
    }
    return out;
  }

  Matrix operator*(double s) const {
    Matrix out;
    for (std::size_t i = 0; i < data_.size(); ++i) {
      out.data_[i] = data_[i] * s;
    }
    return out;
  }

  template <int Inner>
  Matrix<Rows, Inner> times(const Matrix<Cols, Inner>& rhs) const {
    Matrix<Rows, Inner> out = Matrix<Rows, Inner>::zero();
    for (int r = 0; r < Rows; ++r) {
      for (int c = 0; c < Inner; ++c) {
        double sum = 0.0;
        for (int k = 0; k < Cols; ++k) {
          sum += (*this)(r, k) * rhs(k, c);
        }
        out(r, c) = sum;
      }
    }
    return out;
  }

  Matrix<Cols, Rows> transpose() const {
    Matrix<Cols, Rows> out = Matrix<Cols, Rows>::zero();
    for (int r = 0; r < Rows; ++r) {
      for (int c = 0; c < Cols; ++c) {
        out(c, r) = (*this)(r, c);
      }
    }
    return out;
  }

  void symmetrize() {
    static_assert(Rows == Cols, "symmetrize requires a square matrix");
    for (int r = 0; r < Rows; ++r) {
      for (int c = r + 1; c < Cols; ++c) {
        const double v = 0.5 * ((*this)(r, c) + (*this)(c, r));
        (*this)(r, c) = v;
        (*this)(c, r) = v;
      }
    }
  }

 private:
  std::array<double, static_cast<std::size_t>(Rows* Cols)> data_{};
};

inline Matrix<3, 3> skew(Vec3 v) {
  Matrix<3, 3> m = Matrix<3, 3>::zero();
  m(0, 1) = -v.z;
  m(0, 2) = v.y;
  m(1, 0) = v.z;
  m(1, 2) = -v.x;
  m(2, 0) = -v.y;
  m(2, 1) = v.x;
  return m;
}

inline bool invert3(const Matrix<3, 3>& m, Matrix<3, 3>& inv, double eps = 1.0e-12) {
  const double a00 = m(0, 0);
  const double a01 = m(0, 1);
  const double a02 = m(0, 2);
  const double a10 = m(1, 0);
  const double a11 = m(1, 1);
  const double a12 = m(1, 2);
  const double a20 = m(2, 0);
  const double a21 = m(2, 1);
  const double a22 = m(2, 2);

  const double c00 = a11 * a22 - a12 * a21;
  const double c01 = a12 * a20 - a10 * a22;
  const double c02 = a10 * a21 - a11 * a20;
  const double det = a00 * c00 + a01 * c01 + a02 * c02;
  if (std::abs(det) < eps) {
    return false;
  }
  const double d = 1.0 / det;
  const double c10 = a02 * a21 - a01 * a22;
  const double c11 = a00 * a22 - a02 * a20;
  const double c12 = a01 * a20 - a00 * a21;
  const double c20 = a01 * a12 - a02 * a11;
  const double c21 = a02 * a10 - a00 * a12;
  const double c22 = a00 * a11 - a01 * a10;

  inv(0, 0) = c00 * d;
  inv(0, 1) = c10 * d;
  inv(0, 2) = c20 * d;
  inv(1, 0) = c01 * d;
  inv(1, 1) = c11 * d;
  inv(1, 2) = c21 * d;
  inv(2, 0) = c02 * d;
  inv(2, 1) = c12 * d;
  inv(2, 2) = c22 * d;
  return true;
}

}  // namespace fc
