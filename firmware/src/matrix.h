#ifndef MATRIX_H
#define MATRIX_H

namespace control {

template <typename T, int M, int N>
struct Matrix {
  T * begin() { return &data[0]; }
  T * end() { return &data[M * N]; }
  const T * begin() const { return &data[0]; }
  const T * end()  const{ return &data[M * N]; }
  const T & operator()(int i, int j) const { return data[N * i + j]; }
  T & operator()(int i, int j) { return data[N * i + j]; }

  Matrix<T, M, N> & operator+=(const Matrix<T, M, N> & B);
  Matrix<T, M, N> & operator-=(const Matrix<T, M, N> & B);

  T data[M * N];
};

template <typename T, int M, int N, int O>
Matrix<T, M, N> operator*(const Matrix<T, M, O> & A, const Matrix<T, O, N> & B)
{
  Matrix<T, M, N> result{{}};     // entries are zero-initialized

  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j)
      for (int k = 0; k < O; ++k)
        result(i, j) += A(i, k) * B(k, j);

  return result;
}

template <typename T, int M, int N>
Matrix<T, M, N> operator*(T a, const Matrix<T, M, N> & B)
{
  Matrix<T, M, N> result{{}};     // entries are zero-initialized

  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j)
      result(i, j) = a * B(i, j);

  return result;
}

template <typename T, int M, int N, int O>
Matrix<T, M, N> operator*(const Matrix<T, O, N> & B, T a)
{
  return (a * B);
}

template <typename T, int M, int N>
Matrix<T, M, N> operator+(const Matrix<T, M, N> & A, const Matrix<T, M, N> & B)
{
  Matrix<T, M, N> result;       // entries are uninitialized

  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j)
      result(i, j) = A(i, j) + B(i, j);

  return result;
}

template <typename T, int M, int N>
Matrix<T, M, N> operator-(const Matrix<T, M, N> & A, const Matrix<T, M, N> & B)
{
  return A + (T(-1) * B);
}

template <typename T, int M, int N>
Matrix<T, M, N> & Matrix<T, M, N>::operator+=(const Matrix<T, M, N> & B)
{
  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j)
      (*this)(i, j) += B(i, j);

  return *this;
}

template <typename T, int M, int N>
Matrix<T, M, N> & Matrix<T, M, N>::operator-=(const Matrix<T, M, N> & B)
{
  return this->operator+=(T(-1) * B);
}

}

#endif

