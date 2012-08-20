#ifndef STATESPACE_H
#define STATESPACE_H

#include <Eigen/Dense>

template <typename T, int p, int q, int n>
class StateSpace {
 public:
  Eigen::Matrix<T, q, 1> GetOutput() const;

  Eigen::Matrix<T, n, n> A;
  Eigen::Matrix<T, n, p> B;
  Eigen::Matrix<T, q, n> C;
  Eigen::Matrix<T, q, p> D;
  Eigen::Matrix<T, n, 1> x;
  Eigen::Matrix<T, p, 1> u;
};

#include "StateSpace_priv.h"

#endif
