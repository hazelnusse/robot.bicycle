#ifndef STATESPACE_PRIV_H
#define STATESPACE_PRIV_H

template <typename T, int p, int q, int n>
inline Eigen::Matrix<T, q, 1> StateSpace<T, p, q, n>::GetOutput() const
{
  return this->C*this->x + this->D*this->u;
}

#endif
