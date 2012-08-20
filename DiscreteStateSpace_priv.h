#ifndef DISCRETESTATESPACE_PRIV_H
#define DISCRETESTATESPACE_PRIV_H

template <typename T, int p, int q, int n>
inline void DiscreteStateSpace<T, p, q, n>::Iterate()
{
  this->x = this->A*this->x + this->B*this->u;
}

#endif
