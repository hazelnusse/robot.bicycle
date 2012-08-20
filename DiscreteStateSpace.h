#ifndef DISCRETESTATESPACE_H
#define DISCRETESTATESPACE_H

#include "StateSpace.h"

template <typename T, int p, int q, int n>
class DiscreteStateSpace : public StateSpace<T, p, q, n> {
 public:
  void Iterate();
};

#include "DiscreteStateSpace_priv.h"

#endif
