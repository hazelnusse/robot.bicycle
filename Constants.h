#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

template <class T>
class constants {
 public:
  static constexpr T pi = 4.0*std::atan(1.0);
  static constexpr T pi_2 = 2.0*std::atan(1.0);
  static constexpr T pi_4 = std::atan(1.0);
  static constexpr T e = std::exp(1.0);
};

#endif
