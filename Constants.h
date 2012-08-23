#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

template <class T>
class constants {
 public:
  static constexpr T pi = 4.0*std::atan(1.0);
  static constexpr T pi_2 = 2.0*std::atan(1.0);
  static constexpr T pi_4 = std::atan(1.0);
  static constexpr T rad_to_degree = pi / 180.0;
  static constexpr T e = std::exp(1.0);
  static constexpr T g = 9.81;
};

typedef constants<float> constantsf;
typedef constants<double> constantsd;
typedef constants<long double> constantsld;

#endif
