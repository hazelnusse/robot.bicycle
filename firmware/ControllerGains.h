#ifndef CGAINS_H
#define CGAINS_H
#include <array>
#include <cstdint>

namespace cg {

const uint32_t num_gains = 101;
const int32_t a_rows = 4;
const int32_t a_cols = 4;
const int32_t b_rows = 4;
const int32_t b_cols = 3;
const int32_t c_rows = 1;
const int32_t c_cols = 4;

struct ControllerGains {
  float A[a_rows * a_cols];
  float B[b_rows * b_cols];
  float C[c_rows * c_cols];
  float theta_R_dot;
  bool operator<(const ControllerGains & rhs) const {
    return theta_R_dot < rhs.theta_R_dot;
  }
};

} /* namespace cg */

#endif /* CGAINS_H */
