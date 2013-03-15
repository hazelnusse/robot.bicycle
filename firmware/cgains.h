#ifndef CGAINS_H
#define CGAINS_H
#include <array>
#include <cstdint>

namespace cg {

const uint32_t num_gains = 101;
const int32_t a_rows = 5;
const int32_t a_cols = 5;
const int32_t b_rows = 5;
const int32_t b_cols = 3;
const int32_t c_rows = 1;
const int32_t c_cols = 5;

struct ControllerGains {
  float A[a_rows * a_cols];
  float B[b_rows * b_cols];
  float C[c_rows * c_cols];
  float theta_R_dot;
  bool operator<(const ControllerGains & rhs) const {
    return theta_R_dot < rhs.theta_R_dot;
  }
};

/** State and output update.
 *
 * Performs a lookup on theta_R_dot to determine if it is within range of rear
 * wheel rates for which gains have been calculated.  If it is, the A, B, and C
 * matrices of nearest two rates, are used to update the state and output using
 * a linear interpolation.
 *
 * \param[in] theta_R_dot Rear wheel rate relative to rear frame.
 *
 * \param[in] input yaw rate reference, steer angle measurement, roll rate
 *             measurement
 *
 * \param[in,out] x State of system.
 *
 * \param[out] y Output of system.
 *
 * \returns true if theta_R_dot is inside range of speeds, false otherwise. 
 *
 * \post If theta_R_dot is inside range of speeds, state and output of system
 * are update, otherwise they remain unchange.
 */
bool state_and_output_update(float theta_R_dot, const float input[3], float x[5], float & y);

} /* namespace cg */

#endif /* CGAINS_H */
