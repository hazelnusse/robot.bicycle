#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

template <class T>
class constants {
 public:
  // Standard engineering constants
  static constexpr T pi = 4.0*std::atan(1.0);
  static constexpr T pi_2 = 2.0*std::atan(1.0);
  static constexpr T pi_4 = std::atan(1.0);
  static constexpr T rad_per_degree = pi / 180.0;
  static constexpr T e = std::exp(1.0);
  static constexpr T g = 9.81;

  // Gyroscope constants
  static constexpr T Gyroscope_temp_offset = 13200;
  static constexpr T Gyroscope_temp_sensitivity = 1.0 / 280.0;
  static constexpr T Gyroscope_sensitivity = rad_per_degree / 14.375;

  // Accelerometer constants
  static constexpr T Accelerometer_sensitivity = 1.0 / (g * 256.0);

  // Magnetometer constants
  static constexpr T Magnetometer_sensitivity = 1.0 / 1620.0;

  // Steer angle
  static constexpr T Steer_CPR = 5000.0;
  static constexpr T Steer_rad_per_count = 2.0 * pi / Steer_CPR;
  static constexpr T Steer_rad_per_quad_count = Steer_rad_per_count / 4.0;

  // Wheel rates and steer rate
  static constexpr T Wheel_CPR = 200;
  static constexpr T Wheel_rad_per_count = 2.0 * pi / Wheel_CPR;

  static constexpr T Rate_Timer_Frequency = 0.5e6;
  static constexpr T Wheel_rad_counts_per_sec = 2.0 * pi * Rate_Timer_Frequency
                                                / Wheel_CPR;
  static constexpr T Steer_rad_counts_per_sec = 2.0 * pi * Rate_Timer_Frequency
                                                / Steer_CPR;

  // Current command constants
  static constexpr T Current_max_rw = 6.0;
  static constexpr T Current_max_steer = 6.0;

};

typedef constants<float> cf;
typedef constants<double> cd;
typedef constants<long double> cld;

#endif
