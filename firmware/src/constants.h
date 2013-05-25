#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cstdint>

namespace constants {

extern const float pi;
extern const float pi_2;
extern const float pi_4;
extern const float two_pi;
extern const float rad_per_degree;
extern const float degree_per_rad;
extern const float e;
extern const float g;

extern const uint32_t PWM_ARR;
extern const uint32_t ENC_ARR;

constexpr uint8_t loop_period_ms = 5;  // main loop update rate in ms
extern const float system_timer_frequency;
extern const float system_timer_period;
// TODO: Move to fork motor controller
extern const uint32_t YC_N;  // yaw rate control update prescalar

extern const float Nm_per_ozfin;

extern const float wheel_radius;

extern const float accelerometer_sensitivity;
extern const float gyroscope_sensitivity;
extern const float thermometer_offset;
extern const float thermometer_sensitivity;

extern const float gyro_x_bias;
extern const float gyro_y_bias;
extern const float gyro_z_bias;
extern const float dcm[6];

// Unity gain low pass filter
constexpr float lpf_pole = -100.0f;  // 1 / (s/-100 + 1)

} // namespace constants

#endif

