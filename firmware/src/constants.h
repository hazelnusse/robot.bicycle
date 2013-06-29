#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <cstdint>

namespace constants {

constexpr float pi = 4.0f * std::atan(1.0f);
constexpr float pi_2 = 2.0f * std::atan(1.0f);
constexpr float pi_4 = std::atan(1.0f);
constexpr float two_pi = 2.0f * pi;
constexpr float rad_per_degree = pi / 180.0f;
constexpr float degree_per_rad = 180.0f / pi;
constexpr float e = std::exp(1.0f);
constexpr float g = 9.81f;

constexpr uint32_t PWM_ARR = 0xFFFE;
constexpr uint32_t ENC_ARR = 0xFFFF;

constexpr float system_timer_counts_per_second = 4.0e6f;
constexpr float system_timer_seconds_per_count = 1.0f / system_timer_counts_per_second;

constexpr float Nm_per_ozfin = 0.00706155182175f;

constexpr float wheel_radius = 0.3359f;

// Accelerometer constexprants
// gyro_scale calculated experimentally by comparing integration of lean rate
// with calculation of lean using acceleration for the system in a
// quasi-static setting.
constexpr float gyro_scale = 0.866634f;
constexpr float accelerometer_sensitivity = g / 16384.0f;
constexpr float gyroscope_sensitivity = rad_per_degree / 131.0f * gyro_scale;
constexpr float thermometer_offset = 36.53f;
constexpr float thermometer_sensitivity = 1.0f / 340.0f;

constexpr float acc_x_bias = 0.219491423264f;  // Obtained from static hanging configuration
constexpr float acc_y_bias = 0.0f;             // TODO: calibrate?
constexpr float acc_z_bias = 0.0f;             // TODO: calibrate?
constexpr float gyro_x_bias = -0.12831133410801182f;
constexpr float gyro_y_bias = 0.032857218962598515f;
constexpr float gyro_z_bias = 0.010641128707006363f;
constexpr float dcm[6] = {-0.894519492243436f,
                      -0.0635181679465503f,
                      0.0608731305741522f,
                      0.446568482938378f,
                      0.997939373875708f,
                      0.0202846750691697f};

constexpr int32_t fork_encoder_index_offset = -402;

constexpr uint8_t loop_period_ms = 5;
constexpr float loop_period_s = loop_period_ms * 0.001f;

constexpr uint16_t fork_counts_per_revolution = 20000;
constexpr uint16_t wheel_counts_per_revolution = 800;

} // namespace constants

#endif

