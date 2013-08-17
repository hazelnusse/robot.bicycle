#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <cstdint>
#include "matrix.h"

namespace constants {

constexpr float pi_over_four = std::atan(1.0f);
constexpr float pi_over_two = 2.0f * std::atan(1.0f);
constexpr float pi = 4.0f * std::atan(1.0f);
constexpr float three_pi_over_two = 6.0f * std::atan(1.0f);
constexpr float two_pi = 8.0f * std::atan(1.0f);
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

constexpr float thermometer_offset = 36.53f;
constexpr float thermometer_sensitivity = 1.0f / 340.0f;

// Accelerometer calibration results in
// a_calibrated = S_acc * a_measured + b_acc
// Accelerometer sensitivity matrix
constexpr control::Matrix<float, 3, 3> S_acc = {{
     5.98965439e-04f, -5.47664577e-07f,  1.92721134e-06f,
    -5.47664577e-07f,  5.95347246e-04f, -1.64554051e-06f,
     1.92721134e-06f, -1.64554051e-06f,  5.82882068e-04f}};

// Accelerometer bias vector
constexpr control::Matrix<float, 3, 1> b_acc = {{
    -0.56998783f,
     0.05138561f,
     1.16912488f}};

// Gyroscope sensitivity is assumed to be as published in MPU6050 datasheet
// this assumes all three axes have equal sensitivity and that there is no
// cross axis sensitivity
constexpr float S_gyro = rad_per_degree / 131.0f;

// Gyroscope biases
constexpr control::Matrix<float, 3, 1> b_gyro = {{
    0.12041638f,
   -0.03156526f,
   -0.00995525f}};

constexpr control::Matrix<float, 3, 3> dcm_sensor_to_lean = {{
    0.0034306349717862f, 0.999533673476719f,   0.0303424839099496f,
   -0.999847113313713f,  0.00290829696853493f, 0.0172421520401969f,
    0.017145866613409f, -0.0303969964779088f,  0.999390835390838f}};

// TODO: determine dcm relating sensor axes to bike fixed axes
//constexpr float dcm[6] = {-0.894519492243436f,
//                      -0.0635181679465503f,
//                      0.0608731305741522f,
//                      0.446568482938378f,
//                      0.997939373875708f,
//                      0.0202846750691697f};

constexpr int32_t fork_encoder_index_offset = -402;

constexpr uint8_t loop_period_ms = 5;
constexpr float loop_period_s = loop_period_ms * 0.001f;

constexpr uint16_t fork_counts_per_revolution = 20000;
constexpr uint16_t wheel_counts_per_revolution = 800;

} // namespace constants

#endif

