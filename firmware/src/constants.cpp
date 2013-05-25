#include <cmath>
#include "constants.h"

namespace constants {

const float pi = 4.0f * std::atan(1.0f);
const float pi_2 = 2.0f * std::atan(1.0f);
const float pi_4 = std::atan(1.0f);
const float two_pi = 2.0f * pi;
const float rad_per_degree = pi / 180.0f;
const float degree_per_rad = 180.0f / pi;
const float e = std::exp(1.0f);
const float g = 9.81f;

const uint32_t PWM_ARR = 0xFFFE;
const uint32_t ENC_ARR = 0xFFFF;

// const uint8_t loop_period_ms = 5;
const float system_timer_frequency = 4.0e6f;
const float system_timer_period = 1.0f / system_timer_frequency;
const uint32_t YC_N = 1;

const float Nm_per_ozfin = 0.00706155182175f;

const float wheel_radius = 0.3359f;

// Accelerometer constants
const float accelerometer_sensitivity = g / 16384.0f;
const float gyroscope_sensitivity = rad_per_degree / 131.0f;
const float thermometer_offset = 36.53f;
const float thermometer_sensitivity = 1.0f / 340.0f;

const float gyro_x_bias = -0.12831133410801182f;
const float gyro_y_bias = 0.032857218962598515f;
const float gyro_z_bias = 0.010641128707006363f;
const float dcm[6] = {-0.894519492243436f,
                      -0.0635181679465503f,
                      0.0608731305741522f,
                      0.446568482938378f,
                      0.997939373875708f,
                      0.0202846750691697f};

}

