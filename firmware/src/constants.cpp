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

const uint32_t T_ms = 5;
const uint32_t RW_N = 10;
const uint32_t YC_N = 1;
}

