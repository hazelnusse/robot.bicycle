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

extern const uint32_t T_ms;  // main loop update rate in ms
extern const uint32_t RW_N;  // rear wheel control update prescalar
extern const uint32_t YC_N;  // yaw rate control update prescalar

const float Nm_per_ozfin;

} // namespace constants

#endif

