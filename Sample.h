#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>

class Sample {
 public:
  // IMU raw sensor data
  int16_t gyro[4], acc[3], mag[3];
  // Angular position encoder steer data
  uint32_t steerAngle;
  // speed estimates from timer capture compare registers
  uint32_t rearWheelRate, frontWheelRate, steerRate;
  // pwm outputs for rear wheel and steer
  uint16_t CCR_rw, CCR_steer;
  // Set point values for Speed and Steer rate
  float Speed_sp, YawRate_sp;
  // Time sample was taken and any errorcode data we might want to store
  uint32_t systemTime, errorCode;
};
#endif
