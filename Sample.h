#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>

class Sample {
 public :
  int16_t gyro[4], acc[3], mag[3];
  int32_t steerAngle, steerRate, rearWheelRate, frontWheelRate;
  uint32_t systemTime, errorCode;
};
#endif
