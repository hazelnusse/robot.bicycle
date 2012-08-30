#ifndef SAMPLECONVERTED_H
#define SAMPLECONVERTED_H

#include <cstdint>

class SampleConverted {
 public:
  // IMU sensor data
  double Temperature, Gyroscope[3], Accelerometer[3], Magnetometer[3];
  // Angular position
  double SteerAngle;
  // speed estimates
  double RearWheelRate, FrontWheelRate, SteerRate;
  // Commanded current
  double I_rw, I_steer;
  // Set point values for speed and yaw rate
  double RearWheelRate_sp, YawRate_sp;
  // Time sample was taken
  double Time;
  // System state bits
  uint32_t SystemState;
};

#endif
