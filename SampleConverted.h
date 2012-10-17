#ifndef SAMPLECONVERTED_H
#define SAMPLECONVERTED_H

#include <cstdint>

class __attribute__((__packed__)) SampleConverted {
 public:
  // Time sample was taken
  double Time;
  // IMU sensor data
  double Accelerometer[3], Temperature, Gyroscope[3];
  // Angular position encoder data
  double RearWheelAngle, SteerAngle, FrontWheelAngle;
  // speed estimates from timer capture compare registers
  double RearWheelRate, SteerRate, FrontWheelRate;
  // Set point values for speed and yaw rate
  double RearWheelRate_sp, YawRate_sp;
  // Commanded current
  double I_rw, I_steer;
  // System state bits
  uint32_t SystemState;
};

#endif
