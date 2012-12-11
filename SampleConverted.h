#ifndef SAMPLECONVERTED_H
#define SAMPLECONVERTED_H

#include <cstdint>

class __attribute__((__packed__)) SampleConverted {
 public:
  // Time sample was taken
  double Time;
  // Control loop sample and control time
  double ComputationTime;
  // IMU sensor data
  double Accelerometer[3], Temperature, Gyroscope[3];
  // Angular position encoder data
  double RearWheelAngle, SteerAngle, FrontWheelAngle;
  // Set point values for speed and yaw rate
  double RearWheelRate_sp, YawRate_sp;
  // Commanded current
  double I_rw, I_steer;
  // System state bits
  uint32_t SystemState;
};

#endif
