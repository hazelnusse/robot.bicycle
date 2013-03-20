#ifndef SAMPLECONVERTED_H
#define SAMPLECONVERTED_H

#include <cstdint>

class __attribute__((__packed__)) SampleConverted {
 public:
  // Converted raw data
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

  // Generated data useful for debugging
  float theta_R_dot;    // check rear wheel rate estimate
  float phi_dot;        // check that imu_calibration is reasonable
  float x[5];           // check state update equations
  float steer_torque;   // calculated steer torque
  float steer_current;  // calculated steer current
};

#endif

