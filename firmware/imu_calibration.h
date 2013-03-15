#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "Sample.h"

class imu_calibration {
 public:
  static float phi_dot(const Sample & s);
 private:
  static const float gyro_x_bias;
  static const float gyro_y_bias;
  static const float gyro_z_bias;
  static const float dcm[6];
};

#endif

