#include "imu_calibration.h"
#include "Constants.h"

const float imu_calibration::gyro_x_bias = -0.12831133410801182f;
const float imu_calibration::gyro_y_bias = 0.032857218962598515f;
const float imu_calibration::gyro_z_bias = 0.010641128707006363f;
const float imu_calibration::dcm[6] = {-0.894519492243436f,
                                 -0.0635181679465503f,
                                  0.0608731305741522f,
                                  0.446568482938378f,
                                  0.997939373875708f,
                                  0.0202846750691697f};

float imu_calibration::phi_dot(const Sample & s)
{
  const float wx = s.MPU6050[4]*cf::Gyroscope_sensitivity - gyro_x_bias;
  const float wy = s.MPU6050[5]*cf::Gyroscope_sensitivity - gyro_y_bias;
  const float wz = s.MPU6050[6]*cf::Gyroscope_sensitivity - gyro_z_bias;
  return dcm[0] * wx + dcm[3] * wy + dcm[5] * wz;
}

