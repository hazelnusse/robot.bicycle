#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H
class imu_calibration {
 public:
  static constexpr float wx = -0.12831133410801182f;
  static constexpr float wy = 0.032857218962598515f;
  static constexpr float wz = 0.010641128707006363f;
  static constexpr float dcm[6] = {-0.894519492243436f,
                                    -0.0635181679465503f,
                                    0.0608731305741522f,
                                    0.446568482938378f,
                                    0.997939373875708f,
                                    0.0202846750691697f};
};
#endif