#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H
class imu_calibration {
 public:
  static constexpr float wx = -0.12719738599803265f;
  static constexpr float wy = 0.03204398619466895f;
  static constexpr float wz = 0.010330667545989249f;
  static constexpr float dcm[6] = {-0.729457097767529f,
                                    -0.0908290542685650f,
                                    0.124515964744936f,
                                    0.684026565651199f,
                                    0.992217604421348f,
                                    0.0f};
};
#endif
