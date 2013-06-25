#ifndef MPU6050_H
#define MPU6050_H

#include <cstdint>
#include "ch.h"
#include "hal.h"

#include "sample.pb.h"

namespace hardware {

class MPU6050 {
 public:
  MPU6050();
  ~MPU6050();

  bool acquire_data(Sample & s) const;
  bool is_initialized() const { return initialized_; }

  static float phi_dot(const Sample & s);
  static float psi_dot(const Sample & s);
  static void convertData(Sample & s, int16_t ar[7]);

 private:
  bool initialize();

  static bool checkTransmission(msg_t res, Sample & s);

  I2CDriver * i2c_;
  I2CConfig i2cfg_;
  systime_t timeout_;
  uint8_t I2C_ADDR;
  uint8_t ACCEL_XOUT_ADDR;
  bool initialized_;
};

} // namespace hardware

#endif

