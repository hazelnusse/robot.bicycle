#ifndef MPU6050_H
#define MPU6050_H

#include <cstdint>
#include "ch.h"

#include "Singleton.h"

class Sample;

class MPU6050 : public Singleton<MPU6050> {
  friend class Singleton<MPU6050>;
 public:
  bool Acquire(Sample & s) const;
  bool Initialize(I2CDriver * i2c);
  void DeInitialize();

 private:
  MPU6050();
  MPU6050(const MPU6050&) = delete;
  MPU6050 & operator=(const MPU6050&) = delete;

  static bool checkTransmission(msg_t res, Sample & s);

  I2CDriver * i2c_;
  I2CConfig i2cfg_;
  systime_t timeout_;
  uint8_t I2C_ADDR;
  uint8_t ACCEL_XOUT_ADDR;
};
#endif
