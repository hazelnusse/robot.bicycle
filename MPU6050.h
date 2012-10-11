#ifndef MPU6050_H
#define MPU6050_H

#include <cstdint>
#include "ch.h"

class Sample;

class MPU6050 {
 public:
  static MPU6050 & Instance(I2CDriver & i2c);
  void Acquire(Sample & s) const;
  void Initialize();
  void DeInitialize();

 private:
  MPU6050(I2CDriver & i2c);
  ~MPU6050();
  MPU6050 & operator=(const MPU6050 &) = delete;
  MPU6050(const MPU6050 &) = delete;

  static void * operator new(std::size_t, void * location);
  static MPU6050 * instance_;
  I2CDriver & i2c_;
  I2CConfig i2cfg_;
  systime_t timeout_;
  uint8_t I2C_ADDR;
  uint8_t ACCEL_XOUT_ADDR;

};
#endif
