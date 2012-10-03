#ifndef MPU6050_H
#define MPU6050_H

#include <cstdint>
#include "ch.h"

class Sample;

class MPU6050 {
 public:
  static MPU6050 & Instance();
  void Acquire(Sample & s) const;
  void Configure() const;

 private:
  MPU6050();
  ~MPU6050();
  MPU6050 & operator=(const MPU6050 &) = delete;
  MPU6050(const MPU6050 &) = delete;

  static void * operator new(std::size_t, void * location);
  static MPU6050 * instance_;
  static const systime_t timeout;
  static const uint8_t I2C_ADDR;
  static const uint8_t ACCEL_XOUT_ADDR;
};
#endif
