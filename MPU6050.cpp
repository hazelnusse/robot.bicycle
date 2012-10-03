/*
 * =====================================================================================
 *
 *       Filename:  MPU-6050.cpp
 *
 *    Description:  MPU6050 class implementation
 *
 *        Created:  10/03/2012 09:39:44 AM
 *
 *         Author:  Dale Lukas Peterson, hazelnusse@gmail.com
 *
 * =====================================================================================
 */
#include "ch.h"
#include "hal.h"

#include "MPU6050.h"
#include "Sample.h"

MPU6050 * MPU6050::instance_ = 0;
const systime_t MPU6050::timeout = MS2ST(2);
const uint8_t MPU6050::I2C_ADDR = 0b1101000;
const uint8_t MPU6050::ACCEL_XOUT_ADDR = 59;

MPU6050::MPU6050()
{
  Configure();
} // MPU6050())

void MPU6050::Acquire(Sample & s) const
{
  i2cAcquireBus(&I2CD2);
  i2cMasterTransmitTimeout(&I2CD2, MPU6050::I2C_ADDR, &MPU6050::ACCEL_XOUT_ADDR, 1,
                           reinterpret_cast<uint8_t *>(s.MPU6050), 14, timeout);
  i2cReleaseBus(&I2CD2);

  // Reverse bytes in each 16-bit integer because MPU-6050 storage is
  // big-endian, so bytes come out MSB then LSB, but STM32 is little endian, so
  // we need the LSB to go into the low byte of each half-word and the MSB into
  // the high byte of each half-word
  for (uint8_t i = 0; i < 7; ++i) {
    // This results in 3 instructions per loop iteration (for example):
    // ldrh r2, [r4, #0] // loads r2 with contents located at r4+0
    // rev16 r2, r2      // Reverse byte order in each halfword independently
    // strh r2, [r4, #0] // Store contents of r2 to r4+0
    asm("rev16 %0,%1" :  "=r" (s.MPU6050[i]) : "r" (s.MPU6050[i]));
  } // for i
} // Acquire()

void MPU6050::Configure() const
{
  uint8_t tx_data[8];

  tx_data[0] = 15;   // Address of SMPLRT_DIV
  tx_data[1] = 4;    // SMPLRT_DIV register value, divides 32.768kHz clock
  tx_data[2] = 1;    // CONFIG register
  tx_data[3] = 0;    // GYRO_CONFIG register
  tx_data[4] = 0;    // ACCEL_CONFIG register
  i2cAcquireBus(&I2CD2);
  i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, tx_data, 5, NULL, 0, timeout);
  i2cReleaseBus(&I2CD2);


  tx_data[0] = 106;  // Address of USER_CTRL
  tx_data[1] = 1;    // reset GYRO, ACCEL, TEMP, and clear the sensor registers
  tx_data[2] = 4;    // PWR_MGMT_1 -- select PLL with external 32.768kHz ref
  i2cAcquireBus(&I2CD2);
  i2cMasterTransmitTimeout(&I2CD2, MPU6050::I2C_ADDR, tx_data,
                           3, NULL, 0, timeout);
  i2cReleaseBus(&I2CD2);
} // Configure()

void * MPU6050::operator new(std::size_t, void * location)
{
  return location;
} // operator new()

MPU6050 & MPU6050::Instance()
{
  static uint8_t allocation[sizeof(MPU6050)];

  if (instance_ == 0)
      instance_ = new (allocation) MPU6050;

  return * instance_;
} // Instance()
