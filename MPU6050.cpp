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

MPU6050::MPU6050(I2CDriver & i2c)
  : i2c_(i2c), timeout_(MS2ST(2)), I2C_ADDR(0b1101000), ACCEL_XOUT_ADDR(59)
{
  // I2C onfiguration
  i2cfg_ = { OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2 };
  i2cObjectInit(&i2c_);    // Initialize I2CD2
} // MPU6050())

void MPU6050::Acquire(Sample & s) const
{
  i2cAcquireBus(&I2CD2);
  i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, &ACCEL_XOUT_ADDR, 1,
                           reinterpret_cast<uint8_t *>(s.MPU6050), 14, timeout_);
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

void MPU6050::Initialize()
{
  i2cStart(&i2c_, &i2cfg_); // Configure and activate I2CD2
  uint8_t tx_data[8];
  
  tx_data[0] = 107;   // Address of PWR_MGMT_1
  tx_data[1] = 1 << 7;// reset device
  i2cAcquireBus(&i2c_);
  i2cMasterTransmitTimeout(&i2c_, I2C_ADDR, tx_data,
                           2, NULL, 0, timeout_);
  i2cReleaseBus(&i2c_);

  // Sleep 10 ms to allow PLL to settle
  systime_t time = chTimeNow() + MS2ST(10);
  chThdSleepUntil(time);

  tx_data[0] = 25;   // Address of SMPLRT_DIV
  tx_data[1] = 0;    // SMPLRT_DIV register value
  tx_data[2] = 0;    // CONFIG register value
  tx_data[3] = 0;    // GYRO_CONFIG register value
  tx_data[4] = 0;    // ACCEL_CONFIG register value
  i2cAcquireBus(&i2c_);
  i2cMasterTransmitTimeout(&i2c_, I2C_ADDR, tx_data,
                           5, NULL, 0, timeout_);
  i2cReleaseBus(&i2c_);

  tx_data[0] = 106;  // Address of USER_CTRL
  tx_data[1] = 1;    // reset GYRO, ACCEL, TEMP, and clear the sensor registers
  tx_data[2] = 1;    // PWR_MGMT_1 -- select PLL with X axis gyroscope reference
  i2cAcquireBus(&i2c_);
  i2cMasterTransmitTimeout(&i2c_, I2C_ADDR, tx_data,
                           3, NULL, 0, timeout_);
  i2cReleaseBus(&i2c_);

  // Sleep 10 ms to allow PLL on MPU6050 to settle
  time = chTimeNow() + MS2ST(10);
  chThdSleepUntil(time);
} // Initialize()

void MPU6050::DeInitialize()
{
  i2cStop(&i2c_); // Configure and activate I2CD2
}

void * MPU6050::operator new(std::size_t, void * location)
{
  return location;
} // operator new()

MPU6050 & MPU6050::Instance(I2CDriver & i2c)
{
  static uint8_t allocation[sizeof(MPU6050)];

  if (instance_ == 0)
      instance_ = new (allocation) MPU6050(i2c);

  return * instance_;
} // Instance()
