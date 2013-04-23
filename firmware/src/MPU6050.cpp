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
#include "Constants.h"
#include "SystemState.h"

MPU6050::MPU6050()
  : i2c_(0),
    i2cfg_({OPMODE_I2C, 100000, STD_DUTY_CYCLE}),
    timeout_(MS2ST(2)),
    I2C_ADDR(0b1101000),
    ACCEL_XOUT_ADDR(59)
{
  // I2C configuration
  i2cObjectInit(i2c_);    // Initialize I2CD2
} // MPU6050())

bool MPU6050::Acquire(Sample & s) const
{
  int16_t ar[7];
  msg_t res = i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, &ACCEL_XOUT_ADDR, 1,
                                       reinterpret_cast<uint8_t *>(ar),
                                       14, timeout_);

  if (!checkTransmission(res, s))
    return false;

  // Reverse bytes in each 16-bit integer because MPU-6050 storage is
  // big-endian, so bytes come out MSB then LSB, but STM32 is little endian, so
  // we need the LSB to go into the low byte of each half-word and the MSB into
  // the high byte of each half-word
  for (uint8_t i = 0; i < 7; ++i) {
    // This results in 3 instructions per loop iteration (for example):
    // ldrh r2, [r4, #0] // loads r2 with contents located at r4+0
    // rev16 r2, r2      // Reverse byte order in each halfword independently
    // strh r2, [r4, #0] // Store contents of r2 to r4+0
    asm("rev16 %0,%1" :  "=r" (ar[i]) : "r" (ar[i]));
  } // for i

  convertData(s, ar);

  return true;
} // Acquire()

bool MPU6050::Initialize(I2CDriver * i2c)
{
  static Sample s;
  i2c_ = i2c;
  i2cStart(i2c_, &i2cfg_); // Configure and activate I2CD2
  uint8_t tx_data[8];
  
  tx_data[0] = 107;   // Address of PWR_MGMT_1
  tx_data[1] = 1 << 7;// reset device
  msg_t res = i2cMasterTransmitTimeout(i2c_, I2C_ADDR, tx_data,
                                       2, NULL, 0, timeout_);

  if (!checkTransmission(res, s))
    return false;

  // Sleep 20 ms to allow PLL to settle
  systime_t time = chTimeNow() + MS2ST(20);
  chThdSleepUntil(time);

  tx_data[0] = 25;   // Address of SMPLRT_DIV
  tx_data[1] = 0;    // SMPLRT_DIV register value
  tx_data[2] = 0;    // CONFIG register value
  tx_data[3] = 0;    // GYRO_CONFIG register value
  tx_data[4] = 0;    // ACCEL_CONFIG register value
  res = i2cMasterTransmitTimeout(i2c_, I2C_ADDR, tx_data,
                                 5, NULL, 0, timeout_);

  if (!checkTransmission(res, s))
    return false;

  tx_data[0] = 106;  // Address of USER_CTRL
  tx_data[1] = 1;    // reset GYRO, ACCEL, TEMP, and clear the sensor registers
  tx_data[2] = 1;    // PWR_MGMT_1 -- select PLL with X axis gyroscope reference
  res = i2cMasterTransmitTimeout(i2c_, I2C_ADDR, tx_data,
                                 3, NULL, 0, timeout_);

  if (!checkTransmission(res, s))
    return false;

  // Sleep 40 ms to allow PLL and gyro readings on MPU6050 to settle
  time = chTimeNow() + MS2ST(50);
  chThdSleepUntil(time);
  for (int i = 0; i < 10; ++i) {
    if (!Acquire(s)) {
      return false;
    } else {
      time = chTimeNow() + MS2ST(5);
      chThdSleepUntil(time);
    }
  }

  return true;
} // Initialize()

void MPU6050::DeInitialize()
{
  i2cStop(i2c_); // Deactivate I2CD2
}

bool MPU6050::checkTransmission(msg_t res, Sample & s)
{
  if (res != RDY_OK) {
    s.SystemState |= (i2cGetErrors(&I2CD2) << 16);
    if (res == RDY_TIMEOUT)
      s.SystemState |= systemstate::I2C_Software_Timeout;

    return false;
  }
  return true;
}

void MPU6050::convertData(Sample & s, int16_t ar[7])
{
  s.has_mpu6050 = true;
  s.mpu6050.AccelerometerX = ar[0] * cf::Accelerometer_sensitivity;
  s.mpu6050.AccelerometerY = ar[1] * cf::Accelerometer_sensitivity;
  s.mpu6050.AccelerometerZ = ar[2] * cf::Accelerometer_sensitivity;
  s.mpu6050.Temperature = ar[3] * cf::Thermometer_sensitivity + cf::Thermometer_offset;
  s.mpu6050.GyroscopeX = ar[4] * cf::Gyroscope_sensitivity - MPU6050::gyro_x_bias;
  s.mpu6050.GyroscopeY = ar[5] * cf::Gyroscope_sensitivity - MPU6050::gyro_y_bias;
  s.mpu6050.GyroscopeZ = ar[6] * cf::Gyroscope_sensitivity - MPU6050::gyro_z_bias;
}

float MPU6050::phi_dot(const Sample & s)
{
  return s.mpu6050.GyroscopeY;
}

float MPU6050::psi_dot(const Sample & s)
{
  return s.mpu6050.GyroscopeZ;
}

