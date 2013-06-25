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
#include "constants.h"
#include "SystemState.h"

namespace hardware {

MPU6050::MPU6050()
  : i2c_{&I2CD2},
    i2cfg_{OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2},
    timeout_{MS2ST(2)},
    I2C_ADDR{0b1101000},
    ACCEL_XOUT_ADDR{59},
    initialized_{true}
{
  i2cObjectInit(i2c_);          // Initialize I2C driver
  initialized_ = initialize();  // Initialize and configure MPU6050
} // MPU6050())

MPU6050::~MPU6050()
{
  i2cStop(i2c_); // Deactivate I2CD2
}

bool MPU6050::acquire_data(Sample & s) const
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
} // acquire_data()

bool MPU6050::initialize()
{
  static Sample s;
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

  // Sleep 50 ms to allow PLL and gyro readings on MPU6050 to settle
  time = chTimeNow() + MS2ST(50);
  chThdSleepUntil(time);
  for (int i = 0; i < 10; ++i) {
    if (!acquire_data(s)) {
      return false;
    } else {
      time = chTimeNow() + MS2ST(5);
      chThdSleepUntil(time);
    }
  }

  return true;
} // Initialize()

bool MPU6050::checkTransmission(msg_t res, Sample & s)
{
  if (res != RDY_OK) {
    s.system_state |= (i2cGetErrors(&I2CD2) << 16);
    if (res == RDY_TIMEOUT)
      s.system_state |= systemstate::I2C_Software_Timeout;

    return false;
  }
  return true;
}

void MPU6050::convertData(Sample & s, int16_t ar[7])
{
  using namespace constants;
  s.has_mpu6050 = true;
  s.mpu6050.accelerometer_x = ar[0] * accelerometer_sensitivity;
  s.mpu6050.accelerometer_y = ar[1] * accelerometer_sensitivity;
  s.mpu6050.accelerometer_z = ar[2] * accelerometer_sensitivity;
  s.mpu6050.temperature = ar[3] * thermometer_sensitivity + thermometer_offset;
  s.mpu6050.gyroscope_x = ar[4] * gyroscope_sensitivity - gyro_x_bias;
  s.mpu6050.gyroscope_y = ar[5] * gyroscope_sensitivity - gyro_y_bias;
  s.mpu6050.gyroscope_z = ar[6] * gyroscope_sensitivity - gyro_z_bias;
}

float MPU6050::phi_dot(const Sample & s)
{
  return s.mpu6050.gyroscope_y;
}

float MPU6050::psi_dot(const Sample & s)
{
  return s.mpu6050.gyroscope_z;
}

} // namespace hardware
