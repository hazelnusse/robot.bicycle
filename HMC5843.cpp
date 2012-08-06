#include <cstdint>

#include "ch.h"
#include "hal.h"

#include "HMC5843.h"
#include "HMC5843_reg.h"
#include "Sample.h"

void HMC5843Init()
{
  using namespace hmc5843;

  systime_t tmo = MS2ST(4);

  uint8_t mag_tx_data[4];

  mag_tx_data[0] = CRA;                     // Register to write to
  mag_tx_data[1] = FIFTY_HZ | NORMAL_MODE;  // 50 Hz
  mag_tx_data[2] = GAIN_1620;               // Gain
  mag_tx_data[3] = MODE_CONTINUOUS;         // Continuous conversion
  
  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, mag_tx_data, 4, NULL, 0, tmo);
  i2cReleaseBus(&I2CD1);
} // configureITG())

void HMC5843Acquire(Sample & s)
{
  using namespace hmc5843;

  systime_t tmo = MS2ST(2);

  uint8_t reg_address = DXRA;

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, &reg_address, 1,
      reinterpret_cast<uint8_t *>(s.mag), 6, tmo);
  i2cReleaseBus(&I2CD1);

  // Reverse bytes in each half word because HMC5843 storage is big-endian, so
  // bytes come out MSB then LSB, but STM32 is little endian, so we need the
  // LSB to go into the low byte of each half-word and the MSB into the high
  // byte of each half-word
  for (uint8_t i = 0; i < 3; ++i) {
    // This results in 3 instructions per loop iteration (for example):
    // ldrh r2, [r4, #0] // loads r2 with contents located at r4+0
    // rev16 r2, r2      // Reverse byte order in each halfword independently
    // strh r2, [r4, #0] // Store contents of r2 to r4+0
    asm("rev16 %0,%1" :  "=r" (s.mag[i]) : "r" (s.mag[i]));
  } // for i
}
