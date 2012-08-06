#include <cstdint>

#include "ch.h"
#include "hal.h"

#include "ITG3200.h"
#include "ITG3200_reg.h"
#include "Sample.h"

void ITG3200Init()
{
  using namespace itg3200;

  systime_t tmo = MS2ST(4);

  uint8_t gyro_tx_data[4];

  gyro_tx_data[0] = SMPLRT_DIV;
  gyro_tx_data[1] = 0;
  gyro_tx_data[2] = DLPF_42;
  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, gyro_tx_data, 3, NULL, 0, tmo);
  i2cReleaseBus(&I2CD1);

  
  gyro_tx_data[0] = PWR_MGM;
  gyro_tx_data[1] = CLK_SEL_PLL_X_GYRO;
  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, gyro_tx_data, 2, NULL, 0, tmo);
  i2cReleaseBus(&I2CD1);
} // configureITG())


void ITG3200Acquire(Sample & s)
{
  using namespace itg3200;

  systime_t tmo = MS2ST(2);

  uint8_t reg_address = TEMP_OUT_H;

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, &reg_address, 1,
                           reinterpret_cast<uint8_t *>(s.gyro), 8, tmo);
  i2cReleaseBus(&I2CD1);

  // Reverse bytes in each half word because ITG3200 storage is big-endian, so
  // bytes come out MSB then LSB, but STM32 is little endian, so we need the
  // LSB to go into the low byte of each half-word and the MSB into the high
  // byte of each half-word
  for (uint8_t i = 0; i < 4; ++i) {
    // This results in 3 instructions per loop iteration (for example):
    // ldrh r2, [r4, #0] // loads r2 with contents located at r4+0
    // rev16 r2, r2      // Reverse byte order in each halfword independently
    // strh r2, [r4, #0] // Store contents of r2 to r4+0
    asm("rev16 %0,%1" :  "=r" (s.gyro[i]) : "r" (s.gyro[i]));
  } // for i

}
