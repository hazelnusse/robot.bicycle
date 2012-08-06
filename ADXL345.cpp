#include "ADXL345.h"
#include "ADXL345_reg.h"
#include "Sample.h"

#include <cstdint>

#include "ch.h"
#include "hal.h"

void ADXL345Init()
{
  using namespace adxl345;

  systime_t tmo = MS2ST(4);

  uint8_t acc_tx_data[4];

  acc_tx_data[0] = BW_RATE;              // Register to write to
  acc_tx_data[1] = Rate_200_HZ;          // 200 Hz
  acc_tx_data[2] = Measure;              // Measurement mode
  acc_tx_data[3] = 0x00;                 // INT_ENABLE:  No interrupts
  
  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, acc_tx_data, 4, NULL, 0, tmo);
  i2cReleaseBus(&I2CD1);

  acc_tx_data[0] = DATA_FORMAT;
  acc_tx_data[1] = FULL_RES | Range_2g;  // 

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, acc_tx_data, 2, NULL, 0, tmo);
  i2cReleaseBus(&I2CD1);
} // ADXL345Init()

void ADXL345Acquire(Sample & s)
{
  using namespace adxl345;

  systime_t tmo = MS2ST(2);

  uint8_t reg_address = DATAX0;

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ADDR, &reg_address, 1,
                           reinterpret_cast<uint8_t *>(s.acc), 6, tmo);
  i2cReleaseBus(&I2CD1);
}
