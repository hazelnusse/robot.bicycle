#ifndef ADXL345_REG_H
#define ADXL345_REG_H

#include <cstdint>

namespace adxl345 {

 const uint8_t ADDR = 0x53;   // 7-bit address when pin 12 is held at GND

// Register definitions                 (Table 19)
// All reset to 0x00 unless noted
  const uint8_t DEVID          = 0x00;  // Read Only     resets to 0b11100101
  const uint8_t THRESH_TAP     = 0x1D;  // Read / Write
  const uint8_t OFSX           = 0x1E;  // Read / Write
  const uint8_t OFSY           = 0x1F;  // Read / Write
  const uint8_t OFSZ           = 0x20;  // Read / Write
  const uint8_t DUR            = 0x21;  // Read / Write
  const uint8_t Latent         = 0x22;  // Read / Write
  const uint8_t Window         = 0x23;  // Read / Write
  const uint8_t THRESH_ACT     = 0x24;  // Read / Write
  const uint8_t THRES_INACT    = 0x25;  // Read / Write
  const uint8_t TIME_INACT     = 0x26;  // Read / Write
  const uint8_t ACT_INACT_CTL  = 0x27;  // Read / Write
  const uint8_t THRESH_FF      = 0x28;  // Read / Write
  const uint8_t TIME_FF        = 0x29;  // Read / Write
  const uint8_t TAP_AXES       = 0x2A;  // Read / Write
  const uint8_t ACT_TAP_STATUS = 0x2B;  // Read Only
  const uint8_t BW_RATE        = 0x2C;  // Read / Write  resets to 0b00001010
  const uint8_t POWER_CTL      = 0x2D;  // Read / Write
  const uint8_t INT_ENABLE     = 0x2E;  // Read / Write
  const uint8_t INT_MAP        = 0x2F;  // Read / Write
  const uint8_t INT_SOURCE     = 0x30;  // Read Only     resets to 0b00000010
  const uint8_t DATA_FORMAT    = 0x31;  // Read / Write
  const uint8_t DATAX0         = 0x32;  // Read Only
  const uint8_t DATAX1         = 0x33;  // Read Only
  const uint8_t DATAY0         = 0x34;  // Read Only
  const uint8_t DATAY1         = 0x35;  // Read Only
  const uint8_t DATAZ0         = 0x36;  // Read Only
  const uint8_t DATAZ1         = 0x37;  // Read Only
  const uint8_t FIFO_CTL       = 0x38;  // Read / Write
  const uint8_t FIFO_STATUS    = 0x39;  // Read Only

  // Bit field definitions for configurable registers

  // ACT/INACT Settings   (ACT_INACT_CTL)
  const uint8_t ACT_ac_dc      = 0x80;
  const uint8_t ACT_X_enable   = 0x40;
  const uint8_t ACT_Y_enable   = 0x20;
  const uint8_t ACT_Z_enable   = 0x10;
  const uint8_t INACT_ac_dc    = 0x08;
  const uint8_t INACT_X_enable = 0x04;
  const uint8_t INACT_Y_enable = 0x02;
  const uint8_t INACT_Z_enable = 0x01;

  // TAP Settings         (TAP_AXES)
  const uint8_t Suppress     = 0x08;
  const uint8_t TAP_X_enable = 0x04;
  const uint8_t TAP_Y_enable = 0x02;
  const uint8_t TAP_Z_enable = 0x01;

  // ACT/TAP Status       (ACT_TAP_STATUS)
  const uint8_t ACT_X_source = 0x40;
  const uint8_t ACT_Y_source = 0x20;
  const uint8_t ACT_Z_source = 0x10;
  const uint8_t Asleep       = 0x08;
  const uint8_t TAP_X_source = 0x04;
  const uint8_t TAP_Y_source = 0x02;
  const uint8_t TAP_Z_source = 0x01;

  // Power and Bandwidth  (BW_RATE)  (Table 7 and Table 8)
  const uint8_t Rate_3200_HZ = 0x0F;
  const uint8_t Rate_1600_HZ = 0x0E;
  const uint8_t Rate_800_HZ  = 0x0D;
  const uint8_t Rate_400_HZ  = 0x0C;
  const uint8_t Rate_200_HZ  = 0x0B;
  const uint8_t Rate_100_HZ  = 0x0A;
  const uint8_t Rate_50_HZ   = 0x09;
  const uint8_t Rate_25_HZ   = 0x08;
  const uint8_t Rate_12_5_HZ = 0x07;
  const uint8_t Rate_6_25_HZ = 0x06;
  const uint8_t Rate_3_13_HZ = 0x05;
  const uint8_t Rate_1_56_HZ = 0x04;
  const uint8_t Rate_0_78_HZ = 0x03;
  const uint8_t Rate_0_39_HZ = 0x02;
  const uint8_t Rate_0_20_HZ = 0x01;
  const uint8_t Rate_0_10_HZ = 0x00;
  // The following Rates can be used with LOW_POWER bit and realize power
  // savings.  However, low power mode has somewhat higher noise.
  const uint8_t LOW_POWER   = 0x10;
  const uint8_t Rate_400_HZ_LP  = Rate_400_HZ | LOW_POWER;
  const uint8_t Rate_200_HZ_LP  = Rate_200_HZ | LOW_POWER;
  const uint8_t Rate_100_HZ_LP  = Rate_100_HZ | LOW_POWER;
  const uint8_t Rate_50_HZ_LP   = Rate_50_HZ | LOW_POWER;
  const uint8_t Rate_25_HZ_LP   = Rate_25_HZ | LOW_POWER;
  const uint8_t Rate_12_5_HZ_LP = Rate_12_5_HZ | LOW_POWER;

  // Power control (POWER_CTL)
  const uint8_t Link        = 0x20;
  const uint8_t AUTO_SLEEP  = 0x10;
  const uint8_t Measure     = 0x08;
  const uint8_t Sleep       = 0x04;
  const uint8_t Wakeup_8_HZ = 0x00;
  const uint8_t Wakeup_4_HZ = 0x01;
  const uint8_t Wakeup_2_HZ = 0x02;
  const uint8_t Wakeup_1_HZ = 0x03;

  // Interrupt Enable, Map, and Source bits
  // (INT_ENABLE, INT_MAP, INT_SOURCE)
  const uint8_t DATA_READY = 0x80;
  const uint8_t SINGLE_TAP = 0x40;
  const uint8_t DOUBLE_TAP = 0x20;
  const uint8_t Activity   = 0x10;
  const uint8_t Inactivity = 0x08;
  const uint8_t FREE_FALL  = 0x04;
  const uint8_t Watermark  = 0x02;
  const uint8_t Overrun    = 0x01;

  // Data format bits (DATA_FORMAT)
  const uint8_t SELF_TEST  = 0x80;
  const uint8_t SPI        = 0x40;
  const uint8_t INT_INVERT = 0x20;
  const uint8_t FULL_RES   = 0x08;
  const uint8_t Justify    = 0x04;
  const uint8_t Range_2g   = 0x00;
  const uint8_t Range_4g   = 0x01;
  const uint8_t Range_8g   = 0x02;
  const uint8_t Range_16g  = 0x03;

  // FIFO_MODE, Trigger, and Samples (FIFO_CTL)
  const uint8_t FIFO_MODE_Bypass  = 0x00;
  const uint8_t FIFO_MODE_FIFO    = 0x40;
  const uint8_t FIFO_MODE_Stream  = 0x80;
  const uint8_t FIFO_MODE_Trigger = 0xC0;
  const uint8_t Trigger           = 0x20;

  // FIFO_TRIG, Entries (FIFO_STATUS)
  const uint8_t FIFO_TRIG = 0x80;
};  // namespace adxl345

#endif  // ADXL345_REG_H
