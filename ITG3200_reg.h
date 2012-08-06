#ifndef ITG3200_REG_H
#define ITG3200_REG_H
#include <cstdint>

namespace itg3200 {

// I2C Slave Address Depends on state of AD0, pin 9
  const uint8_t ADDR = 0b1101000;  // 7-bit

  // Register definitions, all reset to 0 unless noted
  const uint8_t WHO_AM_I    = 0x00;  // Read / Write, resets to 0b-110100-
  const uint8_t SMPLRT_DIV  = 0x15;  // Read / Write
  const uint8_t DLPF_FS     = 0x16;  // Read / Write
  const uint8_t INT_CFG     = 0x17;  // Read / Write
  const uint8_t INT_STATUS  = 0x1A;  // Read / Write
  const uint8_t TEMP_OUT_H  = 0x1B;  // Read
  const uint8_t TEMP_OUT_L  = 0x1C;  // Read
  const uint8_t GYRO_XOUT_H = 0x1D;  // Read
  const uint8_t GYRO_XOUT_L = 0x1E;  // Read
  const uint8_t GYRO_YOUT_H = 0x1F;  // Read
  const uint8_t GYRO_YOUT_L = 0x20;  // Read
  const uint8_t GYRO_ZOUT_H = 0x21;  // Read
  const uint8_t GYRO_ZOUT_L = 0x22;  // Read
  const uint8_t PWR_MGM     = 0x3E;  // Read / Write

  // Bit field definitions for configurable registers

  // DLPF, Full Scale         (ITG_DLPF_FS)
  // FS_SEL (bits 4:3) must be 0b11 for proper operation, so don't define any bits, and
  // automatically set these bits when we define all the DLPF_CFG (bits 0:2) options
  const uint8_t DLPF_256 = 0x18;
  const uint8_t DLPF_188 = 0x19;
  const uint8_t DLPF_98  = 0x1A;
  const uint8_t DLPF_42  = 0x1B;
  const uint8_t DLPF_20  = 0x1C;
  const uint8_t DLPF_10  = 0x1D;
  const uint8_t DLPF_5   = 0x1E;

  // Interrupt Configuration (ITG_INT_CFG)
  const uint8_t RAW_RDY_EN = 0x01;  // Enable interrupt when data is available
  const uint8_t ITG_RDY_EN = 0x04;  // Enable interrupt when device is ready (PLL ready after changing clock source)
  const uint8_t INT_ANYRD_2CLEAR = 0x10;  // latch clear method, 1=any register read, 0=status register read only
  const uint8_t LATCH_INT_EN = 0x20;  // latch mode, 1=latch until interrupt is cleared, 0=50us pulse
  const uint8_t OPEN = 0x40;  // drive type for INT output pin, 1=open drain, 0=push-pull
  const uint8_t ACTL = 0x80;  // logic level for INT output pin, 1=active Low, 0=active high

// Interrupt Status
  const uint8_t ITG_RDY = 0x04;       // PLL Ready
  const uint8_t RAW_DATA_RDY = 0x01;  // Raw data is ready

// Power Management (ITG_PWR_MGM)
  const uint8_t H_RESET = 0x80;  // Reset device and internal registers to the power-up-default settings
  const uint8_t H_SLEEP = 0x40;  // Enable low power sleep mode
  const uint8_t STBY_XG = 0x20;  // Put gyro X in standby mode (1=standby, 0=normal)
  const uint8_t STBY_YG = 0x10;  // Put gyro Y in standby mode (1=standby, 0=normal)
  const uint8_t STBY_ZG = 0x08;  // Put gyro Z in standby mode (1=standby, 0=normal)

  // Clock select (ITG_PWR_MGM)
  const uint8_t CLK_SEL_INTERNAL = 0x00;  // Internal oscillator (not recommended for stability reasons)
  const uint8_t CLK_SEL_PLL_X_GYRO = 0x01;  // PLL with X Gyro reference
  const uint8_t CLK_SEL_PLL_Y_GYRO = 0x02;  // PLL with Y Gyro reference
  const uint8_t CLK_SEL_PLL_Z_GYRO = 0x03;  // PLL with Z Gyro reference
  const uint8_t CLK_SEL_PLL_32KHZ  = 0x04;  // External clock source of 32.678 kHz
  const uint8_t CLK_SEL_PLL_19MHZ  = 0x05;  // External clock source of 19.2 MHz

} // namespace itg3200

#endif // ITG3200_REG_H
