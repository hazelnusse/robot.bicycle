#ifndef HMC5843_REG_H
#define HMC5843_REG_H

#include <cstdint>

namespace hmc5843 {

  // I2C Slave Address
  const uint8_t ADDR = 0x1E;        // 7-bit address

  // Register definitions
  const uint8_t  CRA = 0x0;         // Configuration Register A
  const uint8_t  CRB = 0x1;         // Configuration Register B
  const uint8_t   MR = 0x2;         // Mode Register
  const uint8_t DXRA = 0x3;         // Data Output X MSB Register
  const uint8_t DXRB = 0x4;         // Data Output X LSB Register
  const uint8_t DYRA = 0x5;         // Data Output Y MSB Register
  const uint8_t DYRB = 0x6;         // Data Output Y LSB Register
  const uint8_t DZRA = 0x7;         // Data Output Z MSB Register
  const uint8_t DZRB = 0x8;         // Data Output Z LSB Register
  const uint8_t   SR = 0x9;         // Status Register
  const uint8_t  IRA = 10;         // Identification Register A
  const uint8_t  IRB = 11;         // Identification Register B
  const uint8_t  IRC = 12;         // Identification Register C

  // Individual bits of Configuration Register A      (Table 4)
  const uint8_t MS0 = 0x01;
  const uint8_t MS1 = 0x02;
  const uint8_t DO0 = 0x04;
  const uint8_t DO1 = 0x08;
  const uint8_t DO2 = 0x10;

  // Valid Minimum Data Output Rate                   (Table 6)
  const uint8_t ONE_HALF_HZ = 0x0;
  const uint8_t ONE_HZ      = DO0;
  const uint8_t TWO_HZ      = DO1;
  const uint8_t FIVE_HZ     = DO0 | DO1;
  const uint8_t TEN_HZ      = DO2;
  const uint8_t TWENTY_HZ   = DO2 | DO0;
  const uint8_t FIFTY_HZ    = DO2 | DO1;

  // Valid Modes                                      (Table 7)
  const uint8_t NORMAL_MODE   = 0x0;
  const uint8_t POS_BIAS_MODE = MS0;
  const uint8_t NEG_BIAS_MODE = MS1;

  // Individual Bits of Configuration Register B      (Table 8)
  const uint8_t GN0 = 0x20;
  const uint8_t GN1 = 0x40;
  const uint8_t GN2 = 0x80;

  // Gain Settings                                (Table 10)
  // Gain units are in counts/Gauss               // Input field range
  const uint8_t GAIN_1620 = 0x0;                  // +/-0.7Ga
  const uint8_t GAIN_1300 = GN0;                  // +/-1.0Ga (default)
  const uint8_t GAIN_970  = GN1;                  // +/-1.5Ga
  const uint8_t GAIN_780  = GN1 | GN0;            // +/-2.0Ga
  const uint8_t GAIN_530  = GN2;                  // +/-3.2Ga
  const uint8_t GAIN_460  = GN2 | GN0;            // +/-3.8Ga
  const uint8_t GAIN_390  = GN2 | GN1;            // +/-4.5Ga
  const uint8_t GAIN_280  = GN2 | GN1 | GN0;      // +/-6.5Ga (Not recommended)

  // Mode Register Bits                           (Table 11)
  const uint8_t MD0 = 0x01;
  const uint8_t MD1 = 0x02;

  // Valid Operation Modes
  const uint8_t MODE_CONTINUOUS = 0x0;
  const uint8_t MODE_SINGLE     = MD0;
  const uint8_t MODE_IDLE       = MD1;
  const uint8_t MODE_SLEEP      = MD0 | MD1;

}; // namespace hmc5843

#endif  // HMC5843_REG_H
