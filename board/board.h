/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the STMicroelectronics STM3210C-EVAL evaluation board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_STM32_P107
#define BOARD_NAME              "Olimex STM32-P107"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            25000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_CL

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID            MII_STE101P_ID
#define BOARD_PHY_RMII

/*
 * IO pins assignments.
 */
#define GPIOA_SWITCH_WKUP       0

#define GPIOB_SPI2NSS           12
#define GPIOD_MMCCP             8

#define GPIOC_LED_STATUS1       6
#define GPIOC_LED_STATUS2       7
#define GPIOC_TIMING_PIN        0

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA0  - Digital input     (Rear wheel encoder B).
 * PA1  - Digital input     (Front wheel encoder B).
 * PA2  - Digital input     (Steer encoder B).
 * PA6  - Digital input     (Steer encoder A).
 * PA7  - Digital input     (Steer encoder B).
 */
#define VAL_GPIOACRL            0x44888444      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x88888888      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup:
 * Everything input with pull-up except:
 * PB6  - Alternate O.D.    (I2C1 SCL).
 * PB7  - Alternate O.D.    (I2C1 SDA).
 * PB12 - Push pull output  (MicroSD SPI2 NSS).
 * PB13 - Alternate output  (MicroSD SPI2 SCK).
 * PB14 - Input with PU     (MicroSD SPI2 MISO).
 * PB15 - Push Pull output  (MicroSD SPI2 MOSI).
 */
#define VAL_GPIOBCRL            0xEE888888      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x98918888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup:
 * Everything input with pull-up except:
 * PC0  - Push Pull output  (Control timing pin)
 * PC3  - Push Pull output  (XBee Reset)
 * PC6  - Push Pull output  (STAT1 green LED).
 * PC7  - Push Pull output  (STAT2 yellow LED).
 * PC8  - Push Pull output  (Steer Motor Direction).
 * PC9  - Push Pull output  (Steer Motor Enable, low enables).
 * PC11 - Push Pull output  (Rear Wheel Motor Enable, low enables).
 * PC12 - Push Pull output  (Rear Wheel Motor Direction, high forward).
 * PC14 - Normal input      (OSC32 IN).
 * PC15 - Normal input      (OSC32 OUT).
 */
#define VAL_GPIOCCRL            0x22882882      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x44822822      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFF53F

/*
 * Port D setup:
 * PD5  - Alternate out PP  (USART2 TX, remapped).
 * PD6  - Digital input     (USART2 RX, remapped).
 * PD8  - Input PU          (SD Card Detect, high when card is present).
 * PD9  - Input PD          (Rear Wheel Controller fault, active high).
 * PD10 - Input PD          (Steer Encoder Index, active high).
 * PD11 - Input PD          (Steer Controller Fault, active high).
 * PD12 - Digital input     (TIM4_CH1, remapped).
 * PD13 - Digital input     (TIM4_CH2, remapped).
 * PD14 - Digital input     (TIM4_CH3, remapped).
 */
#define VAL_GPIODCRL            0x84A88888      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x84448888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFF1FF

/*
 * Port E setup.
 * Everything input with pull-up except:
 * PE9  - Alternate PP      (TIM1_CH1, remapped, steer duty).
 * PE11 - Normal input      (TIM1_CH2, remapped, rear wheel duty).
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x8888A8A8      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
