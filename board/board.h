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
 * Setup for Olimex STM32-H407 board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_STM32_H407
#define BOARD_NAME                  "Olimex STM32-H407"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_KS8721_ID
#define BOARD_PHY_RMII

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                12000000
#endif


/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON_WKUP           0
#define GPIOA_PIN1                  1
#define GPIOA_PIN2                  2
#define GPIOA_PIN3                  3
#define GPIOA_PIN4                  4
#define GPIOA_PIN5                  5
#define GPIOA_TIM3_CH1              6
#define GPIOA_TIM3_CH2              7
#define GPIOA_USB_HS_VBUSON         8
#define GPIOA_OTG_FS_VBUS           9
#define GPIOA_USB_FS_VBUSON         10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_FS_DP             12
#define GPIOA_JTAG_TMS              13
#define GPIOA_JTAG_TCK              14
#define GPIOA_JTAG_TDI              15

#define GPIOB_PIN0                  0
#define GPIOB_PIN1                  1
#define GPIOB_BOOT1                 2
#define GPIOB_JTAG_TDO              3
#define GPIOB_JTAG_TRST             4
#define GPIOB_USB_HS_FAULT          5
#define GPIOB_USB_FS_FAULT          6
#define GPIOB_PIN7                  7
#define GPIOB_PIN8                  8
#define GPIOB_PIN9                  9
#define GPIOB_PIN10                 10
#define GPIOB_PIN11                 11
#define GPIOB_OTG_HS_ID             12
#define GPIOB_OTG_HS_VBUS           13
#define GPIOB_OTG_HS_DM             14
#define GPIOB_OTG_HS_DP             15

#define GPIOC_PIN0                  0
#define GPIOC_PIN1                  1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_PIN4                  4
#define GPIOC_PIN5                  5
#define GPIOC_TIM8_CH1              6
#define GPIOC_TIM8_CH2              7
#define GPIOC_SD_D0                 8
#define GPIOC_SD_D1                 9
#define GPIOC_SD_D2                 10
#define GPIOC_SD_D3                 11
#define GPIOC_SD_CLK                12
#define GPIOC_LED                   13
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_PIN0                  0
#define GPIOD_PIN1                  1
#define GPIOD_SD_CMD                2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_USART2_TX             5
#define GPIOD_USART2_RX             6
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_TIM4_CH1              12
#define GPIOD_TIM4_CH2              13
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_TIM1_CH1              9
#define GPIOE_PIN10                 10
#define GPIOE_TIM1_CH2              11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOF_I2C2_SDA              0
#define GPIOF_I2C2_SCL              1
#define GPIOF_XBEE_RESET            2
#define GPIOF_TIMING_PIN            3
#define GPIOF_RW_ENABLE             4
#define GPIOF_STEER_ENABLE          5
#define GPIOF_RW_DIR                6
#define GPIOF_STEER_DIR             7
#define GPIOF_RW_FAULT              8
#define GPIOF_STEER_FAULT           9
#define GPIOF_VIN_MON               10
#define GPIOF_STEER_ENC_I           11
#define GPIOF_PIN12                 12
#define GPIOF_PIN13                 13
#define GPIOF_PIN14                 14
#define GPIOF_PIN15                 15

#define GPIOG_PIN0                  0
#define GPIOG_PIN1                  1
#define GPIOG_PIN2                  2
#define GPIOG_PIN3                  3
#define GPIOG_PIN4                  4
#define GPIOG_PIN5                  5
#define GPIOG_PIN6                  6
#define GPIOG_PIN7                  7
#define GPIOG_PIN8                  8
#define GPIOG_PIN9                  9
#define GPIOG_PIN10                 10
#define GPIOG_PIN11                 11
#define GPIOG_PIN12                 12
#define GPIOG_PIN13                 13
#define GPIOG_PIN14                 14
#define GPIOG_PIN15                 15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_PIN15                 15

#define GPIOI_PIN0                  0
#define GPIOI_PIN1                  1
#define GPIOI_PIN2                  2
#define GPIOI_PIN3                  3
#define GPIOI_PIN4                  4
#define GPIOI_PIN5                  5
#define GPIOI_PIN6                  6
#define GPIOI_PIN7                  7
#define GPIOI_PIN8                  8
#define GPIOI_PIN9                  9
#define GPIOI_PIN10                 10
#define GPIOI_PIN11                 11
#define GPIOI_PIN12                 12
#define GPIOI_PIN13                 13
#define GPIOI_PIN14                 14
#define GPIOI_PIN15                 15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 *
 * PA0  - BUTTON_WKUP         (input floating, AF0, external pull down).
 * PA1  - PIN1                (input pullup, AF0).
 * PA2  - PIN2                (input pullup, AF0).
 * PA3  - PIN3                (input pullup, AF0).
 * PA4  - PIN4                (input pullup, AF0).
 * PA5  - PIN5                (input pullup, AF0).
 * PA6  - TIM3_CH1            (input floating, AF2).
 * PA7  - TIM3_CH2            (input floating, AF2).
 * PA8  - USB_HS_VBUSON       (output pushpull maximum, AF0).
 * PA9  - OTG_FS_VBUS         (input pulldown, AF0).
 * PA10 - USB_FS_VBUSON       (output pushpull maximum, AF0).
 * PA11 - OTG_FS_DM           (input pullup, AF0).
 * PA12 - OTG_FS_DP           (input pullup, AF0).
 * PA13 - JTAG_TMS            (alternate push pull, AF0, external pull up).
 * PA14 - JTAG_TCK            (alternate push pull, AF0, external pull up).
 * PA15 - JTAG_TDI            (alternate push pull, AF0, external pull up).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_BUTTON_WKUP) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN5) |             \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM3_CH1) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM3_CH2) |     \
                                     PIN_MODE_OUTPUT(GPIOA_USB_HS_VBUSON) |   \
                                     PIN_MODE_INPUT(GPIOA_OTG_FS_VBUS) |      \
                                     PIN_MODE_OUTPUT(GPIOA_USB_FS_VBUSON) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TMS) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TCK) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUTTON_WKUP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TIM3_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TIM3_CH2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_HS_VBUSON) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS_VBUSON) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TMS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TCK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_BUTTON_WKUP) |     \
                                     PIN_OSPEED_100M(GPIOA_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOA_TIM3_CH1) |        \
                                     PIN_OSPEED_100M(GPIOA_TIM3_CH2) |        \
                                     PIN_OSPEED_100M(GPIOA_USB_HS_VBUSON) |   \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_VBUS) |     \
                                     PIN_OSPEED_100M(GPIOA_USB_FS_VBUSON) |   \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) |       \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) |       \
                                     PIN_OSPEED_100M(GPIOA_JTAG_TMS) |        \
                                     PIN_OSPEED_100M(GPIOA_JTAG_TCK) |        \
                                     PIN_OSPEED_100M(GPIOA_JTAG_TDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUTTON_WKUP) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN1) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN5) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_TIM3_CH1) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_TIM3_CH2) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_HS_VBUSON) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS_VBUSON) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_TMS) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_TCK) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_TDI))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_BUTTON_WKUP) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOA_TIM3_CH1) |           \
                                     PIN_ODR_HIGH(GPIOA_TIM3_CH2) |           \
                                     PIN_ODR_HIGH(GPIOA_USB_HS_VBUSON) |      \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_VBUS) |        \
                                     PIN_ODR_HIGH(GPIOA_USB_FS_VBUSON) |      \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |          \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |          \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TMS) |           \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TCK) |           \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUTTON_WKUP, 0) |      \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOA_TIM3_CH1, 2) |         \
                                     PIN_AFIO_AF(GPIOA_TIM3_CH2, 2))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_USB_HS_VBUSON, 0) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_VBUS, 0) |      \
                                     PIN_AFIO_AF(GPIOA_USB_FS_VBUSON, 0) |   \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |       \
                                     PIN_AFIO_AF(GPIOA_JTAG_TMS, 0) |         \
                                     PIN_AFIO_AF(GPIOA_JTAG_TCK, 0) |         \
                                     PIN_AFIO_AF(GPIOA_JTAG_TDI, 0))

/*
 * GPIOB setup:
 *
 * PB0  - PIN0                      (input pullup).
 * PB1  - PIN1                      (input pullup).
 * PB2  - BOOT1                     (input floating).
 * PB3  - JTAG_TDO                  (alternate 0).
 * PB4  - JTAG_TRST                 (alternate 0).
 * PB5  - USB_HS_FAULT              (input floating).
 * PB6  - USB_FS_FAULT              (input floating).
 * PB7  - PIN7                      (input pullup).
 * PB8  - PIN8                      (input pullup).
 * PB9  - PIN9                      (input pullup).
 * PB10 - PIN10                     (input pullup).
 * PB11 - PIN11                     (input pullup).
 * PB12 - OTG_HS_ID                 (alternate 12).
 * PB13 - OTG_HS_VBUS               (input pulldown).
 * PB14 - OTG_HS_DM                 (alternate 12).
 * PB15 - OTG_HS_DP                 (alternate 12).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOB_BOOT1) |            \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TDO) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TRST) |    \
                                     PIN_MODE_INPUT(GPIOB_USB_HS_FAULT) |     \
                                     PIN_MODE_INPUT(GPIOB_USB_FS_FAULT) |     \
                                     PIN_MODE_INPUT(GPIOB_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |            \
                                     PIN_MODE_ALTERNATE(GPIOB_OTG_HS_ID) |    \
                                     PIN_MODE_INPUT(GPIOB_OTG_HS_VBUS) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_OTG_HS_DM) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_OTG_HS_DP))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TDO) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TRST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USB_HS_FAULT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USB_FS_FAULT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OTG_HS_ID) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OTG_HS_VBUS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OTG_HS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OTG_HS_DP))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOB_BOOT1) |           \
                                     PIN_OSPEED_100M(GPIOB_JTAG_TDO) |        \
                                     PIN_OSPEED_100M(GPIOB_JTAG_TRST) |       \
                                     PIN_OSPEED_100M(GPIOB_USB_HS_FAULT) |    \
                                     PIN_OSPEED_100M(GPIOB_USB_FS_FAULT) |    \
                                     PIN_OSPEED_100M(GPIOB_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOB_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOB_OTG_HS_ID) |       \
                                     PIN_OSPEED_100M(GPIOB_OTG_HS_VBUS) |     \
                                     PIN_OSPEED_100M(GPIOB_OTG_HS_DM) |       \
                                     PIN_OSPEED_100M(GPIOB_OTG_HS_DP))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_PIN0) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN1) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_JTAG_TDO) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_JTAG_TRST) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_USB_HS_FAULT) | \
                                     PIN_PUPDR_FLOATING(GPIOB_USB_FS_FAULT) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN7) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN8) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN9) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN10) |          \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN11) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_OTG_HS_ID) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOB_OTG_HS_VBUS) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_OTG_HS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_OTG_HS_DP))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOB_BOOT1) |              \
                                     PIN_ODR_HIGH(GPIOB_JTAG_TDO) |           \
                                     PIN_ODR_HIGH(GPIOB_JTAG_TRST) |          \
                                     PIN_ODR_HIGH(GPIOB_USB_HS_FAULT) |       \
                                     PIN_ODR_HIGH(GPIOB_USB_FS_FAULT) |       \
                                     PIN_ODR_HIGH(GPIOB_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOB_OTG_HS_ID) |          \
                                     PIN_ODR_HIGH(GPIOB_OTG_HS_VBUS) |        \
                                     PIN_ODR_HIGH(GPIOB_OTG_HS_DM) |          \
                                     PIN_ODR_HIGH(GPIOB_OTG_HS_DP))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0) |            \
                                     PIN_AFIO_AF(GPIOB_JTAG_TDO, 0) |         \
                                     PIN_AFIO_AF(GPIOB_JTAG_TRST, 0) |        \
                                     PIN_AFIO_AF(GPIOB_USB_HS_FAULT, 0) |     \
                                     PIN_AFIO_AF(GPIOB_USB_FS_FAULT, 0) |     \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOB_OTG_HS_ID, 12) |       \
                                     PIN_AFIO_AF(GPIOB_OTG_HS_VBUS, 0) |      \
                                     PIN_AFIO_AF(GPIOB_OTG_HS_DM, 12) |       \
                                     PIN_AFIO_AF(GPIOB_OTG_HS_DP, 12))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (input pullup).
 * PC1  - PIN1                      (input pullup).
 * PC2  - PIN1                      (input pullup).
 * PC3  - PIN3                      (input pullup).
 * PC4  - PIN4                      (input floating, AF0, external pullup).
 * PC5  - PIN5                      (input floating, AF0, external pullup).
 * PC6  - TIM8_CH1                  (alternate 3).
 * PC7  - TIM8_CH2                  (alternate 3).
 * PC8  - SD_D0                     (alternate 12).
 * PC9  - SD_D1                     (alternate 12).
 * PC10 - SD_D2                     (alternate 12).
 * PC11 - SD_D3                     (alternate 12).
 * PC12 - SD_CLK                    (alternate 12).
 * PC13 - LED                       (output pushpull maximum).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN1) |             \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN2) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN3) |         \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |             \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM8_CH1) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM8_CH2) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D0) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D1) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D2) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D3) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_CLK) |       \
                                     PIN_MODE_OUTPUT(GPIOC_LED) |             \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM8_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM8_CH2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D0) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D2) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D3) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_CLK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOC_TIM8_CH1) |        \
                                     PIN_OSPEED_100M(GPIOC_TIM8_CH2) |        \
                                     PIN_OSPEED_100M(GPIOC_SD_D0) |           \
                                     PIN_OSPEED_100M(GPIOC_SD_D1) |           \
                                     PIN_OSPEED_100M(GPIOC_SD_D2) |           \
                                     PIN_OSPEED_100M(GPIOC_SD_D3) |           \
                                     PIN_OSPEED_100M(GPIOC_SD_CLK) |          \
                                     PIN_OSPEED_100M(GPIOC_LED) |             \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) |        \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_PIN0) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN1) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5) |           \
                                     PIN_PUPDR_FLOATING(GPIOC_TIM8_CH1) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_TIM8_CH2) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D0) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D1) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D2) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D3) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_CLK) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_LED) |          \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOC_TIM8_CH1) |           \
                                     PIN_ODR_HIGH(GPIOC_TIM8_CH2) |           \
                                     PIN_ODR_HIGH(GPIOC_SD_D0) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_D1) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_D2) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_D3) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_CLK) |             \
                                     PIN_ODR_HIGH(GPIOC_LED) |                \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |           \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN2, 5) |             \
                                     PIN_AFIO_AF(GPIOC_PIN3, 5) |             \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOC_TIM8_CH1, 3) |         \
                                     PIN_AFIO_AF(GPIOC_TIM8_CH2, 3))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SD_D0, 12) |           \
                                     PIN_AFIO_AF(GPIOC_SD_D1, 12) |           \
                                     PIN_AFIO_AF(GPIOC_SD_D2, 12) |           \
                                     PIN_AFIO_AF(GPIOC_SD_D3, 12) |           \
                                     PIN_AFIO_AF(GPIOC_SD_CLK, 12) |          \
                                     PIN_AFIO_AF(GPIOC_LED, 0) |              \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - SD_CMD                    (alternate 12).
 * PD3  - PIN3                      (input pullup).
 * PD4  - PIN4                      (input pullup).
 * PD5  - USART2_TX                 (alternate 7).
 * PD6  - USART2_RX                 (alternate 7).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - TIM4_CH1                  (alternate 2).
 * PD13 - TIM4_CH2                  (alternate 2).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |             \
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD) |       \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |             \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_RX) |    \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |            \
                                     PIN_MODE_ALTERNATE(GPIOD_TIM4_CH1) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_TIM4_CH2) |     \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_TIM4_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_TIM4_CH2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOD_SD_CMD) |          \
                                     PIN_OSPEED_100M(GPIOD_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOD_USART2_TX) |       \
                                     PIN_OSPEED_100M(GPIOD_USART2_RX) |       \
                                     PIN_OSPEED_100M(GPIOD_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOD_TIM4_CH1) |        \
                                     PIN_OSPEED_100M(GPIOD_TIM4_CH2) |        \
                                     PIN_OSPEED_100M(GPIOD_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0) |           \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1) |           \
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CMD) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4) |           \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_TX) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_RX) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |           \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |           \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |           \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |          \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11) |          \
                                     PIN_PUPDR_FLOATING(GPIOD_TIM4_CH1) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_TIM4_CH2) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |          \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOD_SD_CMD) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOD_USART2_TX) |          \
                                     PIN_ODR_HIGH(GPIOD_USART2_RX) |          \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOD_TIM4_CH1) |           \
                                     PIN_ODR_HIGH(GPIOD_TIM4_CH2) |           \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 12) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOD_USART2_TX, 7) |        \
                                     PIN_AFIO_AF(GPIOD_USART2_RX, 7) |        \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOD_TIM4_CH1, 2) |         \
                                     PIN_AFIO_AF(GPIOD_TIM4_CH2, 2) |         \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - PIN2                      (input pullup).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - PIN6                      (input pullup).
 * PE7  - PIN7                      (input pullup).
 * PE8  - PIN8                      (input pullup).
 * PE9  - TIM1_CH1                  (push-pull, AF1).
 * PE10 - PIN10                     (input pullup).
 * PE11 - TIM1_CH2                  (push-pull, AF1).
 * PE12 - PIN12                     (input pullup).
 * PE13 - PIN13                     (input pullup).
 * PE14 - PIN14                     (input pullup).
 * PE15 - PIN15                     (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |             \
                                     PIN_MODE_ALTERNATE(GPIOE_TIM1_CH1) |     \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |            \
                                     PIN_MODE_ALTERNATE(GPIOE_TIM1_CH2) |     \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_TIM1_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_TIM1_CH2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOE_TIM1_CH1) |        \
                                     PIN_OSPEED_100M(GPIOE_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOE_TIM1_CH2) |        \
                                     PIN_OSPEED_100M(GPIOE_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) |           \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) |           \
                                     PIN_PUPDR_FLOATING(GPIOE_TIM1_CH1) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) |          \
                                     PIN_PUPDR_FLOATING(GPIOE_TIM1_CH2) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12) |          \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) |          \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14) |          \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOE_TIM1_CH1) |           \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOE_TIM1_CH2) |           \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOE_TIM1_CH1, 1) |         \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOE_TIM1_CH2, 1) |         \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOF setup:
 *
 * PF0  - I2C2_SDA                  (output open drain, pullup, AF4).
 * PF1  - I2C2_SCL                  (output open drain, pullup, AF4).
 * PF2  - XBEE_RESET                (output pushpull, pullup, AF0).
 * PF3  - TIMING_PIN                (output pushpull, AF0).
 * PF4  - RW_ENABLE                 (output pushpull, AF0).
 * PF5  - STEER_ENABLE              (output pushpull, AF0).
 * PF6  - RW_DIR                    (output pushpull, AF0).
 * PF7  - STEER_DIR                 (output pushpull, AF0).
 * PF8  - RW_FAULT                  (input floating, AF0).
 * PF9  - STEER_FAULT               (input floating, AF0).
 * PF10 - VIN_MON                   (input floating, AF0).
 * PF11 - STEER_ENC_I               (input floating, AF0).
 * PF12 - PIN12                     (input pullup, AF0).
 * PF13 - PIN13                     (input pullup, AF0).
 * PF14 - PIN14                     (input pullup, AF0).
 * PF15 - PIN15                     (input pullup, AF0).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_I2C2_SDA) |         \
                                     PIN_MODE_ALTERNATE(GPIOF_I2C2_SCL) |         \
                                     PIN_MODE_OUTPUT(GPIOF_XBEE_RESET) |      \
                                     PIN_MODE_OUTPUT(GPIOF_TIMING_PIN) |      \
                                     PIN_MODE_OUTPUT(GPIOF_RW_ENABLE) |       \
                                     PIN_MODE_OUTPUT(GPIOF_STEER_ENABLE) |    \
                                     PIN_MODE_OUTPUT(GPIOF_RW_DIR) |          \
                                     PIN_MODE_OUTPUT(GPIOF_STEER_DIR) |       \
                                     PIN_MODE_INPUT(GPIOF_RW_FAULT) |         \
                                     PIN_MODE_INPUT(GPIOF_STEER_FAULT) |      \
                                     PIN_MODE_INPUT(GPIOF_VIN_MON) |          \
                                     PIN_MODE_INPUT(GPIOF_STEER_ENC_I) |      \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SDA) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SCL) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_XBEE_RESET) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_TIMING_PIN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_RW_ENABLE) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STEER_ENABLE) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_RW_DIR) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STEER_DIR) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_RW_FAULT) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STEER_FAULT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_VIN_MON) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STEER_ENC_I) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_I2C2_SDA) |        \
                                     PIN_OSPEED_100M(GPIOF_I2C2_SCL) |        \
                                     PIN_OSPEED_100M(GPIOF_XBEE_RESET) |      \
                                     PIN_OSPEED_100M(GPIOF_TIMING_PIN) |      \
                                     PIN_OSPEED_100M(GPIOF_RW_ENABLE) |       \
                                     PIN_OSPEED_100M(GPIOF_STEER_ENABLE) |    \
                                     PIN_OSPEED_100M(GPIOF_RW_DIR) |          \
                                     PIN_OSPEED_100M(GPIOF_STEER_DIR) |       \
                                     PIN_OSPEED_100M(GPIOF_RW_FAULT) |        \
                                     PIN_OSPEED_100M(GPIOF_STEER_FAULT) |     \
                                     PIN_OSPEED_100M(GPIOF_VIN_MON) |         \
                                     PIN_OSPEED_100M(GPIOF_STEER_ENC_I) |     \
                                     PIN_OSPEED_100M(GPIOF_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_I2C2_SDA) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_I2C2_SCL) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_XBEE_RESET) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_TIMING_PIN) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_RW_ENABLE) |    \
                                     PIN_PUPDR_FLOATING(GPIOF_STEER_ENABLE) | \
                                     PIN_PUPDR_FLOATING(GPIOF_RW_DIR) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_STEER_DIR) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_RW_FAULT) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_STEER_FAULT) |    \
                                     PIN_PUPDR_FLOATING(GPIOF_VIN_MON) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_STEER_ENC_I) |  \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |          \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |          \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |          \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_I2C2_SDA) |           \
                                     PIN_ODR_HIGH(GPIOF_I2C2_SCL) |           \
                                     PIN_ODR_HIGH(GPIOF_XBEE_RESET) |         \
                                     PIN_ODR_HIGH(GPIOF_TIMING_PIN) |         \
                                     PIN_ODR_HIGH(GPIOF_RW_ENABLE) |          \
                                     PIN_ODR_HIGH(GPIOF_STEER_ENABLE) |       \
                                     PIN_ODR_HIGH(GPIOF_RW_DIR) |             \
                                     PIN_ODR_HIGH(GPIOF_STEER_DIR) |          \
                                     PIN_ODR_HIGH(GPIOF_RW_FAULT) |           \
                                     PIN_ODR_HIGH(GPIOF_STEER_FAULT) |        \
                                     PIN_ODR_HIGH(GPIOF_VIN_MON) |            \
                                     PIN_ODR_HIGH(GPIOF_STEER_ENC_I) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_I2C2_SDA, 4) |         \
                                     PIN_AFIO_AF(GPIOF_I2C2_SCL, 4) |         \
                                     PIN_AFIO_AF(GPIOF_XBEE_RESET, 0) |       \
                                     PIN_AFIO_AF(GPIOF_TIMING_PIN, 0) |       \
                                     PIN_AFIO_AF(GPIOF_RW_ENABLE, 0) |        \
                                     PIN_AFIO_AF(GPIOF_STEER_ENABLE, 0) |     \
                                     PIN_AFIO_AF(GPIOF_RW_DIR, 0) |           \
                                     PIN_AFIO_AF(GPIOF_STEER_DIR, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_RW_FAULT, 0) |         \
                                     PIN_AFIO_AF(GPIOF_STEER_FAULT, 0) |      \
                                     PIN_AFIO_AF(GPIOF_VIN_MON, 0) |          \
                                     PIN_AFIO_AF(GPIOF_STEER_ENC_I, 0) |      \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input pullup).
 * PG1  - PIN1                      (input pullup).
 * PG2  - PIN2                      (input pullup).
 * PG3  - PIN3                      (input pullup).
 * PG4  - PIN4                      (input pullup).
 * PG5  - PIN5                      (input pullup).
 * PG6  - PIN6                      (input pullup).
 * PG7  - PIN7                      (input pullup).
 * PG8  - PIN8                      (input pullup).
 * PG9  - PIN9                      (input pullup).
 * PG10 - PIN10                     (input pullup).
 * PG11 - PIN11                     (input pullup).
 * PG12 - PIN12                     (input pullup).
 * PG13 - PIN13                     (input pullup).
 * PG14 - PIN14                     (input pullup).
 * PG15 - PIN15                     (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_PIN0) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN1) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN5) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN6) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN7) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN8) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN9) |           \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN10) |          \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN11) |          \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN12) |          \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN13) |          \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN14) |          \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |           \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) |          \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |           \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |          \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |          \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |          \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |          \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |          \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |             \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) |           \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_100M(GPIOI_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |           \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |          \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |          \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |          \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |          \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |          \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0))


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
