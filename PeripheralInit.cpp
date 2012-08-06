#include "PeripheralInit.h"
#include "ch.h"
#include "hal.h"

// For all peripheral initialization, the following apprach is taken:
// -- on reset, all inputs are input floating by default
// -- configure peripherals (i.e., timer, i2c, usart, spi)
// -- configure GPIO pins associated with all used peripherals
// -- care needs to be taken so that the default state of pins on boot up
//    doesn't turn anything on, in particular this applies to the PWM pins and
//    the enable lines that go to the motor controller.

// Private functions
static void configureRCC();
static void configureEncoderTimers(void);
static void configureMotorTimers();

// This function is called early on in main() 
void PeripheralInit()
{
  configureRCC();           // Enable peripheral clocks
  configureEncoderTimers(); // Configure TIM3,TIM4
  configureMotorTimers();   // Configure TIM1 PWM
}

static void configureRCC()
{
  // SYSCLOCK is at 72.0 MHz
  // APB1     is at 36.0 MHz
  // APB2     is at 72.0 MHz
  // Enable I2C1, SPI2, USART2, TIM3, TIM4
  RCC->APB1ENR |= ((1 << 21)  // I2C1
               |   (1 << 17)  // USART2
               |   (1 << 14)  // SPI2
               |   (1 <<  2)  // TIM4
               |   (1 <<  1));// TIM3

  // Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, TIM1, AFIO
  RCC->APB2ENR |= ((1 << 11)  // TIM1
               |   (1 <<  6)  // GPIOE
               |   (1 <<  5)  // GPIOD
               |   (1 <<  4)  // GPIOC
               |   (1 <<  3)  // GPIOB
               |   (1 <<  2)  // GPIOA
               |   (1 <<  0));// AFIO
} // configureRCC

void configureEncoderTimers(void)
{
  // Set the auto reload value for steer encoder and pulse duration timer
  STM32_TIM3->ARR = STM32_TIM4->ARR = static_cast<uint16_t>(0xFFFF);

  // Put timer 3 in encoder mode
  STM32_TIM3->SMCR = static_cast<uint16_t>(0x0003);

  // Put CC1 & CC2 in input mode IC1 mapped to TI1, IC2 mapped to TI2, no
  // prescalar
  // Enable digital filtering, see page 397 of RM0008
  // Inputs are sampled at 36.0 MHz, need 8 samples at hi/low to validate a
  // transition.  This translates to a .22 micro second delay from when the
  // edge occurs
  STM32_TIM3->CCMR1 = static_cast<uint16_t>(0x3131);

  // Configure capture compare register for pulse duration counter
  // CCxS = 01, ICxPSC = 00, ICxF = 0011
  STM32_TIM4->CCMR1 = static_cast<uint16_t>(0x3131);
  STM32_TIM4->CCMR2 = static_cast<uint16_t>(0x0031);

  // Enable capture on IC1, IC2, IC3, with normal (rising edge) polarity
  STM32_TIM4->CCER = static_cast<uint16_t>(0x0111);

  // Enable interrupts on UE, IC1, IC2, IC3
  // STM32_TIM4->DIER = static_cast<uint16_t>(0x000F);

  // Clear the counters
  STM32_TIM3->CNT = static_cast<uint16_t>(0x0000);

  // Enable all three counters
  // Do this instead when the singleton classes for rear wheel, front wheel,
  // and steer are created
  TIM3->CR1 = TIM4->CR1 = static_cast<uint16_t>(0x0001);
} // configureEncoderTimers()

static void configureMotorTimers()
{
  // TIM3 Frequency = TIM3 counter clock / (ARR + 1)
  //                = 18 MHz / 2^14
  //                = 1098.63 Hz
  STM32_TIM1->ARR = (uint16_t) 0x3FFF; // 2^14 - 1
  STM32_TIM1->PSC = (uint16_t) 0x0003; // Results in 18MHz TIM3 Counter clock
  // STM32_TIM1->RCR = 0; // reset state is fine
  STM32_TIM1->EGR = (uint16_t) 0x0001;  // immediately update

  // CR2 settings
  // Reset state is fine.
  // STM32_TIM1->CR2 = 0x0000;

  // CCMR1
  // OC2CE -- 0b0   -- OC2Ref not affected by ETRF Input
  // OC2M  -- 0b110 -- PWM Mode 1
  // OC2PE -- 0b1   -- CH2 preload enable
  // OC2FE -- 0b0   -- CH2 fast enable
  // CC2S  -- 0b00  -- CH2 is output
  // OC1CE -- 0b1   -- OC2Ref not affected by ETRF Input
  // OC1M  -- 0b110 -- PWM Mode 1
  // OC1PE -- 0b1  --  CH1 preload enable
  // OC1FE -- 0b0  --  CH1 fast enable
  // CC1S  -- 0b00  -- CH1 is output
  // NOT SURE ABOUT OC1CE, OC2CE, OC1FE, OC2FE.  Reset state is 0 for these
  // bits and my test PWM code didn't modify them, and the PWM looked fine, so
  // I will assume this is correct.
  STM32_TIM1->CCMR1 = 0x6868;

  // CCMR2
  // Reset state is fine.
  // STM32_TIM1->CCMR2 = 0x0000; // This is for CH3 and CH4, aren't used
  
  // CCR1 and CCR2 
  // reset state is fine -- 0% duty cycle
  // STM32_TIM1->CCR1 = STM32_TIM1->CCR2 = 0x0000;

  // CCER
  // CC2P -- 0b1 -- Active Low
  // CC2E -- 0b1 -- Enable CC2
  // CC1P -- 0b1 -- Active Low
  // CC1E -- 0b1 -- Enable CC1
  STM32_TIM1->CCER = (uint16_t) 0x0033;

  // CR1
  // CKD -- clock division -- 0
  // ARPE -- ARR buffering -- 1
  // DIR -- count up -- 0
  // CMS -- edge-aligned mode -- 0
  // CEN -- enable -- 1
  STM32_TIM1->CR1 = (uint16_t) 0x0081;

  // BDTR
  // MOE -- 0b1 -- Main output enable
  STM32_TIM1->BDTR = (uint16_t) 0x8000;
} // configureMotorTimers()
