#include "ch.h"
#include "hal.h"
#include "PeripheralInit.h"
#include "Constants.h"


// For all peripheral initialization, the following apprach is taken:
// -- on reset, all inputs are input floating by default
// -- configure peripherals (i.e., timer, i2c, usart, spi)
// -- configure GPIO pins associated with all used peripherals
// -- care needs to be taken so that the default state of pins on boot up
//    doesn't turn anything on, in particular this applies to the PWM pins and
//    the enable lines that go to the motor controller.

// Private functions
static void configureRCC();
static void configureEncoderTimers();
static void configureMotorPWM();

// This function is called in boardInit(), which is called during halInit()
void PeripheralInit()
{
  configureRCC();           // Enable peripheral clocks
  configureEncoderTimers();        // Configure Timers
  configureMotorPWM();      // Configure Motor PWM
}

static void configureRCC()
{
  // SYSCLOCK is at 168.0 MHz
  // APB1 timers are clocked at  84.0 MHz
  // APB2 timers are clocked at 168.0 MHz

  // Enable GPIOF, GPIOE, GPIOD, GPIOC, GPIOB, GPIOA
  RCC->AHB1ENR |= ((1 <<  5)  // GPIOE
               |   (1 <<  4)  // GPIOE
               |   (1 <<  3)  // GPIOD
               |   (1 <<  2)  // GPIOC
               |   (1 <<  1)  // GPIOB
               |   (1 <<  0));// GPIOA
  // Enable I2C2, USART2, TIM5, TIM4, TIM3
  RCC->APB1ENR |= ((1 << 22)  // I2C2  --  MPU-6050
               |   (1 << 17)  // USART2--  XBee Serial
               |   (1 <<  3)  // TIM5  --  Encoder speed measurement
               |   (1 <<  2)  // TIM4  --  Front wheel angle measurement
               |   (1 <<  1));// TIM3  --  Steer angle measurement
  // Enable TIM10, SDIO, ADC3, TIM8, TIM1
  RCC->APB2ENR |= ((1 << 11)  // SDIO  --  SD Card
               |   (1 << 10)  // ADC3  --  Battery monitor
               |   (1 <<  1)  // TIM8  --  Rear wheel angle measurement
               |   (1 <<  0));// TIM1  --  PWM Output
} // configureRCC

void configureEncoderTimers(void)
{
  // Position encoder timers are all 16-bit
  // TIM3 and TIM4 are APB1@84.0MHz, TIM8 is APB2@168.0MHz.
  //
  // The clock speed only affect the filtering that is done in the edge
  // detector, which is set to be as slow as possible.  To be considered a
  // rising edge, the incoming signal must be high for 3.0us and 1.5us
  // respectively.
  // STM32_TIM8: Rear wheel encoder timer                  
  // STM32_TIM3: Steer encoder timer                  
  // STM32_TIM4: Front wheel encoder timer                  
  stm32_tim_t * encoderTimers[3] = {STM32_TIM8, STM32_TIM3, STM32_TIM4};
  for (auto timer : encoderTimers) {
    timer->SMCR = 3;          // Encoder mode 3
    timer->CCER = 0;          // rising edge polarity
    timer->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
    timer->CCMR1 = 0xF1F1;    // f_DTS/32, N=8, IC1->TI1, IC2->TI2
    timer->EGR = 1;           // Generate an update event
    timer->CNT = 0; // 1 << 15;     // Initialize counter with 2^15
    timer->CR1 = 1;           // Enable the counter
  }

  // STM32_TIM5_CH2:  Rear wheel encoder A
  // STM32_TIM5_CH3:  Steer encoder A
  // STM32_TIM5_CH4:  Front wheel encoder A
  //
  // Configure prescalar for speed estimates.  This is needed in order to slow
  // down the sampling of the IC channels to avoid spurious velocity
  // measurements.  This setting directly affects the accuracy of the speed
  // estimate as well as the time before an overflow event will occur.
  // Overflows indicate the speed is very slow, likely stationary.
  STM32_TIM5->PSC = 20; // f_CLK =  84.0MHz/(20+1)==4.0 MHz, --> 0.25 us / count

  // Set the ARR so that counting goes from 0 to 2**16 - 1 and then starts
  // counting again from 0
  STM32_TIM5->ARR = 0xFFFFFFFF;

  // Configure capture compare register for pulse duration counter
  // CCxS = 01, ICxPSC = 00, ICxF = 0011: f_sampling = f_CLK, N = 8
  // This means rising edge must be stable for 0.25 us * 8 = 2 us to be
  // considered a transition.
  STM32_TIM5->CCMR1 = 0x3100;     // CH2
  STM32_TIM5->CCMR2 = 0x3131;     // CH3, CH4

  // Enable capture compare bits, detect rising and falling edge detection
  STM32_TIM5->CCER = 0xBBB0;      // IC2, IC3, IC4

  // Clear speed count
  STM32_TIM5->CNT = 0;

  // Generate an update event so that all registers are updated.
  STM32_TIM5->EGR = 1;

  // Enable interrupts on CH4, CH3, CH2
  STM32_TIM5->DIER = (1 << 4) | (1 << 3) | (1 << 2);

  // Enable speed timer 
  STM32_TIM5->CR1 = 1;
} // configureEncoderTimers()

static void configureMotorPWM()
{
  // Disable the timer and enable auto preload register
  STM32_TIM1->CR1 = (1 << 7);
  
  // TIM1 counter clock = 168.0MHz / (PSC + 1)
  STM32_TIM1->PSC = 0; // Results in 168MHz TIM3 Counter clock

  // TIM1 Frequency = TIM1 counter clock / (ARR + 1)
  //                = 168 MHz / 2^16
  //                = 2563.48 Hz
  STM32_TIM1->ARR = reg::PWM_ARR; // 2^16 - 1

  // Select PWM1 mode for OC1 and OC2 (OCX inactive when CNT<CCRX)
  STM32_TIM1->CCMR1 = (0b110 << 12) | (0b110 << 4);

  // Select 0% duty cycle for channel 1 and 2
  STM32_TIM1->CCR[0] = 0;
  STM32_TIM1->CCR[1] = 0;

  // Select OC1 and OC2 polarity to active low and enable them.  We choose
  // active to be low because Copley drives have been configured with pull up
  // resistors to ensure that the 5.0V --> 0% duty cycle.
  // Since OC1 and OC2 are in PWM1 mode, this means:
  //   CNT <  CCR[X] ===> OCX is active (low)
  //   CNT >= CCR[X] ===> OCX is inactive (high)
  STM32_TIM1->CCER = 0x0033;

  // Generate an update event to update all timer registers
  STM32_TIM1->EGR = 1;

  // TIM1 enable
  STM32_TIM1->CR1 |= 1;

  // TIM1 Main Output Enable
  STM32_TIM1->BDTR = (1 << 15);
} // configureMotorPWM()
