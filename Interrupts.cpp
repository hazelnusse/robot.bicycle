#include <cstdint>

#include "ch.h"
#include "hal.h"

#include "EncoderTimers.h"
#include "Interrupts.h"
#include "SampleAndControl.h"

// There are 4 ways this interrupt gets called.
// 1) Timer overflow
// 2) Rising edge on IC2  ( Rear wheel encoder A)
// 3) Rising edge on IC3  (      Steer encoder A)
// 4) Risign edge on IC4  (Front wheel encoder A)
CH_IRQ_HANDLER(VectorB8)
{
  CH_IRQ_PROLOGUE();

  static uint32_t overflows[3] = { 0, 0, 0 }; // to store # of overflows per
                                              // per input channel
  static uint32_t CCR_prev[3] = { 0, 0, 0 };  // to store the value of CNT at
                                              // the time of the input capture
                                              // events

  uint32_t sr = STM32_TIM5->SR;   // Save TIM5 status register
  uint32_t dir = 0;               // To store state of B channel of encoders
  if (GPIOC->IDR & (1 << GPIOC_TIM8_CH2))
    dir = 1;
  if (GPIOA->IDR & (1 << GPIOA_TIM3_CH2))
    dir |= (1 << 1); 
  if (GPIOD->IDR & (1 << GPIOD_TIM4_CH2))
    dir |= (1 << 2);

  STM32_TIM5->SR = ~sr;           // Write zero to bits high at time of SR read
                                  // note that writing '1' has no effect on SR
                                  // because all bits are 'rc_w0': Software can
                                  // read as well as clear this bit by writing
                                  // 0.  Writing '1' has no effect on the bit
                                  // value.

  if (sr & 1) {                   // UE (overflow, since TIM5 is upcounting)
    // When the timer has overflowed, we increment the overflow counters:
    ++overflows[0]; ++overflows[1]; ++overflows[2];
  }

  for (int i = 0; i < 3; ++i) {
    // For IC2, IC3, IC4 events the Clockticks is the clock
    // value at the time of the current IC event, CCR[i], minus the clock value at the
    // time of the previous event, CCR_prev[i], plus 2^16 times the number of
    // overflows.
    if (sr & (1 << (i + 2))) { // IC2, IC3, IC4 are on bits 2, 3, 4 of SR
      uint32_t tmp = STM32_TIM5->CCR[i]; // Timer counts since last overflow
      tmp &= 0x0000FFFF;                 // Mask out top bits, just to be safe

      SampleAndControl::timers.Clockticks[i] = tmp + overflows[i]*(1 << 16) - CCR_prev[i];
      overflows[i] = 0;
      CCR_prev[i] = tmp;      // save the capture compare register

      // Direction is in dir[2:0] bits
      if (dir & (1 << i))     // Encoder B line is high
        SampleAndControl::timers.Clockticks[i] |= (1 << 31);  // set high bit
      else                    // Encoder B line is low
        SampleAndControl::timers.Clockticks[i] &= ~(1 << 31); // clear high bit
    }
  }
  CH_IRQ_EPILOGUE();
}
