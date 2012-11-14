#include <cstdint>

#include "ch.h"
#include "hal.h"

#include "EncoderTimers.h"
#include "Sample.h"
#include "SampleAndControl.h"
#include "RearWheel.h"

#ifdef __cplusplus
extern "C" {
#endif

// VectorE0 is called on a rising/falling edge of the steer index
// Need to save the steer encoder count and determine the direction, which can
// be obtained from STM32_TIM3->CNT and STM32_TIM3->CR1[4]  (DIR bit)

CH_IRQ_HANDLER(EXTI15_10_IRQHandler)
{
  CH_IRQ_PROLOGUE();

  CH_IRQ_EPILOGUE();
}

// There are 4 ways this interrupt gets called.
// 1) Timer overflow
// 2) Rising edge on IC2  ( Rear wheel encoder A)
// 3) Rising edge on IC3  (      Steer encoder A)
// 4) Risign edge on IC4  (Front wheel encoder A)
CH_IRQ_HANDLER(TIM5_IRQHandler)
{
  CH_IRQ_PROLOGUE();

  static uint32_t CCR_prev[3] = { 0, 0, 0 };  // to store the value of CNT at
                                              // the time of the input capture
                                              // events

  uint32_t sr = STM32_TIM5->SR;   // Save TIM5 status register
  STM32_TIM5->SR = ~sr;           // Write zero to bits high at time of SR read
                                  // note that writing '1' has no effect on SR
                                  // because all bits are 'rc_w0': Software can
                                  // read as well as clear this bit by writing
                                  // 0.  Writing '1' has no effect on the bit
                                  // value.
  uint32_t dir = 0;               // To store state of B channel of encoders

  SampleAndControl & sc = SampleAndControl::Instance();
  RearWheel & rw = RearWheel::Instance();
  for (int i = 0; i < 3; ++i) {
    // For IC2, IC3, IC4 events the Clockticks is the clock value at the time
    // of the current IC event, CCR[i], minus the clock value at the time of
    // the previous event CCR_previ[i]. In the event that the timer has
    // overflown since that last event, CCR will be less than CCR_prev.
    // Because of the way unsigned subtraction works, this will still give us
    // the answer we are looking for.  For example, with 8-bit unsigned types,
    // 2 - 255 will give 3, which is what we want.
    if (sr & (1 << (i + 2))) { // IC2, IC3, IC4 are on bits 2, 3, 4 of SR
      uint32_t tmp = STM32_TIM5->CCR[i + 1]; // Timer counts since last overflow
      sc.timers.Clockticks[i] = tmp - CCR_prev[i];
      CCR_prev[i] = tmp;      // save the capture compare register

      // Direction is in dir[2:0] bits
      if (i == 0) {
        dir |= (GPIOC->IDR & (1 << GPIOC_TIM8_CH2)) ? Sample::RearWheelEncoderB : 0;
        rw.PredictAndCorrect(tmp, sc.timers.Clockticks[0]);
      } else if (i == 1) {
        dir |= (GPIOA->IDR & (1 << GPIOA_TIM3_CH2)) ? Sample::SteerEncoderB : 0;
      } else {
        dir |= (GPIOD->IDR & (1 << GPIOD_TIM4_CH2)) ? Sample::FrontWheelEncoderB : 0;
      }

      sc.timers.Direction = dir;
    }
  }
  CH_IRQ_EPILOGUE();
}

#ifdef __cplusplus
}
#endif
