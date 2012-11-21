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

// There are 3 ways this interrupt gets called.
// 1) Rising edge on IC2  ( Rear wheel encoder A)
// 2) Rising edge on IC3  (      Steer encoder A)
// 3) Risign edge on IC4  (Front wheel encoder A)
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

  uint32_t CCR;
  // Rear wheel encoder A edge
  if (sr & (1 << 2)) { // IC2
    CCR = STM32_TIM5->CCR[1];  // Channel 2
    sc.timers.Clockticks[0] = CCR - CCR_prev[0];
    CCR_prev[0] = CCR;
    float w = cf::Wheel_rad_halfquad_counts_per_sec / sc.timers.Clockticks[0];
    if (STM32_TIM8->CR1 & (1 << 4)) {
      dir |=  Sample::RearWheelEncoderB;
      w = -w;
    }
    rw.Update(CCR, w);    // Update state estimation
  }

  // Steer encoder A edge
  if (sr & (1 << 3)) { // IC3
    CCR = STM32_TIM5->CCR[2];  // Channel 3
    sc.timers.Clockticks[1] = CCR - CCR_prev[1];
    CCR_prev[1] = CCR;
    if (STM32_TIM3->CR1 & (1 << 4)) {
      dir |=  Sample::SteerEncoderB;
    }
  }

  // Front wheel encoder A edge
  if (sr & (1 << 4)) { // IC4
    CCR = STM32_TIM5->CCR[3];  // Channel 4
    sc.timers.Clockticks[2] = CCR - CCR_prev[2];
    CCR_prev[2] = CCR;
    if (STM32_TIM4->CR1 & (1 << 4)) {
      dir |=  Sample::FrontWheelEncoderB;
    }
  }

  sc.timers.Direction = dir;

  CH_IRQ_EPILOGUE();
}

#ifdef __cplusplus
}
#endif
