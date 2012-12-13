#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

#include "SampleBuffer.h"

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
msg_t SampleAndControl::Stop()
{
  msg_t m;
  chThdTerminate(tp_control);
  m = chThdWait(tp_control);
  tp_control = 0;
  return m;
}

__attribute__((noreturn))
inline
void SampleAndControl::controlThread_(__attribute__((unused))void * arg)
{
  SampleAndControl::Instance().controlThread();
}

__attribute__((noreturn))
inline
void SampleAndControl::writeThread_(__attribute__((unused)) void * arg)
{
  SampleAndControl::Instance().writeThread();
}

inline
void SampleAndControl::sampleTimers(Sample & s)
{
  s.SystemTime = STM32_TIM5->CNT;
  s.RearWheelAngle = STM32_TIM8->CNT;
  s.SteerAngle = STM32_TIM3->CNT;
  s.FrontWheelAngle = STM32_TIM4->CNT;
  s.CCR_rw = STM32_TIM1->CCR[0];      // RW PWM duty cycle
  s.CCR_steer = STM32_TIM1->CCR[1];    // Steer PWM duty cycle
}

#endif
