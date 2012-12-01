#ifndef REARWHEEL_PRIV_H
#define REARWHEEL_PRIV_H

#include "bitband.h"
#include "Constants.h"

inline
void RearWheel::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  RearWheel::Instance().cmd(chp, argc, argv);
} // shellcmd()

inline
void RearWheel::turnOn()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_ENABLE)) = 0x0;
  Reset();
}

inline
void RearWheel::turnOff()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_ENABLE)) = 0x1;
  Reset();
}

inline
void RearWheel::Reset()
{
  e_int_ = 0.0f;
  PWM_CCR(0);
  setCurrentDirNegative();// negative wheel rotation direction is forward
}

inline
bool RearWheel::isEnabled() const
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)), GPIOF_RW_ENABLE));
}

inline
void RearWheel::RateCommanded(float rate)
{
  r_ = rate;
}

inline
float RearWheel::RateCommanded() const
{
  return r_;
}

inline
void RearWheel::ProportionalGain(float kp)
{
  Kp_ = kp;
}

inline
void RearWheel::IntegralGain(float ki)
{
  Ki_ = ki;
}

inline
void RearWheel::DerivativeGain(float kd)
{
  Kd_ = kd;
}

inline
void RearWheel::setCurrentDirPositive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_DIR)) = 0x0;
}

inline 
void RearWheel::setCurrentDirNegative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_DIR)) = 0x1;
}

inline
bool RearWheel::RotationDirection() const
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(STM32_TIM8->SR)), (1 << 4)));
}

inline
bool RearWheel::hasFault()
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)), GPIOF_RW_FAULT));
}

inline
void RearWheel::QuadratureCount(uint32_t count)
{
  STM32_TIM8->CNT = count;
}

inline
uint32_t RearWheel::QuadratureCount() const
{
  return STM32_TIM8->CNT;
}

inline
uint32_t RearWheel::PWM_CCR() const
{
  return STM32_TIM1->CCR[0];
}

inline
void RearWheel::PWM_CCR(uint32_t ccr)
{
  STM32_TIM1->CCR[0] = ccr;
}

inline
uint32_t RearWheel::CurrentToCCR(float current)
{
  return static_cast<uint32_t>((((reg::PWM_ARR + 1) / cf::Current_max_rw)) * current);
}
#endif
