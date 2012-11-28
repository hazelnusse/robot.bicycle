#ifndef REARWHEEL_PRIV_H
#define REARWHEEL_PRIV_H

#include "bitband.h"
#include "Constants.h"

inline
void RearWheel::turnOn()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_ENABLE)) = 0x0;
}

inline
void RearWheel::turnOff()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_ENABLE)) = 0x1;
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
void RearWheel::setDirPositive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_DIR)) = 0x0;
}

inline 
void RearWheel::setDirNegative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_RW_DIR)) = 0x1;
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
