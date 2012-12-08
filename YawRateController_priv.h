#ifndef YAWRATECONTROLLER_PRIV_H
#define YAWRATECONTROLLER_PRIV_H

#include "bitband.h"
#include "Constants.h"

inline
void YawRateController::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  YawRateController::Instance().shellcmd(chp, argc, argv);
}

inline
void YawRateController::calibrateSteerEncoder(BaseSequentialStream *chp, int  __attribute__((unused)) argc, __attribute__((unused)) char *argv[])
{
  YawRateController::Instance().calibrateSteerEncoder(chp);
}

inline
void YawRateController::homeFork(BaseSequentialStream * chp, int __attribute__((unused)) argc, char __attribute__((unused)) * argv[])
{
  YawRateController::Instance().homeFork(chp);
}

inline
void YawRateController::turnOn()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_ENABLE)) = 0x0;
  Reset();
}

inline
void YawRateController::turnOff()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_ENABLE)) = 0x1;
  Reset();
}

inline
void YawRateController::Reset()
{
  x_[0] = x_[1] = x_[2] = x_[3] = x_[4] = 0.0f;
  PWM_CCR(0);
  setCurrentDirNegative();// negative rotation direction is to the left
}

inline
bool YawRateController::isEnabled() const
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)), GPIOF_STEER_ENABLE));
}

inline
void YawRateController::RateCommanded(float yaw_rate)
{
  r_ = yaw_rate;
}

inline
float YawRateController::RateCommanded() const
{
  return r_;
}

inline
void YawRateController::setCurrentDirPositive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_DIR)) = 0x0;
}

inline 
void YawRateController::setCurrentDirNegative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_DIR)) = 0x1;
}

inline
bool YawRateController::hasFault()
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)), GPIOF_STEER_FAULT));
}

inline
uint32_t YawRateController::PWM_CCR() const
{
  return STM32_TIM1->CCR[1];
}

inline
void YawRateController::PWM_CCR(uint32_t ccr)
{
  STM32_TIM1->CCR[1] = ccr;
}

inline
uint32_t YawRateController::CurrentToCCR(float current)
{
  return static_cast<uint32_t>((((reg::PWM_ARR + 1) / cf::Current_max_steer)) * current);
}

#endif
