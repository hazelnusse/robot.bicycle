#ifndef YAWRATECONTROLLER_PRIV_H
#define YAWRATECONTROLLER_PRIV_H

#include "bitband.h"

inline
void YawRateController::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  YawRateController::Instance().cmd(chp, argc, argv);
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
}

inline
void YawRateController::turnOff()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_ENABLE)) = 0x1;
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
void YawRateController::setDirPositive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_DIR)) = 0x0;
}

inline 
void YawRateController::setDirNegative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_DIR)) = 0x1;
}

inline
bool YawRateController::hasFault()
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)), GPIOF_STEER_FAULT));
}

#endif
