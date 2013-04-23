#ifndef YAWRATECONTROLLER_PRIV_H
#define YAWRATECONTROLLER_PRIV_H

#include "bitband.h"
#include "hal.h"
#include "Constants.h"

inline
void YawRateController::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  YawRateController::Instance().shellcmd(chp, argc, argv);
}

inline
void YawRateController::calibrateSteerEncoder_(BaseSequentialStream *chp, int  __attribute__((unused)) argc, __attribute__((unused)) char *argv[])
{
  YawRateController::Instance().calibrateSteerEncoder(chp);
}

inline
void YawRateController::homeFork_(BaseSequentialStream * chp, int __attribute__((unused)) argc, char __attribute__((unused)) * argv[])
{
  YawRateController::Instance().homeFork(chp);
}

inline
void YawRateController::turnOn()
{
  if (!homed_)
    return;

  Reset();
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_ENABLE)) = 0x0;
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
  estimation_triggered_ = control_triggered_ = false; 
  r_ = u_ = x_[0] = x_[1] = x_[2] = x_[3] = x_pi_ = 0.0f;
  SystemTime_prev_ = 0;
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
float YawRateController::EstimationThreshold() const
{
  return estimator_theta_R_dot_threshold_;
}

inline
void YawRateController::EstimationThreshold(float thresh)
{
  estimator_theta_R_dot_threshold_ = thresh;
}

inline
float YawRateController::ControlThreshold() const
{
  return controller_theta_R_dot_threshold_;
}

inline
void YawRateController::ControlThreshold(float thresh)
{
  controller_theta_R_dot_threshold_ = thresh;
}

inline
bool YawRateController::isPIEnabled() const
{
  return PI_enabled_;
}

inline
void YawRateController::EnablePI()
{
  PI_enabled_ = true;
}

inline
void YawRateController::DisablePI()
{
  PI_enabled_ = false;
}

inline
void YawRateController::setCurrentDirPositive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_DIR)) = 0x1;
}

inline 
void YawRateController::setCurrentDirNegative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_DIR)) = 0x0;
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

inline
bool YawRateController::CurrentDir() const
{
  return MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)), GPIOF_STEER_DIR));
}

inline
bool YawRateController::RotationDir() const
{
  return MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(STM32_TIM3->SR)), (1 << 4)));
}

inline
void YawRateController::SteerOffset(int32_t N)
{
  offset_ = N;
}

inline
int32_t YawRateController::SteerOffset() const
{
  return offset_;
}

inline
void YawRateController::saveEstimatorState(Sample & s) const
{
  s.has_estimate = true;
  s.estimate.phi = x_[0];
  s.estimate.delta = x_[1];
  s.estimate.phi_dot = x_[2];
  s.estimate.delta_dot = x_[3];
  s.estimate.theta_R_dot_lower = ar_[0]->theta_R_dot;
  s.estimate.theta_R_dot_lower = ar_[1]->theta_R_dot;
}

inline
float YawRateController::CurrentCommanded() const
{
  return u_;
}
  
#endif

