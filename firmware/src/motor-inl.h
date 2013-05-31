#ifndef MOTOR_INL_H
#define MOTOR_INL_H

#include "bitband.h"
#include "constants.h"

namespace hardware {

inline
void Motor::disable()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   enable_pin_)) = 1;
}

inline
void Motor::enable()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   enable_pin_)) = 0;
}

inline
uint32_t Motor::current_to_ccr(float current) const
{
  return static_cast<uint32_t>((((constants::PWM_ARR + 1) / max_current_)) * current);
}

inline
void Motor::set_ccr(uint32_t ccr)
{
  pwm_timer_->CCR[ccr_channel_] = ccr;
}

inline
void Motor::set_direction_negative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   direction_pin_)) = dir_negative_;
}

inline
void Motor::set_direction_positive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   direction_pin_)) = dir_positive_;
}
  
inline
bool Motor::is_enabled() const
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->IDR)), enable_pin_));
}

inline
bool Motor::has_fault() const
{
  return !MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->IDR)), fault_pin_));
}

inline
bool Motor::current_direction() const
{
  return MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->IDR)), direction_pin_));
}

}

#endif

