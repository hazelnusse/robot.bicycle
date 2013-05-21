#include "bitband.h"
#include "constants.h"
#include "motor.h"

namespace hardware {

Motor::Motor(GPIO_TypeDef * gpio_port,
             uint8_t direction_pin,
             uint8_t enable_pin,
             uint8_t fault_pin,
             stm32_tim_t * pwm_timer,
             uint8_t ccr_channel,
             float max_current,          // Amps
             float torque_constant,      // Newton meters per amp
             bool reverse_polarity)
  : gpio_port_(gpio_port),
    pwm_timer_(pwm_timer),
    inv_torque_constant_(1.0f / torque_constant),
    max_current_(max_current),
    max_torque_(max_current * torque_constant),
    direction_pin_(direction_pin),
    enable_pin_(enable_pin),
    fault_pin_(fault_pin),
    ccr_channel_(ccr_channel),
    dir_negative_(reverse_polarity ? 1 : 0),
    dir_positive_(reverse_polarity ? 0 : 1)
{
  set_torque(0.0f);
  enable();
}

Motor::~Motor()
{
  disable();
  set_torque(0.0f);
}

bool Motor::set_torque(float torque)
{
  bool saturated = false;
  if (torque < 0.0f) {
    set_direction_negative();
    torque = -torque;
  } else {
    set_direction_positive();
  }

  if (torque > max_torque_) {
    torque = max_torque_;
    saturated = true;
  }

  set_ccr(current_to_ccr(torque * inv_torque_constant_));
  return saturated;
}

void Motor::disable()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   enable_pin_)) = 1;
}

void Motor::enable()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   enable_pin_)) = 0;
}

uint32_t Motor::current_to_ccr(float current) const
{
  return static_cast<uint32_t>((((constants::PWM_ARR + 1) / max_current_)) * current);
}

void Motor::set_ccr(uint32_t ccr)
{
  pwm_timer_->CCR[ccr_channel_] = ccr;
}

void Motor::set_direction_negative()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   direction_pin_)) = dir_negative_;
}

void Motor::set_direction_positive()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(gpio_port_->ODR)),
                   direction_pin_)) = dir_positive_;
}

} // namespace hardware

