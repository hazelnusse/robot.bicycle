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
  : current_{0.0f},
    torque_{0.0f},
    gpio_port_{gpio_port},
    pwm_timer_{pwm_timer},
    inv_torque_constant_{1.0f / torque_constant},
    max_current_{max_current},
    max_torque_{max_current * torque_constant},
    direction_pin_{direction_pin},
    enable_pin_{enable_pin},
    fault_pin_{fault_pin},
    ccr_channel_{ccr_channel},
    dir_negative_{reverse_polarity ? uint8_t(1) : uint8_t(0)},
    dir_positive_{reverse_polarity ? uint8_t(0) : uint8_t(1)}
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
  bool torque_set = true;
  if (torque < 0.0f)
    set_direction_negative();
  else
    set_direction_positive();

  if (std::abs(torque) > max_torque_) {
    if (torque > 0.0f)
      torque = max_torque_;
    else
      torque = -max_torque_;
    torque_set = false;
  }

  torque_ = torque;
  current_ = torque * inv_torque_constant_;
  set_ccr(current_to_ccr(std::abs(current_)));
  return torque_set;
}

} // namespace hardware

