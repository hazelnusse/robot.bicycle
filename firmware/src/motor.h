#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>
#include "hal.h"

class MotorStatus {
 public:
  bool direction;
  bool enable;
  bool fault;
};

class Motor {
 public:
  Motor(GPIO_TypeDef * gpio_port,
        uint8_t direction_pin,
        uint8_t enable_pin,
        uint8_t fault_pin,
        stm32_tim_t * pwm_timer,
        uint8_t ccr_channel,
        float max_current,          // Amps
        float torque_constant,      // Newton meters per amp
        bool reverse_polarity = false);
  ~Motor();

  bool set_torque(float torque);

  void disable();
  void enable();

 private:
  uint32_t current_to_ccr(float current) const;
  void set_ccr(uint32_t ccr);
  void set_direction_negative();
  void set_direction_positive();

  GPIO_TypeDef * const gpio_port_;
  stm32_tim_t * const pwm_timer_;
  const float inv_torque_constant_; // Amps per Newton meter
  const float max_current_;
  const float max_torque_;
  const uint8_t direction_pin_;
  const uint8_t enable_pin_;
  const uint8_t fault_pin_;
  const uint8_t ccr_channel_;
  const uint8_t dir_negative_;
  const uint8_t dir_positive_;
};

#include "motor_priv.h"

#endif

