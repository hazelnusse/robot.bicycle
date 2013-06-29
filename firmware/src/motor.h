#ifndef MOTOR_H
#define MOTOR_H

#include <cmath>
#include <cstdint>
#include "hal.h"

namespace hardware {

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
  float get_torque() const { return torque_; }
  float get_current() const { return current_; }
  bool would_saturate(float torque) const { return std::abs(torque) >= max_torque_; }

  void disable();
  void enable();

  bool is_enabled() const;
  bool has_fault() const;
  bool current_direction() const;

 private:
  uint32_t current_to_ccr(float current) const;
  void set_ccr(uint32_t ccr);
  void set_direction_negative();
  void set_direction_positive();

  float current_;
  float torque_;
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

} // namespace hardware

#include "motor-inl.h"

#endif

