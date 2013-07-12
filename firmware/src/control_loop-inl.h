#ifndef CONTROL_LOOP_INL_H
#define CONTROL_LOOP_INL_H

#include <cmath>
#include "board.h"
#include "bitband.h"

namespace hardware {

inline
bool ControlLoop::hw_button_enabled() const
{
  return MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)),
                          GPIOF_HW_SWITCH_PIN));
}

inline
void ControlLoop::illuminate_lean_steer(const Sample & s)
{
  if (s.gyro_lean.startup) {
    MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_LEAN_LED)) = 1;
    MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_LED)) = 1;
    return;
  }

  const float mag = std::sqrt(std::pow(s.mpu6050.accelerometer_x, 2.0f)
                            + std::pow(s.mpu6050.accelerometer_y, 2.0f)
                            + std::pow(s.mpu6050.accelerometer_z, 2.0f));
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)),
                   GPIOF_LEAN_LED)) = (std::abs(s.mpu6050.accelerometer_y / mag) < instance_->acc_y_thresh_) ? 1 : 0;
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)),
                   GPIOF_STEER_LED)) = (std::abs(s.encoder.steer) < 1.0f * constants::rad_per_degree) ? 1 : 0;

}

inline
ControlLoop::~ControlLoop()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_LEAN_LED)) = 0;
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_LED)) = 0;
  instance_ = 0;
}


}

#endif

