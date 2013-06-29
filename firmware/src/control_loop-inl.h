#ifndef CONTROL_LOOP_INL_H
#define CONTROL_LOOP_INL_H

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
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)),
                   GPIOF_LEAN_LED)) = (std::abs(s.estimate.lean) < 1.0f * constants::rad_per_degree) ? 1 : 0;
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)),
                   GPIOF_STEER_LED)) = (std::abs(s.encoder.steer) < 1.0f * constants::rad_per_degree) ? 1 : 0;

}

inline
ControlLoop::~ControlLoop()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_LEAN_LED)) = 0;
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_LED)) = 0;
}


}

#endif

