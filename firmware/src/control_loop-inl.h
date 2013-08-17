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
ControlLoop::~ControlLoop()
{
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_LEAN_LED)) = 0;
  MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->ODR)), GPIOF_STEER_LED)) = 0;
  instance_ = 0;
}


}

#endif

