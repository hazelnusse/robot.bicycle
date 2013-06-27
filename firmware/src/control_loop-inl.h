#ifndef CONTROL_LOOP_INL_H
#define CONTROL_LOOP_INL_H

#include "board.h"
#include "bitband.h"

namespace hardware {

inline
bool ControlLoop::hw_button_enabled()
{
  return MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(GPIOF->IDR)),
                          GPIOF_HW_SWITCH_PIN));
}


}

#endif

