#include <cstdint>
#include "interrupts.h"
#include "encodertimers.h"
#include "ch.h"
#include "hal.h"

extern EncoderTimers timers;

void VectorB8(void)
{
  uint16_t sr = STM32_TIM4->SR;
  uint16_t dir = GPIOD->IDR;    // get register
  if (sr & 0x0002) { // IC1
    timers.CCR[0] = STM32_TIM4->CCR[0];
    timers.dir[0] = dir & (1 << 12);
  }
  if (sr & 0x0004) { // IC2
    timers.CCR[1] = STM32_TIM4->CCR[1];
    timers.dir[1] = dir & (1 << 13);
  }
  if (sr & 0x0008) { // IC3
    timers.CCR[2] = STM32_TIM4->CCR[2];
    timers.dir[2] = dir & (1 << 14);
  }
  if (sr & 0x0001) { // UE (overflow/underflow)

  }

}
