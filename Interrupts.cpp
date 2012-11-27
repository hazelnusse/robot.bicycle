#include <cstdint>

#include "ch.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// VectorE0 is called on a rising/falling edge of the steer index
// Need to save the steer encoder count and determine the direction, which can
// be obtained from STM32_TIM3->CNT and STM32_TIM3->CR1[4]  (DIR bit)
CH_IRQ_HANDLER(EXTI15_10_IRQHandler)
{
  CH_IRQ_PROLOGUE();

  CH_IRQ_EPILOGUE();
}

#ifdef __cplusplus
}
#endif
