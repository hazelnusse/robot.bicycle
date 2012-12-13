#include <cstdlib>
#include <cstdint>
#include <cmath>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "Sample.h"
#include "YawRateController.h"
#include "Constants.h"
#include "textutilities.h"
#include "VectorTable.h"

YawRateController::YawRateController()
  : i_(0), offset_(0), homed_(false), u_(0.0f), r_(0.0f),
    x_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{
  turnOff();
}

void YawRateController::setCurrent(float current)
{
  // Saturate current
  if (current > cf::Current_max_steer) {
    current = cf::Current_max_steer;
  } else if (current < -cf::Current_max_steer) {
    current = -cf::Current_max_steer;
  }

  // save current
  u_ = current;

  // Set direction
  if (current < 0.0f) {
    setCurrentDirNegative();
    current = -current;
  } else {
    setCurrentDirPositive();
  }
  
  PWM_CCR(CurrentToCCR(current));
} // setCurrent

void YawRateController::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled
    if (isEnabled()) {
      turnOff();
      chprintf(chp, "Yaw rate control disabled.\r\n");
    } else {
      turnOn();
      chprintf(chp, "Yaw rate control enabled.\r\n");
    }
  } else if (argc == 1) { // change set point
    RateCommanded(tofloat(argv[0]));
    chprintf(chp, "Yaw rate set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void YawRateController::Update(const Sample & s)
{
  (void) s;

} // Update


void YawRateController::calibrateSteerEncoder(BaseSequentialStream * chp)
{
  // Spawn a thread that waits until 10 rising and falling edges of steer index
  // have occured
  char response[4];
  chprintf(chp, "Is the fork locked? [y/n]\r\n");
  if (shellGetLine(chp, response, sizeof(response)))
    return;

  if ((response[0] == 'n') | (response[0] == 'N'))
    return;

  STM32_TIM3->CNT = 0;  // zero out the steer count when fork is locked
  i_ = 0;               // zero out number of times interrupt has triggered
  for (int i = 0; i < 10; ++i) {
    counts_[i] = dir_[i] = 0;
  }

  chprintf(chp, "Unlock the fork and move it back and forth 10 times");

  VectorTable v;
  irq_vector_t vec40 = v.GetISR(EXTI15_10_IRQn);
  v.SetISR(EXTI15_10_IRQn, CalibrationISR_);
  chSysLock();
  nvicEnableVector(EXTI15_10_IRQn, CORTEX_PRIORITY_MASK(7));
  while (i_ < 10) { } // sit and spin until the fork has moved back and forth 10 times
  chSysUnlock();
  v.SetISR(EXTI15_10_IRQn, vec40);

  for (int i = 0; i < 10; ++i)
    chprintf(chp, "%d, %u\r\n", counts_[i], dir_[i]);
}

void YawRateController::homeFork(BaseSequentialStream * chp)
{
  chprintf(chp, "Move fork past index position.\r\n");
}

// CalibrationISR is called on a rising/falling edge of the steer index
// Need to save the steer encoder count and determine the direction, which can
// be obtained from STM32_TIM3->CNT and STM32_TIM3->CR1[4]  (DIR bit)
CH_IRQ_HANDLER(YawRateController::CalibrationISR_)
{
  CH_IRQ_PROLOGUE();
  YawRateController::Instance().CalibrationISR();
  CH_IRQ_EPILOGUE();
}

void YawRateController::CalibrationISR()
{
  if (i_ < 10) {
    counts_[i_] = STM32_TIM3->CNT;
    dir_[i_] = (STM32_TIM3->SR & (1 << 4)) ? 1 : 0;
    ++i_;
  } else {
    nvicDisableVector(EXTI15_10_IRQn);
  }
}
