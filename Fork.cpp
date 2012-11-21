#include "hal.h"
#include "Fork.h"

Fork::Fork()
  : offset_(0)
{

}
void Fork::calibrate(BaseSequentialStream *chp, int  __attribute__((unused)) argc, __attribute__((unused)) char *argv[])
{
  Fork::Instance().calibrateEncoder(chp);
}

void Fork::calibrateEncoder(BaseSequentialStream * chp)
{
  // Spawn a thread that waits until 10 rising and falling edges of steer index
  // have occured
  chprintf(chp, "Ensure fork is locked at bootup then move fork back and forth.\r\n");
  return;
}

void Fork::home(BaseSequentialStream * chp, int __attribute__((unused)) argc, char __attribute__((unused)) * argv[])
{
  Fork::Instance().homeFork(chp);
}

void Fork::homeFork(BaseSequentialStream * chp)
{

}

void Fork::setMotorEnabled(bool state)
{
  STM32_TIM1->CCR[1] = 0; // set 0% duty cycle
  if (state) {
    palClearPad(GPIOF, GPIOF_STEER_ENABLE);  // enable
  } else {
    palSetPad(GPIOF, GPIOF_STEER_ENABLE);    // disable
  }
}
