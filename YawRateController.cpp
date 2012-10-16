#include <cstdlib>
#include <cstdint>
#include <cmath>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "Sample.h"
#include "YawRateController.h"
#include "Constants.h"

YawRateController::YawRateController()
  : SetPoint_(0.0f), Enabled_(false)
{
  setEnabled(false);
}

void YawRateController::setEnabled(bool state)
{
  STM32_TIM1->CCR[1] = 0; // 0% duty cycle

  if (state) {
    palClearPad(GPIOF, GPIOF_STEER_ENABLE);  // enable
  } else {
    palSetPad(GPIOF, GPIOF_STEER_ENABLE);  //   disable
  }

  Enabled_ = state;
} // setEnabled()

void YawRateController::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  YawRateController::Instance().cmd(chp, argc, argv);
}

void YawRateController::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      setEnabled(false);
      chprintf(chp, "Yaw rate control disabled.\r\n");
    } else {
      setEnabled(true);
      chprintf(chp, "Yaw rate control enabled.\r\n");
    }
  } else if (argc == 1) { // change set point
    float sp = 0.02f*((argv[0][0] - '0')*100 +
                     (argv[0][1] - '0')*10  +
                     (argv[0][2] - '0'));
    SetPoint(sp);
    chprintf(chp, "Set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void YawRateController::Update(Sample & s)
{
} // Update
