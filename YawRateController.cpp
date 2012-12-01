#include <cstdlib>
#include <cstdint>
#include <cmath>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "Sample.h"
#include "YawRateController.h"
#include "Constants.h"
#include "textutilities.h"

YawRateController::YawRateController()
  : offset_(0), homed_(false), u_(0.0f), r_(0.0f),
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

void YawRateController::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 1) { // Change reference yaw rate
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
  chprintf(chp, "Ensure fork is locked at bootup then move fork back and forth.\r\n");
}

void YawRateController::homeFork(BaseSequentialStream * chp)
{
  chprintf(chp, "Move fork past index position.\r\n");
}
