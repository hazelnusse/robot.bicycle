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
  : offset_(0), homed_(false), u_(0.0f), r_(0.0f),
    x_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{
  turnOff();
}

void YawRateController::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      turnOff();
      chprintf(chp, "Yaw rate control disabled.\r\n");
    } else {
      turnOn();
      chprintf(chp, "Yaw rate control enabled.\r\n");
    }
  } else if (argc == 1) { // Change reference yaw rate
    float sp = 0.02f*((argv[0][0] - '0')*100 +
                     (argv[0][1] - '0')*10  +
                     (argv[0][2] - '0'));
    RateCommanded(sp);
    chprintf(chp, "Set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void YawRateController::Update(Sample & s)
{

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
