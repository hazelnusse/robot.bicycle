#include <cmath>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "Constants.h"
#include "RearWheel.h"


RearWheel::RearWheel()
{
  turnOff();
  PWM_CCR(0);
  setDirNegative();      // negative wheel rotation direction is forward
} // RearWheel()

void RearWheel::setCurrent(float current)
{
  // Saturate current
  if (current > cf::Current_max_rw) {
    current = cf::Current_max_rw;
  } else if (current < -cf::Current_max_rw) {
    current = -cf::Current_max_rw;
  }

  // save current
  u_ = current;

  // Set direction
  if (current > 0.0f) {
    setDirPositive();
    PWM_CCR(CurrentToCCR(current));
  } else {
    setDirNegative();
    PWM_CCR(CurrentToCCR(-current));
  }
} // setCurrent

void RearWheel::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  RearWheel::Instance().cmd(chp, argc, argv);
} // shellcmd()

void RearWheel::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      turnOff();
      chprintf(chp, "Speed control disabled.\r\n");
    } else {
      turnOn();
      chprintf(chp, "Speed control enabled.\r\n");
    }
  } else if (argc == 1) { // change set point
//    float sp = 0.02f*((argv[0][0] - '0')*100 +
//                      (argv[0][1] - '0')*10  +
//                      (argv[0][2] - '0'));
    float sp = -((argv[0][0] - '0')*100 +
                 (argv[0][1] - '0')*10  +
                 (argv[0][2] - '0'))*(cf::Current_max_rw/1000.0f);
//    RateCommanded(sp);
    setCurrent(sp);
    chprintf(chp, "Set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
} // cmd()
