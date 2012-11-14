#include <cstdlib>
#include <cstdint>
#include <cmath>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "RearWheel.h"
#include "Sample.h"
#include "SpeedController.h"
#include "Constants.h"

SpeedController::SpeedController()
  : SetPoint_(0.0f),
    A{1.0f, 0.534089953667745f},
    B{0.125f, 1.178973669432010f},
    C{0.312399275353728f, 1.178973669432010f},
    D(5.746075010421994f),
    x{0.0f, 0.0f},
    u(0.0f),
    Enabled_(false)
{
  setEnabled(false);
} // SpeedController()

void SpeedController::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  SpeedController::Instance().cmd(chp, argc, argv);
} // shellcmd()

void SpeedController::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      setEnabled(false);
      chprintf(chp, "Speed control disabled.\r\n");
    } else {
      setEnabled(true);
      chprintf(chp, "Speed control enabled.\r\n");
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
} // cmd()

void SpeedController::Update()
{
  // Set input to PID controller
  u = SetPoint() - RearWheel::Instance().SpeedEstimate();

  // Compute output of PID Controller
  float current = C[0]*x[0] + C[1]*x[1] + D*u;
  
  // Apply the calculated current command
  RearWheel::Instance().SetCurrent(current);

  // Update state of PID controller
  x[0] = A[0]*x[0] + B[0]*u;
  x[1] = A[1]*x[1] + B[1]*u;
} // Update()

void SpeedController::setEnabled(bool state)
{
  x[0] = x[1] = u = 0.0f;
  Enabled_ = state;
} // setEnabled()
