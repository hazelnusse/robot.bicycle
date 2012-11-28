#include <cmath>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "Constants.h"
#include "RearWheel.h"


RearWheel::RearWheel()
  : u_(0.0f), r_(0.0f), Kp_(1.0f), Ki_(1.0f), Kd_(0.0f), e_int_(0.0f)
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
    // format of argument must be a six character string of the form {+-}dd.dd
    float sp = ((argv[0][1] - '0')*10 +
                (argv[0][2] - '0')) +
               ((argv[0][4] - '0')*0.1f +
                (argv[0][5] - '0')*0.01f);
    if (argv[0][0] == '-')
      sp = -sp;

    RateCommanded(sp);
//    float sp = -((argv[0][0] - '0')*100 +
//                 (argv[0][1] - '0')*10  +
//                 (argv[0][2] - '0'))*(cf::Current_max_rw/1000.0f);
//    setCurrent(sp);
    chprintf(chp, "Set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
} // cmd()

void RearWheel::Update(uint32_t N, uint32_t cnt)
{
  float z = (cnt - cnt_) * cf::Wheel_rad_per_quad_count,
        e = r_ - z,
        dt = (N - N_) * cf::timer_dt,
        e_int_update = e_int_ + e * dt,
        I = Kp_ * e + Ki_ * e_int_update;

  setCurrent(I);

  if (u_ == I) { // we haven't saturated
    e_int_ = e_int_update;    // update integral of error
  }

  N_ = N;     // save timer count
  cnt_ = cnt; // save quadrature count
}


