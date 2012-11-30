#include <cmath>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "Constants.h"
#include "RearWheel.h"


RearWheel::RearWheel()
  : u_(0.0f), r_(0.0f), Kp_(1.0f), Ki_(1.0f), Kd_(0.0f),
    e_int_(0.0f), N_(0), cnt_(0)
{
  turnOff();
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
  if (current < 0.0f) {
    setCurrentDirNegative();
    current = -current;
  } else {
    setCurrentDirPositive();
  }
  
  PWM_CCR(CurrentToCCR(current));
} // setCurrent

void RearWheel::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  RearWheel::Instance().cmd(chp, argc, argv);
} // shellcmd()

void RearWheel::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 1) { // change set point
    // format of argument must be a six character string of the form {+-}dd.dd
    float sp = ((argv[0][1] - '0')*10 +
                (argv[0][2] - '0')) +
               ((argv[0][4] - '0')*0.1f +
                (argv[0][5] - '0')*0.01f);
    if (argv[0][0] == '-')
      sp = -sp;
    RateCommanded(sp);
    chprintf(chp, "Set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
} // cmd()

float RearWheel::Update(uint32_t N, uint32_t cnt)
{
  float dtheta = static_cast<int16_t>(cnt - cnt_) * cf::Wheel_rad_per_quad_count;
  float dt = (N - N_) * cf::timer_dt;
  // TODO: try better approximations of derivative, maybe second order
  // derivative filter to get high frequency roll-off
  float z = dtheta / dt;
  float e = RateCommanded() - z;
  float e_int_update = e_int_ + e * dt;
  float I = Kp_ * e + Ki_ * e_int_update;

  setCurrent(I);

  if (u_ == I) { // we haven't saturated
    e_int_ = e_int_update;    // update integral of error
  }

  N_ = N;     // save timer count
  cnt_ = cnt; // save quadrature count
  return z;
}


