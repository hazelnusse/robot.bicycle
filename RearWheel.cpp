#include <cmath>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "Constants.h"
#include "RearWheel.h"
#include "textutilities.h"

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
  if (argc == 0) { // toggle enabled
    if (isEnabled()) {
      turnOff();
      chprintf(chp, "Rear wheel motor disabled.\r\n");
    } else {
      turnOn();
      chprintf(chp, "Rear wheel motor enabled.\r\n");
    }
  } else if (argc == 1) { // change set point
    RateCommanded(tofloat(argv[0]));
    chprintf(chp, "Rear wheel rate set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
} // cmd()

void RearWheel::Update(uint32_t N, uint32_t cnt)
{
  float dtheta = static_cast<int16_t>(cnt - cnt_) * cf::Wheel_rad_per_quad_count;
  float dt = (N - N_) * cf::Rate_Timer_sec_per_count;
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
}


