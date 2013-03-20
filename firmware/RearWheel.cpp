#include <cmath>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "Constants.h"
#include "RearWheel.h"
#include "textutilities.h"

RearWheel::RearWheel()
  : u_(0.0f), r_(0.0f), Kp_(1.0f), Ki_(1.0f), e_int_(0.0f), z_(0.0f),
    SystemTime_prev_(0), RearWheelAngle_prev_(0)
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
  if (argc == 0) {
      turnOff();
      chprintf(chp, "Rear wheel rate control disabled.\r\n");
  } else if (argc == 1) {
      RateCommanded(tofloat(argv[0]));
      turnOn();
      chprintf(chp, "Rear wheel rate control enabled and set.\r\n");
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
} // cmd()

void RearWheel::Update(const Sample & s)
{
  // TODO: try better approximations of derivative, maybe second order
  // derivative filter to get high frequency roll-off
  const float dtheta = static_cast<int16_t>(s.RearWheelAngle - RearWheelAngle_prev_) * cf::Wheel_rad_per_quad_count;
  const float dt = (s.SystemTime - SystemTime_prev_) * cf::Rate_Timer_sec_per_count;
  z_ = dtheta / dt;

  const float e = RateCommanded() - z_;
  const float e_int_update = e_int_ + e * dt;
  const float I = Kp_ * e + Ki_ * e_int_update;

  setCurrent(I);

  if (u_ == I)                // we haven't saturated
    e_int_ = e_int_update;    // update integral of error

  SystemTime_prev_ = s.SystemTime;
  RearWheelAngle_prev_ = s.RearWheelAngle;
}


