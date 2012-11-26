#include <cmath>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "Constants.h"
#include "RearWheel.h"


RearWheel::RearWheel()
  : N_(0), N_c_(0), r_(0.0f), u_(0.0f), x_(0.0f), x_c_(0.0f), K_(0.0f),
    P_(0.01f), Q_(2.5e-5f), R_(2.5e-8f)
{
  turnOff();
  PWM_CCR(0);
  setDirNegative();      // negative wheel rotation direction is forward
}

/*! \brief Update state estimate in absence of new measurement.
 *
 * \param[in] t The number of timer counts forward.
 *
 *
 */
//void RearWheel::Predict(uint32_t n)
//{
//  uint32_t h = n - n_;
//  x_ = A(h) * x_ + B(h) * u_;     // State estimate extrapolation
//  P_ = A(h) * P_ * A(h) + Q_;     // Error covariance extrapolation
//  n_ = n;
//}
//
//void RearWheel::PredictAndCorrect(uint32_t n, uint32_t counts)
//{
//  Predict(n);
//  x_ += K_*(cf::Wheel_rad_per_halfquad_count / counts - x_);
//  P_ -= K_*P_;
//  K_ = P_ / R_;
//}

//float RearWheel::A(uint32_t n)
//{
//  return 1.0f - (cf::c_rw / cf::J * cf::Rate_Timer_sec_per_count) * n;
//}
//
//float RearWheel::B(uint32_t n)
//{
//  return (cf::kT_rw / cf::J * cf::Rate_Timer_sec_per_count) * n;
//}


void RearWheel::setCurrent(float current)
{
  // Saturate current
  if (current > cf::Current_max_rw)
    current = cf::Current_max_rw;
  if (current < -cf::Current_max_rw)
    current = -cf::Current_max_rw;

  // save current
  u_ = current;

  // Set direction
  if (current > 0.0f) {
    setDirPositive();
  } else {
    setDirNegative();
    current = -current;                // Make current positive
  }
  
  PWM_CCR(CurrentToCCR(current));
}

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


void RearWheel::Update(uint32_t N_c)
{
  const uint32_t dN = N_c - N_;
  const uint32_t a = A(dN);
  P_ = a * a * P_ + Q(dN);
  x_ = a * x_ + B(dN) * u_;
  N_ = N_c;
} // Update()

void RearWheel::Update(uint32_t N_m, float z)
{
  const uint32_t dN = N_m - N_;
  const uint32_t a = A(dN);
  P_ = a * a * P_ + Q(dN);
  K_ = P_/(P_ + R_);
  P_ *= (1.0f - K_);
  x_ = a * x_ + B(dN) * u_ +  K_*(z - x_);
  N_ = N_m;
} // Update()
