#include <cmath>

#include "ch.h"
#include "hal.h"

#include "Constants.h"
#include "RearWheel.h"


RearWheel::RearWheel()
  : x_(0.0f), u_(0.0f), K_(0.0f), P_(0.0f), Q_(1.0e-6f), R_(1.0e-6f), n_(0)
{

}

/*! \brief Update state estimate in absence of new measurement.
 *
 * \param[in] t The number of timer counts forward.
 *
 *
 */
void RearWheel::Predict(uint32_t n)
{
  uint32_t h = n - n_;
  x_ = A(h) * x_ + B(h) * u_;     // State estimate extrapolation
  P_ = A(h) * P_ * A(h) + Q_;     // Error covariance extrapolation
  n_ = n;
}

void RearWheel::PredictAndCorrect(uint32_t n, uint32_t counts)
{
  Predict(n);
  x_ += K_*(cf::Wheel_rad_per_halfquad_count / counts - x_);
  P_ -= K_*P_;
  K_ = P_ / R_;
}

float RearWheel::A(uint32_t n)
{
  return 1.0f - (cf::c_rw / cf::J * cf::Rate_Timer_sec_per_count) * n;
}

float RearWheel::B(uint32_t n)
{
  return (cf::kT_rw / cf::J * cf::Rate_Timer_sec_per_count) * n;
}


void RearWheel::SetCurrent(float current)
{
  u_ = current;

  // Set direction
  if (current > 0.0f) {
    palClearPad(GPIOF, GPIOF_RW_DIR);  // set to reverse direction
  } else {
    palSetPad(GPIOF, GPIOF_RW_DIR);    // set to forward direction
    current = -current;                // Make current positive
  }
  
  // Saturate current at max continuous current of Copley drive
  if (current > cf::Current_max_rw)
    current = cf::Current_max_rw;
  
  // Convert from current to PWM duty cycle;
  float duty = current / cf::Current_max_rw;   // float in range of [0.0, 1.0]

  // Convert duty cycle to an uint32_t in range of [0, TIM1->ARR + 1] and set
  // it in TIM1
  STM32_TIM1->CCR[0] = static_cast<uint32_t>(static_cast<float>(STM32_TIM1->ARR + 1) * duty);
}

void RearWheel::setMotorEnabled(bool state)
{
  STM32_TIM1->CCR[0] = 0; // set 0% duty cycle
  if (state) {
    palClearPad(GPIOF, GPIOF_RW_ENABLE);  // enable
  } else {
    palSetPad(GPIOF, GPIOF_RW_ENABLE);    // disable
  }
  x_ = u_ = P_ = 0.0f;    // zero the state estimate, input, and estimate covariance
}
