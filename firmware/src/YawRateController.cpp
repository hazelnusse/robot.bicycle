#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "RearWheel.h"
#include "Sample.pb.h"
#include "MPU6050.h"
#include "YawRateController.h"
#include "Constants.h"
#include "textutilities.h"
#include "VectorTable.h"
  
#define EDGES 16
int16_t counts_[EDGES];
volatile uint8_t i_;

YawRateController::YawRateController()
  : offset_(-402),
    homed_(false),
    estimation_triggered_(false),
    control_triggered_(false),
    PI_enabled_(false),
    u_(0.0f), r_(0.0f), x_{0.0f, 0.0f, 0.0f, 0.0f},
    estimator_theta_R_dot_threshold_(-2.0f),
    controller_theta_R_dot_threshold_(-2.5f),
    ar_{0, 0},
    alpha_(0.0f),
    x_pi_(0.0f),
    SystemTime_prev_(0)
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

void YawRateController::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {
      turnOff();
      chprintf(chp, "Yaw rate control disabled.\r\n");
  } else if (argc == 1) {
      RateCommanded(tofloat(argv[0]));
      turnOn();
      chprintf(chp, "Yaw rate control enabled and set.\r\n");
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void YawRateController::Update(Sample & s)
{
  const float theta_R_dot = RearWheel::Instance().RateEstimate();
  // Estimator inputs:
  //   -- Steer torque applied at previous step
  //   -- Steer angle from most recent measurement
  //   -- Roll rate from most recent measurement
  const float input[3] = {u_ * cf::kT_steer,
                          s.encoder.Steer,
                          MPU6050::phi_dot(s)};

  u_ = 0.0f;  // set current to zero
  if (theta_R_dot < estimator_theta_R_dot_threshold_ || estimation_triggered_) {
    estimation_triggered_ = true;
    // State update occurs only if we have hit threshold speed
    if (state_estimate_update(theta_R_dot, input) &&
        (std::fabs(input[1]) < (45.0f * cf::rad_per_degree))) {

      saveEstimatorState(s);
      // And if the speed is inside the speed range for which gains are computed
      // Control law is updated only if steer is < 45 degrees,
      if (theta_R_dot < controller_theta_R_dot_threshold_ || control_triggered_) {
        // And if we have hit the threshold control speed
        control_triggered_ = true;
        if (PI_enabled_) {
          // Compute PI state update
          const float e = RateCommanded() - MPU6050::psi_dot(s);
          const float dt = static_cast<uint32_t>(s.SystemTime - SystemTime_prev_) * cf::Rate_Timer_sec_per_count;
          float Kp, Ki;
          interpolate_PI_gains(Kp, Ki); // interpolate Kp and Ki from lookup table
          const float r_pi = Ki * dt * x_pi_ + (Ki * dt + Kp) * e;
          u_ = (control_output_update() + r_pi) * cf::kT_steer_inv;
          // Update integral term only if we don't saturate the control
          if (std::fabs(u_) < cf::Current_max_steer) {
            x_pi_ += e;
          }
          s.has_yaw_rate_pi = true;
          s.yaw_rate_pi.e = e;
          s.yaw_rate_pi.Kp = Kp;
          s.yaw_rate_pi.Ki = Ki;
          s.yaw_rate_pi.x = x_pi_;
        } else {
          u_ = control_output_update() * cf::kT_steer_inv;
        }
      }
    }
  }
  setCurrent(u_);  // sets current to 0.0f or that commanded by control law
} // Update

void YawRateController::interpolate_PI_gains(float & Kp, float & Ki) const
{
  Kp = (ar_[1]->Kp - ar_[0]->Kp) * alpha_ + ar_[0]->Kp;
  Ki = (ar_[1]->Ki - ar_[0]->Ki) * alpha_ + ar_[0]->Ki;
}

void YawRateController::calibrateSteerEncoder(BaseSequentialStream * chp)
{
  float mean[2];
  for (int j = 0; j < 2; j++) {
    char response[4];
    chprintf(chp, "Is the fork locked? [y/n]\r\n");
    if (shellGetLine(chp, response, sizeof(response))) return;

    if ((response[0] == 'n') | (response[0] == 'N')) return;

    STM32_TIM3->CNT = 0;  // zero out the steer count when fork is locked
    i_ = 0;               // zero out number of times interrupt has triggered
    for (uint8_t i = 0; i < EDGES; ++i) {
      counts_[i] = 0;
    }

    chprintf(chp, "Unlock the fork and move it back and forth %u times\r\n", EDGES/4);

    VectorTable v;
    irq_vector_t vec40 = v.GetISR(EXTI15_10_IRQn);
    v.SetISR(EXTI15_10_IRQn, CalibrationISR_);
    uint16_t tmp = SYSCFG->EXTICR[2];
    SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI11_PF;
    nvicEnableVector(EXTI15_10_IRQn, CORTEX_PRIORITY_MASK(7));
    EXTI->IMR = (1 << 11);
    EXTI->RTSR = (1 << 11);
    EXTI->FTSR = (1 << 11);

    while (i_ < EDGES) { } // sit and spin until the fork has moved back and forth 10 times

    EXTI->IMR = 0;
    EXTI->RTSR = 0;
    EXTI->FTSR = 0;
    SYSCFG->EXTICR[2] = tmp;
    nvicDisableVector(EXTI15_10_IRQn);
    v.SetISR(EXTI15_10_IRQn, vec40);

    float sum = 0.0f;
    for (uint8_t i = 0; i < EDGES; ++i) {
      chprintf(chp, "%d\r\n", counts_[i]);
      sum += counts_[i];
    }

    mean[j] = sum / EDGES;
    chprintf(chp, "Offset mean: %f\r\n", mean[j]);

    if (j == 0) {
      chprintf(chp, "Reverse the fork fixture and lock the fork straight.\r\n");
    }
  }
  float mean_both_runs = (mean[0] + mean[1])/2.0f;
  chprintf(chp, "Mean: %f\r\n", mean_both_runs);
  int32_t n = round(mean_both_runs);
  chprintf(chp, "Steer offset set to (as integer): %d\r\n", n);
  SteerOffset(n);
}

void YawRateController::homeFork(BaseSequentialStream * chp)
{
  chprintf(chp, "Move fork past index position.\r\n");
  VectorTable v;
  irq_vector_t vec40 = v.GetISR(EXTI15_10_IRQn);
  v.SetISR(EXTI15_10_IRQn, homeISR_);
  uint16_t tmp = SYSCFG->EXTICR[2];
  SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI11_PF;
  nvicEnableVector(EXTI15_10_IRQn, CORTEX_PRIORITY_MASK(7));
  EXTI->IMR = (1 << 11);
  EXTI->RTSR = (1 << 11);
  EXTI->FTSR = (1 << 11);

  i_ = 1;
  while (i_) { } // sit and spin until homeISR sets i_ to zero

  EXTI->IMR = 0;
  EXTI->RTSR = 0;
  EXTI->FTSR = 0;
  SYSCFG->EXTICR[2] = tmp;
  nvicDisableVector(EXTI15_10_IRQn);
  v.SetISR(EXTI15_10_IRQn, vec40);
  
  homed_ = true;
  chprintf(chp, "Fork has been successfully homed.\r\n");
}

// CalibrationISR is called on a rising/falling edge of the steer index
// Need to save the steer encoder count and determine the direction, which can
// be obtained from STM32_TIM3->CNT and STM32_TIM3->CR1[4]  (DIR bit)
CH_IRQ_HANDLER(YawRateController::CalibrationISR_)
{
  EXTI->PR = (1 << 11);   // clear the pending bit.
  if (i_ < EDGES) {
    counts_[i_] = STM32_TIM3->CNT;
    ++i_;
  } else {
    EXTI->IMR = 0;
  }
}

CH_IRQ_HANDLER(YawRateController::homeISR_)
{
  STM32_TIM3->CNT = YawRateController::Instance().SteerOffset();
  EXTI->PR = (1 << 11);   // clear the pending bit.
  i_ = 0;
}

void YawRateController::setEstimationThreshold(BaseSequentialStream * chp, int argc, char * argv[])
{
  YawRateController & yr = YawRateController::Instance();
  if (argc != 1) {
    chprintf(chp, "Invalid usage.\r\n");
  } else {
    float thresh = tofloat(argv[0]);
    float thresh_old = yr.EstimationThreshold();
    yr.EstimationThreshold(tofloat(argv[0]));
    chprintf(chp, "Estimation threshold changed from %f to %f\r\n", thresh_old, thresh);
  }
}

void YawRateController::setControlThreshold(BaseSequentialStream * chp, int argc, char * argv[])
{
  YawRateController & yr = YawRateController::Instance();
  if (argc != 1) {
    chprintf(chp, "Invalid usage.\r\n");
  } else {
    float thresh = tofloat(argv[0]);
    float thresh_old = yr.ControlThreshold();
    yr.ControlThreshold(tofloat(argv[0]));
    chprintf(chp, "Control threshold changed from %f to %f\r\n", thresh_old, thresh);
  }

}

void YawRateController::togglePI(BaseSequentialStream *, int, char **)
{
  YawRateController & yr = YawRateController::Instance();
  if (yr.isPIEnabled())
    yr.DisablePI();
  else
    yr.EnablePI();
}

