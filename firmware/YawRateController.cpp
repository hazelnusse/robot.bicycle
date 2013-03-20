#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "cgains.h"
#include "RearWheel.h"
#include "Sample.h"
#include "YawRateController.h"
#include "Constants.h"
#include "textutilities.h"
#include "VectorTable.h"
#include "imu_calibration.h"
  
#define EDGES 16
int16_t counts_[EDGES];
volatile uint8_t i_;

YawRateController::YawRateController()
  : offset_(-402), homed_(false), u_(0.0f), r_(0.0f),
    x_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
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

void YawRateController::Update(const Sample & s)
{
  const float theta_R_dot = RearWheel::Instance().RateEstimate();
  const float input[3] = {r_,
                          s.SteerAngle * cf::Steer_rad_per_quad_count,
                          imu_calibration::phi_dot(s)};

  // Steer torque at previous time step
  // NOTE: this gets used, and updated, in call to state_and_output_update
  float steer_torque = u_ / cf::kT_steer_inv;

  // Perform controller state update as long as steer isn't too big
  if ((std::fabs(input[1]) < (45.0f * cf::rad_per_degree)) &&
    cg::state_and_output_update(theta_R_dot, input, x_, steer_torque)) {
    u_ = steer_torque * cf::kT_steer_inv;
  } else {
    u_ = 0.0f;
  }

  setCurrent(u_);
} // Update

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
