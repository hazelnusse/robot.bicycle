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
  if (argc == 0) { // toggle enabled
    if (isEnabled()) {
      turnOff();
      chprintf(chp, "Yaw rate control disabled.\r\n");
    } else {
      turnOn();
      chprintf(chp, "Yaw rate control enabled.\r\n");
    }
  } else if (argc == 1) { // change set point
    RateCommanded(tofloat(argv[0]));
    chprintf(chp, "Yaw rate set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void YawRateController::Update(const Sample & s)
{
  float wx = s.MPU6050[4]*cf::Gyroscope_sensitivity - imu_calibration::wx;
  float wy = s.MPU6050[5]*cf::Gyroscope_sensitivity - imu_calibration::wy;
  float wz = s.MPU6050[6]*cf::Gyroscope_sensitivity - imu_calibration::wz;
  float delta = s.SteerAngle * cf::Steer_rad_per_count;
  float phi_dot = imu_calibration::dcm[0] * wx +
                  imu_calibration::dcm[3] * wy +
                  imu_calibration::dcm[5] * wz;
//  cg::ControllerGains * gains = cg::gains[RearWheel::Instance().RateEstimate()];
  auto gains = cg::gains.lu_bounds(RearWheel::Instance().RateEstimate());

  if (!gains.first || !gains.second) { // outside speed range for which gains have been scheduled
    setCurrent(0.0f);
    return;
  }

  cg::ControllerGains * ar[2] = {gains.first, gains.second};

  // controller inputs are:
  //   - yaw rate reference,
  //   - steer angle measurement
  //   - roll rate measurement
  float yrc_input[3] = {r_, delta, phi_dot};
 
  // Compute output control signal
  float torque = 0.0f;
  for (int k = 0; k < 2; ++k) {
    for (int i = 0; i < 5; ++i) {
      torque += ar[k]->C[i] * x_[i];
    }
  } // for k
  setCurrent(torque * cf::kT_steer / 2.0f);

  // Compute state update
  float x_new[cg::a_rows] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for (int k = 0; k < 2; ++k) {
    for (int i = 0; i < cg::a_rows; ++i) {   // rows of state equations
      for (int j = 0; j < cg::a_cols; ++j) {    // columns of A * rows of x
        x_new[i] += ar[k]->A[cg::a_cols*i + j] * x_[j];
      } // for j
      for (int j = 0; j < cg::b_cols; ++j) {    // columns of B * rows of controller input
        x_new[i] += ar[k]->B[cg::b_cols*i + j] * yrc_input[j];
      } // for j
    } // for i
  } // for k
  // Average of the two gains
  for (int i = 0; i < 5; ++i) {
    x_new[i] /= 2.0f;
  }
  std::copy(x_new, x_new + 5, x_);
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
