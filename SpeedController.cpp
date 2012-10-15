#include <cstdlib>
#include <cstdint>
#include <cmath>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "Sample.h"
#include "SpeedController.h"
#include "Constants.h"

SpeedController::SpeedController()
  : SetPoint_(0.0f),
    A({1.0f, 0.534089953667745f}),
    B({0.125f, 1.178973669432010f}),
    C({0.312399275353728f, 1.178973669432010f}),
    D(5.746075010421994f),
    x({0.0f, 0.0f}),
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

void SpeedController::Update(Sample & s)
{
  // PeriodCounts has units of TIM5 clock ticks per rear wheel encoder cycle.
  // TIM5 clock is at 4.0MHz, one cycle of rear wheel is 2.0*M_PI/200 rad.

  uint32_t PeriodCounts = s.RearWheelRate;
  bool dir_bit = PeriodCounts & ~(1 << 31); // get the high bit
  PeriodCounts &= ~(1 << 31);   // clear the high bit which indicates direction
  float vel = 0.0f;
  
  // Determine velocity
  if (PeriodCounts) {
    vel = cf::Wheel_rad_counts_per_sec / PeriodCounts;
    if (!dir_bit)
      vel *= -1.0f;
  }

  // Set input to PID controller
  u = SetPoint() - vel;

  // Compute output of PID Controller
  float current = C[0]*x[0] + C[1]*x[1] + D*u;

  // Update state of PID controller
  x[0] = A[0]*x[0] + B[0]*u;
  x[1] = A[1]*x[1] + B[1]*u;
  
  // Set direction
  if (current > 0.0f) {
    palSetPad(GPIOF, 6);    // set to forward direction
  } else {
    palClearPad(GPIOF, 6);  // set to reverse direction
  }

  // Make current positive
  current = std::fabs(current);

  // Saturate current at max continuous current of Copley drive
  if (current > cf::Current_max_rw)
    current = cf::Current_max_rw;

  // Convert from current to PWM duty cycle;
  float duty = current / cf::Current_max_rw;   // float in range of [0.0, 1.0]

  // Convert duty cycle to an uint32_t in range of [0, TIM1->ARR + 1] and set
  // it in TIM1
  STM32_TIM1->CCR[0] = static_cast<uint32_t>((STM32_TIM1->ARR + 1) * duty);
} // Update()

void SpeedController::setEnabled(bool state)
{
  if (state) {
    palClearPad(GPIOF, 4);  // enable
    STM32_TIM1->CCR[0] = 0; // 0% duty cycle
  } else {
    palSetPad(GPIOF, 4);  //   disable
    STM32_TIM1->CCR[0] = 0; // 0% duty cycle
  }
  Enabled_ = state;
} // setEnabled()
