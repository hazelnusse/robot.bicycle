#include <cstdlib>
#include <cstdint>
#include <cmath>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "Sample.h"
#include "SpeedController.h"
#include "Constants.h"

// These depend on frequency of control update loop -- don't forget to change
// these if the control update loop is changed.
template <class T>
SpeedController<T> * SpeedController<T>::instance_ = 0;

template <class T>
const float SpeedController<T>::A[2] = {1.0f, 0.534089953667745f};

template <class T>
const float SpeedController<T>::B[2] = {0.125f, 1.178973669432010f};

template <class T>
const float SpeedController<T>::C[2] = {0.312399275353728f, 1.178973669432010f};

template <class T>
const float SpeedController<T>::D = 5.746075010421994f;

template <class T>
float SpeedController<T>::x[2] = {0.0f, 0.0f};

template <class T>
float SpeedController<T>::u = 0.0f;

template <class T>
SpeedController<T>::SpeedController()
  : Enabled_(false), SetPoint_(3.0f), MinSetPoint_(3.0f)
{
  DisableHubMotor();
}

template <class T>
inline void SpeedController<T>::Enable()
{
  EnableHubMotor();
  Enabled_ = true;
}

template <class T>
inline bool SpeedController<T>::Enabled() const
{
  return Enabled_;
}

template <class T>
inline void SpeedController<T>::Disable()
{
  DisableHubMotor();
  Enabled_ = false;
}

template <class T>
inline bool SpeedController<T>::Disabled() const
{
  return !Enabled_;
}

template <class T>
inline void * SpeedController<T>::operator new(size_t, void * location)
{
  return location;
}

template <class T>
SpeedController<T> & SpeedController<T>::Instance()
{
  static uint8_t allocation[sizeof(SpeedController<T>)];

  if (instance_ == 0)
      instance_ = new (allocation) SpeedController<T>;

  return *instance_;
}

template <class T>
void SpeedController<T>::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  SpeedController<T>::Instance().cmd(chp, argc, argv);
}

template <class T>
void SpeedController<T>::cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (Enabled()) {
      Disable();
      chprintf(chp, "Speed control disabled.\r\n");
    } else {
      Enable();
      chprintf(chp, "Speed control enabled.\r\n");
    }
  } else if (argc == 1) { // change set point
    T sp = 0.02*((argv[0][0] - '0')*100 +
                 (argv[0][1] - '0')*10  +
                 (argv[0][2] - '0'))  + MinSetPoint();
    SetPoint(sp);
    chprintf(chp, "Set point changed.\r\n");
  } else { // invalid
    chprintf(chp, "Invalid usage.\r\n");
//        "Usage: speed_control      // Toggles enable/disable\r\n"
//        "       speed_control ijk  // Changed speed set point\r\n"
//        "ijk is a 3 character argument (000-999) which is used as follows:\r\n"
//        "omega_ref = 0.02*ijk + 3.0 [rad/s]\r\n");
  }
}

template <class T>
inline void SpeedController<T>::SetPoint(const T speed)
{
  SetPoint_ = speed;
}

template <class T>
inline T SpeedController<T>::SetPoint() const
{
  return SetPoint_;
}

template <class T>
inline T SpeedController<T>::MinSetPoint() const
{
  return MinSetPoint_;
}

template <class T>
void SpeedController<T>::MinSetPoint(T min)
{
  MinSetPoint_ = min;
}

template <class T>
void SpeedController<T>::Update(Sample & s)
{
  // PeriodCounts has units of TIM4 clock ticks per cycle of rear wheel
  // encoder.  TIM4 clock is at 1.0MHz, one cycle of rear wheel is 2.0*M_PI/200
  // rad.

  uint32_t PeriodCounts = s.RearWheelRate;
  bool dir_bit = PeriodCounts & ~(1 << 31); // get the high bit
  PeriodCounts &= ~(1 << 31);   // clear the high bit which indicates direction
  float vel = 0.0;
  
  // Determine velocity
  if (PeriodCounts) {
    vel = cf::Wheel_rad_counts_per_sec / PeriodCounts;
    if (!dir_bit)
      vel *= -1.0;
  }

  // Set input to PID controller
  u = SetPoint() - vel;

  // Compute output of PID Controller
  float current = C[0]*x[0] + C[1]*x[1] + D*u;

  // Update state of PID controller
  x[0] = A[0]*x[0] + B[0]*u;
  x[1] = A[1]*x[1] + B[1]*u;
  
  // Set direction
  if (current > 0.0) {
    palSetPad(GPIOC, 12);    // set to forward direction
  } else {
    palClearPad(GPIOC, 12);  // set to reverse direction
  }

  // Make current positive
  current = std::fabs(current);

  // Saturate current at max continuous current of Copley drive
  if (current > cf::Current_max_rw)
    current = cf::Current_max_rw;

  // Convert from current to PWM duty cycle;
  float duty = current / cf::Current_max_rw;   // float in range of [0.0, 1.0]

  // Convert duty cycle to an uint32_t in range of [0, TIM4->ARR + 1] and set
  // it in TIM1
  STM32_TIM1->CCR[1] = static_cast<uint32_t>((STM32_TIM1->ARR + 1) * duty);
}

template <class T>
inline void SpeedController<T>::EnableHubMotor()
{
  STM32_TIM1->CCR[1] = 0; // 0% duty cycle
  palClearPad(GPIOC, 11); // enable
}

template <class T>
inline void SpeedController<T>::DisableHubMotor()
{
  STM32_TIM1->CCR[1] = 0; // 0% duty cycle
  palSetPad(GPIOC, 11);   // disable
}

template class SpeedController<int32_t>;
template class SpeedController<float>;
template class SpeedController<double>;
