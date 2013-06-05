#include <cstdint>
#include "constants.h"
#include "fork_motor_controller.h"
#include "MPU6050.h"
#include "SystemState.h"

namespace hardware {

const uint8_t ccr_channel = 0;                 // PWM Channel 0
const float max_current = 6.0;                 // Copley Controls ACJ-055-18
const float torque_constant = 106.459f * constants::Nm_per_ozfin;

ForkMotorController::ForkMotorController()
  : MotorController("Fork"),
  e_(STM32_TIM3, constants::fork_counts_per_revolution),
  m_(GPIOF, GPIOF_STEER_DIR, GPIOF_STEER_ENABLE, GPIOF_STEER_FAULT,
     STM32_TIM1, ccr_channel, max_current, torque_constant),
  estimation_threshold_(0.0f),
  control_threshold_(0.0f)
{
  instances[fork] = this;
}

ForkMotorController::~ForkMotorController()
{
  instances[fork] = 0;
}

void ForkMotorController::set_reference(float yaw_rate)
{
  yaw_rate_command_ = yaw_rate;
}

void ForkMotorController::set_estimation_threshold(float wheel_rate)
{
  estimation_threshold_  = wheel_rate;
}

void ForkMotorController::set_control_threshold(float wheel_rate)
{
  control_threshold_ = wheel_rate;
}

void ForkMotorController::disable()
{
  m_.disable();
}

void ForkMotorController::enable()
{
  m_.enable();
}

void ForkMotorController::set_estimation_threshold_shell(BaseSequentialStream *chp,
                                                         int argc, char *argv[])
{
  if (argc == 1) {
      ForkMotorController* fmc = reinterpret_cast<ForkMotorController*>(instances[fork]);
      fmc->set_estimation_threshold(tofloat(argv[0]));
      chprintf(chp, "%s estimation threshold set to %f.\r\n", fmc->name(),
               fmc->estimation_threshold_);
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void ForkMotorController::set_control_threshold_shell(BaseSequentialStream *chp,
                                                      int argc, char *argv[])
{
  if (argc == 1) {
      ForkMotorController* fmc = reinterpret_cast<ForkMotorController*>(instances[fork]);
      fmc->set_control_threshold(tofloat(argv[0]));
      chprintf(chp, "%s control threshold set to %f.\r\n", fmc->name(),
               fmc->control_threshold_);
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void ForkMotorController::update(Sample & s)
{
  s.encoder.steer = e_.get_angle();
  s.set_point.psi_dot = yaw_rate_command_;
  s.yaw_rate_pi.e = s.set_point.psi_dot - MPU6050::psi_dot(s);
  x_pi_ += s.yaw_rate_pi.e;
  s.yaw_rate_pi.x = x_pi_;

  if (should_estimate(s) && fork_control_.set_sample(s)) {
    const float torque = fork_control_.compute_updated_torque(m_.get_torque());
    if (should_control(s))
      m_.set_torque(torque);
  } else {
    m_.set_torque(0.0f);
  }
  s.motor_current.steer = m_.get_current();

  if (e_.rotation_direction())
    s.system_state |= systemstate::SteerEncoderDir;
  if (m_.is_enabled())
    s.system_state |= systemstate::SteerMotorEnable;
  if (m_.has_fault())
    s.system_state |= systemstate::SteerMotorFault;
  if (m_.current_direction())
    s.system_state |= systemstate::SteerMotorCurrentDir;
}

// estimation/control thresholds are in terms of wheel rate, which is defined
// to be negative when the speed of the bicycle is positive. estimation/control
// should occur when speed > threshold which is equivalent to rate < threshold.
bool ForkMotorController::should_estimate(const Sample& s) const {
  return s.encoder.rear_wheel_rate < estimation_threshold_;
}

bool ForkMotorController::should_control(const Sample& s) const {
  return (s.encoder.rear_wheel_rate < control_threshold_ &&
          std::fabs(s.encoder.steer) < max_steer_angle_);
}

} // namespace hardware
