#include <cstdint>
#include <cmath>
#include "constants.h"
#include "rear_motor_controller.h"
#include "SystemState.h"

namespace hardware {

const uint8_t ccr_channel = 1;              // PWM Channel 1
const float max_current = 24.0f;            // Copley Controls ACJ-090-36
                                            // Configured 24.0A peak, 12.0A
                                            // continuous
const float torque_constant = 6.654987675770698f;  // Experimentally determined
const float d0 = 10.0f * constants::two_pi; // filter pole at -d0 rad / s
const float n0 = d0;
const float n1 = 0.0f;

RearMotorController::RearMotorController()
  : MotorController{"rear wheel"},
  e_{STM32_TIM8, constants::wheel_counts_per_revolution},
  m_{GPIOF, GPIOF_RW_DIR, GPIOF_RW_ENABLE, GPIOF_RW_FAULT,
     STM32_TIM1, ccr_channel, max_current, torque_constant, true},
  theta_R_dot_command_{0.0f}, integrator_state_{0.0f},
  K_{10.0f},
  rear_wheel_rate_prev_{0.0f},
  system_time_prev_{0}, rear_wheel_count_prev_{0},
  low_pass_filter_{n0, n1, d0, constants::loop_period_s},
  dthetadt_array_{{}}, dthetadt_elem_{0}
{
  instances[rear_wheel] = this;
  e_.set_count(0);
}

RearMotorController::~RearMotorController()
{
  instances[rear_wheel] = 0;
}

void RearMotorController::set_reference(float speed)
{
  theta_R_dot_command_ = speed / -constants::wheel_radius;
}

void RearMotorController::disable()
{
  m_.disable();
}

void RearMotorController::enable()
{
  m_.enable();
}

void RearMotorController::update(Sample & s)
{
  s.encoder.rear_wheel_count = e_.get_count();
  s.encoder.rear_wheel = e_.get_angle();
  s.set_point.theta_R_dot = theta_R_dot_command_;

  // moving average for rear_wheel_rate
  auto& now = dthetadt_array_[dthetadt_elem_];
  now.first = s.system_time;
  now.second = s.encoder.rear_wheel_count;

  dthetadt_elem_ = (dthetadt_elem_ + 1) % dthetadt_array_.size();
  auto& prev = dthetadt_array_[dthetadt_elem_];

  const float dt = (now.first - prev.first) * constants::system_timer_seconds_per_count;
  const float dthetadt = static_cast<int16_t>(now.second - prev.second) *
                            e_.get_rad_per_count() / dt;

  low_pass_filter_.update(dthetadt);
  s.encoder.rear_wheel_rate = low_pass_filter_.output(dthetadt);

  s.motor_torque.desired_rear_wheel = K_ * (theta_R_dot_command_ - s.encoder.rear_wheel_rate);
  m_.set_torque(s.motor_torque.desired_rear_wheel);         // desired torque
  s.motor_torque.rear_wheel = m_.get_torque();              // saturated torque

  system_time_prev_ = s.system_time;
  rear_wheel_count_prev_ = s.encoder.rear_wheel_count;
  rear_wheel_rate_prev_ = s.encoder.rear_wheel_rate;

  if (e_.rotation_direction())
    s.system_state |= systemstate::RearWheelEncoderDir;
  if (m_.is_enabled())
    s.system_state |= systemstate::RearWheelMotorEnable;
  if (m_.has_fault())
    s.system_state |= systemstate::RearWheelMotorFault;
  if (m_.current_direction())
    s.system_state |= systemstate::RearWheelMotorCurrentDir;
}

} // namespace hardware

