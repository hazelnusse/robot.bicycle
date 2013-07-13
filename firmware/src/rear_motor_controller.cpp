#include <cstdint>
#include <cmath>
#include "ch.h"
#include "chprintf.h"
#include "constants.h"
#include "rear_motor_controller.h"
#include "SystemState.h"
#include "textutilities.h"

namespace hardware {

const uint8_t ccr_channel = 1;              // PWM Channel 1
const float max_current = 24.0f;            // Copley Controls ACJ-090-36
                                            // Configured 24.0A peak, 12.0A
                                            // continuous
const float torque_constant = 6.654987675770698f;  // Experimentally determined, N*m/A
const float max_torque = max_current * torque_constant;
const float d0 = 10.0f * constants::two_pi; // filter pole at -d0 rad / s
const float n0 = d0;
const float n1 = 0.0f;

RearMotorController::RearMotorController()
  : MotorController{"rear wheel"},
  e_{STM32_TIM8, constants::wheel_counts_per_revolution},
  m_{GPIOF, GPIOF_RW_DIR, GPIOF_RW_ENABLE, GPIOF_RW_FAULT,
     STM32_TIM1, ccr_channel, max_current, torque_constant, true},
  theta_R_dot_command_{0.0f}, integrator_state_{0.0f},
  K_{50.0f}, Ti_{2000.0f},
  rear_wheel_rate_prev_{0.0f},
  system_time_prev_{0}, rear_wheel_count_prev_{0},
  low_pass_filter_{n0, n1, d0, constants::loop_period_s},
  dthetadt_array_{{}}, dthetadt_elem_{0},
  distance_{0.0f}, distance_limit_{0.0f}
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
  float theta_R_dot_command_new = speed / -constants::wheel_radius;
  theta_R_dot_command_ = theta_R_dot_command_new;
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

  const float error = theta_R_dot_command_ - s.encoder.rear_wheel_rate;
  s.motor_torque.desired_rear_wheel = K_ * error + integrator_state_;

  m_.set_torque(s.motor_torque.desired_rear_wheel);         // desired torque
  s.motor_torque.rear_wheel = m_.get_torque();              // saturated torque

  // update integrator state if torque not saturating
  if (s.motor_torque.rear_wheel == s.motor_torque.desired_rear_wheel)
    integrator_state_ += K_ / Ti_ * error * dt;

  // update distance travelled
  distance_ += ((s.encoder.rear_wheel_count - rear_wheel_count_prev_) *
                e_.get_rad_per_count() * constants::wheel_radius);
  // step down speed setpoint if distance limit has been reached and
  // distance limit is positive
  if (distance_limit_ > 0.0f && distance_ > distance_limit_) {
    distance_limit_ = 0.0f;
    // Set the reference speed to be a small value that allows a person to
    // catch up to the bike but large enough such that the bike balances.
    set_reference(1.0f);
  }

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

  //s.wheel_rate_pi.x = theta_R_dot_command_;
  //s.wheel_rate_pi.e = e;
  //s.wheel_rate_pi.e_s = e_s;
  //s.wheel_rate_pi.K = K_;
  //s.wheel_rate_pi.Ti = Ti_;
  //s.wheel_rate_pi.Tt = Tt_;
  //s.wheel_rate_pi.dt = dt;
  //s.has_wheel_rate_pi = true;
}

void RearMotorController::set_distance_limit(float distance)
{
  distance_limit_ = distance;
}

void RearMotorController::speed_limit_shell(BaseSequentialStream *chp,
                                            int argc, char *argv[])
{
  if (argc == 2) {
    RearMotorController* fmc = reinterpret_cast<RearMotorController*>(
                                        instances[rear_wheel]);
    fmc->set_reference_shell(chp, argc - 1, argv);
    fmc->set_distance_limit(tofloat(argv[1]));
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
}

} // namespace hardware

