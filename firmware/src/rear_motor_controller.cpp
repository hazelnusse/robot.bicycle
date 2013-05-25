#include <cstdint>
#include <cmath>
#include "constants.h"
#include "rear_motor_controller.h"

namespace hardware {

const uint16_t counts_per_revolution = 800; // US Digital 200 count + 4:1 pulley
const uint8_t ccr_channel = 1;              // PWM Channel 1
const float max_current = 12.0f;            // Copley Controls ACJ-090-36
const float torque_constant = 6.654987675770698f;  // Experimentally determined
const uint8_t prescalar = 10;               // Update control at 1/10th the frequency of main

RearMotorController::RearMotorController()
  : MotorController{"rear wheel"},
  e_{STM32_TIM3, counts_per_revolution},
  m_{GPIOF, GPIOF_RW_DIR, GPIOF_RW_ENABLE, GPIOF_RW_FAULT,
     STM32_TIM1, ccr_channel, max_current, torque_constant, true},
  theta_R_dot_command_{0.0f}, error_integral_{0.0f}, Kp_{3.0f}, Ki_{1.0f},
  system_time_prev_{0}, rear_wheel_count_prev_{0},
  A_lpf_{std::exp(constants::lpf_pole * prescalar * constants::loop_period_ms * 0.001f)},
  B_lpf_{(A_lpf_ - 1.0f)/constants::lpf_pole},
  C_lpf_{-constants::lpf_pole}
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
  theta_R_dot_command_ = speed / constants::wheel_radius;
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
  // Save encoder values, commanded rear wheel rate
  s.encoder.rear_wheel_count = e_.get_count();
  s.encoder.rear_wheel = e_.get_angle();
  s.set_point.theta_R_dot = theta_R_dot_command_;

  if (s.loop_count % prescalar == 0) {
    const float dt = (s.system_time - system_time_prev_)
                      * constants::system_timer_period;
    const float error =  theta_R_dot_command_ - theta_R_dot_estimate(s, dt);
    const float error_integral_update = error_integral_ + error * dt;
    const float torque = Kp_ * error + Ki_ * error_integral_update;
    if (m_.set_torque(torque))
      error_integral_ = error_integral_update;

    system_time_prev_ = s.system_time;
    rear_wheel_count_prev_ = s.encoder.rear_wheel_count;

  }

  s.motor_current.rear_wheel = m_.get_current();
}

float RearMotorController::theta_R_dot_estimate(const Sample & s, float dt)
{
  const float dthetadt = static_cast<int16_t>(s.encoder.rear_wheel_count
                                            - rear_wheel_count_prev_)
                                        * e_.get_rad_per_count() / dt;
  x_lpf_ = A_lpf_ * x_lpf_ + B_lpf_ * dthetadt;
  return C_lpf_ * x_lpf_;
}


} // namespace hardware
