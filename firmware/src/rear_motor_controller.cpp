#include <cstdint>
#include "constants.h"
#include "rear_motor_controller.h"

namespace hardware {

const uint16_t counts_per_revolution = 800; // US Digital 200 count + 4:1 pulley
const uint8_t ccr_channel = 1;              // PWM Channel 1
const float max_current = 12.0f;             // Copley Controls ACJ-090-36
const float torque_constant = 6.654987675770698f;  // Experimentally determined

RearMotorController::RearMotorController()
  : MotorController("rear wheel"),
  e_(STM32_TIM3, counts_per_revolution),
  m_(GPIOF, GPIOF_RW_DIR, GPIOF_RW_ENABLE, GPIOF_RW_FAULT,
     STM32_TIM1, ccr_channel, max_current, torque_constant, true),
  theta_R_dot_command_{0.0f}
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
  s.encoder.rear_wheel_count = e_.get_count();
  s.encoder.rear_wheel = e_.get_angle();
  s.set_point.theta_R_dot = theta_R_dot_command_;
  // TODO:
  //  implement rate divider
  //  determine motor current command, save it in s.motor_current
}

} // namespace hardware
