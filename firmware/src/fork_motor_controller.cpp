#include <cstdint>
#include "constants.h"
#include "fork_motor_controller.h"

namespace hardware {

const uint16_t counts_per_revolution = 20000;  // Teknic M-3441E-LN-20D
const uint8_t ccr_channel = 0;                 // PWM Channel 0
const float max_current = 6.0;                 // Copley Controls ACJ-055-18
const float torque_constant = 106.459f * constants::Nm_per_ozfin;

ForkMotorController::ForkMotorController()
  : MotorController("Fork"),
  e_(STM32_TIM3, counts_per_revolution),
  m_(GPIOF, GPIOF_STEER_DIR, GPIOF_STEER_ENABLE, GPIOF_STEER_FAULT,
     STM32_TIM1, ccr_channel, max_current, torque_constant)
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

void ForkMotorController::disable()
{

}

void ForkMotorController::enable()
{

}

void ForkMotorController::update(Sample & s)
{
  s.encoder.steer = e_.get_angle();
}

} // namespace hardware