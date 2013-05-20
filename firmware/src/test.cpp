#include "hal.h"
#include "test.h"
#include "encoder.h"
#include "motor.h"

bool test_encoder()
{
  Encoder e(STM32_TIM3, 5000);
  float y = e.get_angle();
  e.set_count(y);
  y = e.get_count();
  return true;
}

bool test_motor()
{
  Motor steer_motor(GPIOF, GPIOF_STEER_DIR, GPIOF_STEER_ENABLE, GPIOF_STEER_FAULT,
                    STM32_TIM1, 0, 6.0f, 1.0f);
  steer_motor.set_torque(1.0f);
  steer_motor.enable();
  steer_motor.disable();

  Motor rear_wheel_motor(GPIOF, GPIOF_RW_DIR, GPIOF_RW_ENABLE, GPIOF_RW_FAULT,
                         STM32_TIM1, 0, 6.0f, 1.0f, true);
  rear_wheel_motor.set_torque(1.0f);
  rear_wheel_motor.enable();
  rear_wheel_motor.disable();
  return true;
}

bool test_all()
{
  if (test_encoder() &&
      test_motor())
    return true;

  return false;
}

