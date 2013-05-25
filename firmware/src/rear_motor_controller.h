#ifndef REAR_MOTOR_CONTROLLER_H
#define REAR_MOTOR_CONTROLLER_H

#include "encoder.h"
#include "motor.h"
#include "motor_controller.h"
#include "sample.pb.h"

namespace hardware {

class RearMotorController : public MotorController {
 public:
  RearMotorController();
  ~RearMotorController();
  virtual void set_reference(float speed);
  virtual void disable();
  virtual void enable();
  virtual void update(Sample & s);

 private:
  float theta_R_dot_estimate(const Sample & s, float dt);

  Encoder e_;
  Motor m_;
  float theta_R_dot_command_;
  float error_integral_;
  float Kp_, Ki_;
  uint32_t system_time_prev_;
  uint32_t rear_wheel_count_prev_;
  float A_lpf_, B_lpf_, C_lpf_, x_lpf_;
};

} // namespace hardware

#endif

