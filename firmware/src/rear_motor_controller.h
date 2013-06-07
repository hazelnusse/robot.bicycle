#ifndef REAR_MOTOR_CONTROLLER_H
#define REAR_MOTOR_CONTROLLER_H

#include <array>

#include "encoder.h"
#include "filter.h"
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
  float sg_smoother(float dthetadt);
  Encoder e_;
  Motor m_;
  float theta_R_dot_command_;
  float error_integral_;
  float Kp_, Ki_;
  float rear_wheel_rate_prev_;
  uint32_t system_time_prev_;
  uint32_t rear_wheel_count_prev_;
  control::first_order_discrete_filter<float> low_pass_filter_;
  std::array<float, 5> sg_data_;
  uint8_t sg_insert_index_;
};

} // namespace hardware

#endif

