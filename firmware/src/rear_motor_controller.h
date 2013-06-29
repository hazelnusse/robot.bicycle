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
  float update_dthetadt(uint32_t wheel_count, uint32_t time);
  Encoder e_;
  Motor m_;
  float theta_R_dot_command_;
  float integrator_state_;
  float K_, Ti_, Tt_;
  float rear_wheel_rate_prev_;
  float desired_torque_prev_;
  uint32_t system_time_prev_;
  uint32_t rear_wheel_count_prev_;
  control::first_order_discrete_filter<float> low_pass_filter_;
  std::array<float, 5> sg_data_;
  uint8_t sg_insert_index_;

  static const int averaging_size_ = 20; // 5 ms * 20 = 100 ms
  std::array<std::pair<uint32_t, uint32_t>, averaging_size_> dthetadt_array_;
  int dthetadt_elem_;

};

} // namespace hardware

#endif

