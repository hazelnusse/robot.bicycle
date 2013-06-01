#ifndef FORK_MOTOR_CONTROLLER_H
#define FORK_MOTOR_CONTROLLER_H

#include "encoder.h"
#include "gain_schedule.h"
#include "motor.h"
#include "motor_controller.h"
#include "sample.pb.h"

namespace hardware {

class ForkMotorController : public MotorController {
 public:
  ForkMotorController();
  ~ForkMotorController();
  virtual void set_reference(float yaw_rate);
  virtual void disable();
  virtual void enable();
  virtual void update(Sample & s);

 private:
  bool should_estimate(const Sample& s) const;
  bool should_control(const Sample& s) const;
  Encoder e_;
  Motor m_;
  control::GainSchedule fork_control_;

  float yaw_rate_command_;
  float x_pi_;
  float estimation_threshold_; // in terms of rear wheel rate
  float control_threshold_; // in terms of rear wheel rate
  const float max_steer_angle_ = 45; //degrees
};

} // namespace hardware

#endif // FORK_MOTOR_CONTROLLER_H

