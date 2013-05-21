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
  Encoder e_;
  Motor m_;
  float speed_command_;
};

} // namespace hardware

#endif

