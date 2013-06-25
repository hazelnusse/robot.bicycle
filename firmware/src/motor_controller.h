#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "hal.h"
#include "sample.pb.h"

namespace hardware {

class MotorController {
 public:
  MotorController(const char * name) : name_(name) {};
  const char * name() { return name_; }

  virtual void set_reference(float reference) = 0;
  virtual void disable() = 0;
  virtual void enable() = 0;
  virtual void update(Sample & s) = 0;

  void set_reference_shell(BaseSequentialStream * chp, int argc, char * argv[]);

 private:
  const char * const name_;
};

extern MotorController * instances[];
enum controller_t {rear_wheel = 0,
                   fork = 1};

template <controller_t T>
static void set_reference_shell(BaseSequentialStream * chp, int argc, char * argv[])
{
  instances[T]->set_reference_shell(chp, argc, argv);
}

} // namespace hardware

#endif

