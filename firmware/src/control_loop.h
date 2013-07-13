#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "MPU6050.h"
#include "rear_motor_controller.h"
#include "fork_motor_controller.h"
#include "sample_buffer.h"

namespace hardware {

class ControlLoop {
 public:
  static void shell_command(BaseSequentialStream *chp, int argc, char *argv[]);
  static void set_lean_threshold_shell(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  ControlLoop();
  ~ControlLoop();
  bool hw_button_enabled() const;
  msg_t exec(const char * file_name);
  static void illuminate_lean_steer(const Sample &);
  void set_gyro_lean(Sample& s);
  static msg_t start(const char * file_name);
  static msg_t stop();
  static msg_t thread_function(void * arg);
  static WORKING_AREA(waControlThread, 4096);
  static Thread * tp_control_;
  bool startup_;

  MPU6050 imu_;
  RearMotorController rear_motor_controller_;
  ForkMotorController fork_motor_controller_;
  Encoder front_wheel_encoder_;
  float acc_x_thresh_;
  static ControlLoop * instance_;
};

namespace BikeState { enum bike_state_t {STARTUP, COLLECT, RUNNING, RAMPDOWN}; }
}

#include "control_loop-inl.h"

#endif

