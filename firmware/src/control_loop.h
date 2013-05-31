#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "MPU6050.h"
#include "rear_motor_controller.h"
#include "fork_motor_controller.h"
#include "SampleBuffer.h"

namespace hardware {

class ControlLoop {
 public:
  static void shell_command(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  ControlLoop(const char * arg);
  ~ControlLoop();
  msg_t exec();
  static msg_t start(const char * data_file);
  static msg_t stop();
  static msg_t thread_function(void * arg);
  static WORKING_AREA(waControlThread, 4096);
  static Thread * tp_control_;

  MPU6050 imu_;
  RearMotorController rear_motor_controller_;
  ForkMotorController fork_motor_controller_;
  Encoder front_wheel_encoder_;
  Sample s_;
  // TODO:  Fix SampleBuffer to be non-singleton
  SampleBuffer & sample_buffer_;

};

}

#include "control_loop-inl.h"

#endif

