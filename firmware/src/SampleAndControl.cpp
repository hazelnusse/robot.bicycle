#include <cstring>

#include "SampleAndControl.h"
#include "chprintf.h"

#include "constants.h"
#include "MPU6050.h"
#include "rear_motor_controller.h"
#include "fork_motor_controller.h"
#include "SampleBuffer.h"
#include "SystemState.h"

SampleAndControl::SampleAndControl()
  : tp_control_(0), state_(0)
{
}

void SampleAndControl::controlThread(const char * filename)
{
  chRegSetThreadName("Control");
  // TODO: verify that functionality here has been moved to motor_controller
  // subclasses
  // enableSensorsMotors();
  hardware::MPU6050 imu;
  if (!imu.is_initialized())
    chThdExit(-1); // TODO: figure what to do here

  hardware::RearMotorController rear_motor_controller;
  hardware::ForkMotorController fork_motor_controller;
  hardware::Encoder front_wheel_encoder(STM32_TIM4, 800);
  front_wheel_encoder.set_count(0);

  // TODO: make SampleBuffer non-singleton
  SampleBuffer & sb = SampleBuffer::Instance();
  sb.initialize(filename);
  
  // zero out system timer
  STM32_TIM5->CNT = 0;

  // Create a sample to populate
  Sample s;
  memset(&s, 0, sizeof(s));

  systime_t time = chTimeNow();     // Initial time
  systime_t sleep_time;
  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(constants::loop_period_ms);       // Next deadline

    // Begin pre control data collection
    s.system_time = STM32_TIM5->CNT;
    s.loop_count = i;
    imu.acquire_data(s);
    s.encoder.front_wheel = front_wheel_encoder.get_angle();
    s.system_state |= systemstate::CollectionEnabled;
    // End pre control data collection

    // Begin control
    rear_motor_controller.update(s);
    fork_motor_controller.update(s);
    // End control

    // TODO: Push sampleMotorState functionality down into motor_controller
    // class or subclasses.
    // sampleMotorState(s);

    // Put the sample in to the buffer
    bool encode_failure = false;
    if (!sb.insert(s))
      encode_failure = true;

    // Clear the sample for the next iteration
    // The first time through the loop, computation_time will be logged as zero,
    // subsequent times will be accurate but delayed by one sample period. This
    // is done to ensure that encoding computation is part of timing
    // measurement.
    uint32_t ti = s.system_time;
    memset(&s, 0, sizeof(s));
    s.computation_time = STM32_TIM5->CNT - ti;
    // Similarly, encode failures will be delayed by one sample.
    if (encode_failure) 
      s.system_state |= systemstate::SampleBufferEncodeError;

    // Go to sleep until next interval
    chSysLock();
    sleep_time = time - chTimeNow();
    if (static_cast<int32_t>(sleep_time) > 0)
      chThdSleepS(sleep_time);
    chSysUnlock();
  } // for
  
  // Clean up
  // TODO:
  // disableSensorsMotors();
  // TODO: change samplebuffer to be a non-singleton class and implement
  // cleanup in destructor
  msg_t write_errors = sb.deinitialize();
  // End cleanup
 
  // TODO: possibly just return...
  chThdExit(write_errors);
}

// Caller: Shell thread
void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc > 1 || argc < 0) {
    chprintf(chp, "Invalid usage.\r\n");
    return;
  }

  msg_t m;
  if (tp_control_) {
    m = Stop();
    if (argc == 0) {
      chprintf(chp, "Data collection and control terminated with %d errors.\r\n", m);
      return;
    }
  } else { // control thread not running
    if (argc == 0)
      m = Start("samples.dat");// Start data collection to default file "samples.dat"
    else
      m = Start(argv[0]);// Start data collection to file in argv[0]

    if (m == 0) {
      chprintf(chp, "Data collection and control initiated.\r\n");
      return;
    }
  }
  chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
}

msg_t SampleAndControl::Start(const char * filename)
{
  tp_control_ = chThdCreateStatic(SampleAndControl::waControlThread,
                                 sizeof(waControlThread),
                                 NORMALPRIO + 3,
                                 reinterpret_cast<tfunc_t>(controlThread_),
                                 const_cast<char *>(filename));
  if (!tp_control_)
    return 1;

  return 0;
}

msg_t SampleAndControl::Stop()
{
  chThdTerminate(tp_control_);
  msg_t m = chThdWait(tp_control_);
  tp_control_ = 0;
  return m;
}

