#include "control_loop.h"
#include "SystemState.h"

namespace hardware {

WORKING_AREA(ControlLoop::waControlThread, 4096);
Thread * ControlLoop::tp_control_ = 0;

ControlLoop::ControlLoop()
  : front_wheel_encoder_{STM32_TIM4, 800}
{
  front_wheel_encoder_.set_count(0);
  STM32_TIM5->CNT = 0;
}

// Caller: shell thread
msg_t ControlLoop::start(const char * filename)
{
  tp_control_ = chThdCreateStatic(waControlThread,
                                  sizeof(waControlThread),
                                  chThdGetPriority() + 3,
                                  ControlLoop::thread_function, 
                                  const_cast<char *>(filename));
  if (!tp_control_)
    return 1;

  return 0;
}

// Caller: shell thread
msg_t ControlLoop::stop()
{
  chThdTerminate(tp_control_);
  msg_t m = chThdWait(tp_control_);
  tp_control_ = 0;
  return m;
}

// Caller: shell thread
void ControlLoop::shell_command(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc > 1 || argc < 0) {
    chprintf(chp, "Invalid usage.\r\n");
    return;
  }
  
  msg_t m;
  if (tp_control_) { // control thread is running
    m = stop();
    if (argc == 0) {
      chprintf(chp, "Data collection and control terminated with %d errors.\r\n", m);
      return;
    }
  } else {           // control thread is not running
    if (argc == 0)
      m = start("samples.dat");
    else
      m = start(argv[0]);

    if (m == 0) {
      chprintf(chp, "Data collection and control initiated.\r\n");
      return;
    }
  }
  chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
}

// Caller: initially called by OS when thread is created.
// This is the highest priority thread
msg_t ControlLoop::thread_function(void * arg)
{
  chRegSetThreadName("control");
  ControlLoop loop;

  return loop.exec(static_cast<const char *>(const_cast<const void *>(arg)));
}

// Caller: Control thread
msg_t ControlLoop::exec(const char * file_name)
{
  logging::SampleBuffer sample_buffer(file_name);
  Sample s;

  systime_t time = chTimeNow();     // Initial time
  systime_t sleep_time;
  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(constants::loop_period_ms);       // Next deadline

    // Begin pre control data collection
    s.system_time = STM32_TIM5->CNT;
    s.loop_count = i;
    imu_.acquire_data(s);
    s.encoder.front_wheel = front_wheel_encoder_.get_angle();
    s.system_state |= systemstate::CollectionEnabled;
    // End pre control data collection

    // Begin control
    rear_motor_controller_.update(s); // must be called prior to fork update since
    fork_motor_controller_.update(s); // rear wheel rate is needed for gain scheduling
    // End control

    // Put the sample in to the buffer
    bool encode_failure = !sample_buffer.insert(s);

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
  
  return sample_buffer.flush_and_close();
}

}
