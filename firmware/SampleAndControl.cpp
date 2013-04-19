#include <cstring>

#include "SampleAndControl.h"
#include "chprintf.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "YawRateController.h"
#include "Test.pb.h"
#include "pb_encode.h"

SampleAndControl::SampleAndControl()
  : tp_control(0), tp_write(0), state_(0)
{
}

void SampleAndControl::controlThread(char* filename)
{
  chRegSetThreadName("Control");
  FRESULT res;
  msg_t msg;

  res = f_open(&f_, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK) {
    chSysHalt(); while (1) {}   // couldn't properly open the file!
  }

  for (uint32_t i = 0; i < NUMBER_OF_SAMPLES; ++i)
    samples[i].clear();

  MPU6050 & imu = MPU6050::Instance();
  if (!imu.Initialize(&I2CD2)) {
    chSysHalt(); while(1) {}    // couldn't initialize the MPU6050
  }

  RearWheel & rw = RearWheel::Instance();
  rw.Reset();
  rw.turnOn();
  YawRateController & yc = YawRateController::Instance();
  yc.Reset();
  yc.turnOn();

  // zero out wheel encoders and system timer
  STM32_TIM4->CNT = STM32_TIM5->CNT = STM32_TIM8->CNT = 0;
  bool data = false;                // flag to indicate we have data to write
  uint32_t write_errors = 0;
  //uint32_t rw_fault_count = 0;
  //uint32_t steer_fault_count = 0;

  systime_t time = chTimeNow();     // Initial time
  uint32_t i;
  for (i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(con::T_ms);       // Next deadline

    // Get a sample to populate
    Sample & s = samples[i % NUMBER_OF_SAMPLES];

    // Begin data collection
    sampleTimers(s);  // sample system time/encoder counts/PWM duty cycle
    imu.Acquire(s);   // sample rate gyro, accelerometer and temperature sensors
    // sampleStates(s);  // sample various system states
    state_ = sampleSystemState() | Sample::CollectionEnabled;

    s.RearWheelRate_sp = rw.RateCommanded();
    s.YawRate_sp = yc.RateCommanded();
    // End data collection

    // Begin control
    if (rw.isEnabled() && (i % con::RW_N == 0))
      rw.Update(s);

    if (yc.isEnabled() && (i % con::YC_N == 0))
      yc.Update(s);
    // End control

    // Record PWM duty cycle and direction
    s.CCR_rw = STM32_TIM1->CCR[0];      // RW PWM duty cycle
    s.CCR_steer = STM32_TIM1->CCR[1];   // Steer PWM duty cycle
    state_ |= rw.CurrentDir() | yc.CurrentDir(); // RW and Steer current dir
    
    if (data && (i % (NUMBER_OF_SAMPLES/2) == 0)) {
      /* write data from buffer half that just completed */
      msg_t buffer = reinterpret_cast<msg_t>(get_buffer(i + NUMBER_OF_SAMPLES / 2));
      state_ |= Sample::FileSystemWriteTriggered;
      msg = chMsgSend(tp_write, buffer);
      if (static_cast<FRESULT>(msg) != FR_OK)
        ++write_errors;
    }

    data = true;

    s.SystemState = state_;
    // Measure computation time
    s.ComputationTime = STM32_TIM5->CNT - s.SystemTime;
    // Go to sleep until next 5ms interval
    chThdSleepUntil(time);
  } // for i @ 200Hz
  
  // Clean up
  imu.DeInitialize();
  rw.turnOff();
  yc.turnOff();
  // End cleanup

  msg = chMsgSend(tp_write, 0); // send a message to end the write thread.
  if (static_cast<FRESULT>(msg) != FR_OK)
    ++write_errors;

  while (tp_write)
    chThdYield();           // yield this time slot so write thread can finish

  // Write remaing samples in partially filled buffer.
  if (write_last_samples(i) != FR_OK)
    ++write_errors;
  // Save system state, clearing a few bits.
  state_ &= ~(Sample::CollectionEnabled | Sample::FileSystemWriteTriggered
              | Sample::RearWheelMotorEnable | Sample::SteerMotorEnable);
  f_close(&f_);           // close the file
  chThdExit(write_errors);
}

Sample* SampleAndControl::get_buffer(uint32_t index) const {
  /* If i is an odd multiple NUMBER_OF_SAMPLES/2  (64, 192, 320, ...), return
   * the second half of the buffer. Otherwise return the first half.
   * */
  uint32_t offset = 0;
  if ((index / (NUMBER_OF_SAMPLES / 2)) & 1) 
    offset = NUMBER_OF_SAMPLES / 2;
  return const_cast<Sample*>(samples) + offset;
}

FRESULT SampleAndControl::write_last_samples(uint32_t index) {
  FRESULT res = FR_OK;
  uint32_t j = index % (NUMBER_OF_SAMPLES/2);
  if (j) {
    uint8_t* b = reinterpret_cast<uint8_t*>(get_buffer(index));
    UINT bytes;
    res = f_write(&f_, b, sizeof(Sample) * j, &bytes);
  }
  return res;
}

// Caller: Shell thread
void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {         // Start/Stop data collection, default filename
    if (tp_control) {      // Data collection enabled
      msg_t m = Stop();    // Stop it
      chprintf(chp, "Data collection and control terminated with %d errors.\r\n", m);
    } else {               // Data collecton disabled
      msg_t m = Start("samples.dat");// Start data collection to default file "samples.dat"
      if (m == 0) {
        chprintf(chp, "Data collection and control initiated.\r\n");
      } else {
        chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
      }
    }
  } else if (argc == 1) { // Start/Stop data collection, with filename
    if (tp_control) {     // Data collection enabled
      msg_t m = Stop();   // Stop it, ignoring the argument
      chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
    } else {              // Data collection is disabled
      msg_t m = Start(argv[0]);// Start data collection to file in argv[0]
      if (m == 0) {
        chprintf(chp, "Data collection and control initiated.\r\n");
      } else {
        chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
      }
    }
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
}

void SampleAndControl::writeThread()
{
  UINT bytes;
  msg_t m = -1;
  FRESULT res = FR_OK;
  uint8_t *b;

  chRegSetThreadName("WriteThread");
  while (1) {
    Thread * calling_thread = chMsgWait();
    m = chMsgGet(calling_thread);
    chMsgRelease(calling_thread, res);

    if (m == 0) break;

    b = reinterpret_cast<uint8_t *>(m);

    res = f_write(&f_, b, sizeof(Sample)*NUMBER_OF_SAMPLES/2, &bytes);
  }
  tp_write = 0;
  chThdExit(0);
}

msg_t SampleAndControl::Start(const char* filename)
{
  msg_t m = 0;
  tp_write = chThdCreateStatic(SampleAndControl::waWriteThread,
                          sizeof(waWriteThread),
                          NORMALPRIO + 1,
                          reinterpret_cast<tfunc_t>(writeThread_),
                          0);
  if (!tp_write) {
    m = (1 << 0);
  }
  tp_control = chThdCreateStatic(SampleAndControl::waControlThread,
                          sizeof(waControlThread),
                          NORMALPRIO + 1,
                          reinterpret_cast<tfunc_t>(controlThread_),
                          const_cast<char*>(filename));
  if (!tp_control) {
    m |= (1 << 1);
  }

  return m;
}

msg_t SampleAndControl::Stop()
{
  msg_t m;
  chThdTerminate(tp_control);
  m = chThdWait(tp_control);
  tp_control = 0;
  return m;
}

