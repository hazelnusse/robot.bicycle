#include <cstring>

#include "SampleAndControl.h"
#include "chprintf.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "YawRateController.h"
#include "pb_encode.h"
#include "SystemState.h"


const uint16_t SampleAndControl::buffer_size_;
const uint16_t SampleAndControl::write_size_;

SampleAndControl::SampleAndControl()
  : tp_control(0), tp_write(0), state_(0)
{
}

void SampleAndControl::controlThread()
{
  chRegSetThreadName("Control");
  enableSensorsMotors();
  MPU6050 & imu = MPU6050::Instance();
  RearWheel & rwc = RearWheel::Instance();
  YawRateController & yrc = YawRateController::Instance();
  
  // zero out wheel encoders and system timer
  STM32_TIM4->CNT = STM32_TIM5->CNT = STM32_TIM8->CNT = 0;

  // Create a sample to populate
  Sample s;
  memset(&s, 0, sizeof(s));

  systime_t time = chTimeNow();     // Initial time
  systime_t sleep_time;
  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(con::T_ms);       // Next deadline

    // Begin pre control data collection
    sampleTimers(s);  // sample system time/encoder counts/PWM duty cycle
    imu.Acquire(s);   // sample rate gyro, accelerometer and temperature sensors
    sampleSetPoints(s); // sample rear wheel and yaw rate commands
    // End pre control data collection

    // Begin control
    if (rwc.isEnabled() && (i % con::RW_N == 0))
      rwc.Update(s);

    if (yrc.isEnabled() && (i % con::YC_N == 0))
      yrc.Update(s);
    // End control

    // Begin post control data collection
    sampleMotorState(s);
    s.system_state |= systemstate::CollectionEnabled;
    // End post control data collection

    chMsgSend(tp_write, reinterpret_cast<msg_t>(&s));

    // Clear the sample for the next iteration
    // The first time through the loop, computation_time will be logged as zero,
    // subsequent times will be accurate but delayed by one sample period
    uint32_t temp = s.system_time;
    memset(&s, 0, sizeof(s));
    s.computation_time= STM32_TIM5->CNT - temp;

    // Go to sleep until next interval
    chSysLock();
    sleep_time = time - chTimeNow();
    if (static_cast<int32_t>(sleep_time) > 0)
      chThdSleepS(sleep_time);
    chSysUnlock();
  } // for
  
  // Clean up
  disableSensorsMotors();
  // End cleanup

  chThdExit(0);
}

// Caller: Shell thread
void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc > 1 || argc < 0) {
    chprintf(chp, "Invalid usage.\r\n");
    return;
  }

  msg_t m;
  if (tp_control) {
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

void SampleAndControl::writeThread(char* filename)
{
  chRegSetThreadName("WriteThread");
  FRESULT res = FR_OK;
  res = f_open(&f_, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK) {
    chSysHalt(); while (1) {}   // couldn't properly open the file!
  }

  std::array<uint8_t, buffer_size_> *buffer = &buffer0_;
  std::array<uint8_t, buffer_size_> *inactive_buffer = &buffer1_;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer->data(),
                                               buffer->size());

  uint16_t bytes = 0;
  uint16_t extra_bytes;
  uint16_t message_size;
  uint32_t write_errors = 0;
  UINT bytes_written = 0;
  Sample *sp;
  while (1) {
    Thread *calling_thread = chMsgWait();
    sp = reinterpret_cast<Sample*>(chMsgGet(calling_thread));
    if (sp == 0) {
      chMsgRelease(calling_thread, res);
      break;
    } else {
      if (bytes_written > 0) { // record if a write occured with the last sample
        sp->system_state |= systemstate::FileSystemWriteTriggered;
        bytes_written = 0;
      }

      message_size = getMessageSize(*sp);
      pb_write(&stream, reinterpret_cast<uint8_t*>(&message_size), sizeof(message_size));
      pb_encode(&stream, Sample_fields, sp);
      bytes += sizeof(message_size) + message_size;
      chMsgRelease(calling_thread, res);
    }

    if (bytes < write_size_)
        continue;

    res = f_write(&f_, buffer->data(), write_size_, &bytes_written);
    extra_bytes = bytes - write_size_;
    bytes = 0;

    if (extra_bytes > 0) // copy over bytes not written to disk
        memcpy(inactive_buffer->data(), buffer->data() + write_size_, extra_bytes);

    // swap buffers
    inactive_buffer = buffer;
    if (buffer == &buffer0_)
        buffer = &buffer1_;
    else
        buffer = &buffer0_;
    stream = pb_ostream_from_buffer(buffer->data(), buffer->size());

    if (res != FR_OK)
      ++write_errors;
  }

  f_close(&f_);           // close the file
  chThdExit(write_errors);
}

msg_t SampleAndControl::Start(const char* filename)
{
  msg_t m = 0;
  tp_write = chThdCreateStatic(SampleAndControl::waWriteThread,
                          sizeof(waWriteThread),
                          NORMALPRIO + 1,
                          reinterpret_cast<tfunc_t>(writeThread_),
                          const_cast<char*>(filename));
  if (!tp_write) {
    m = (1 << 0);
    return m;
  }

  while (tp_write->p_state != THD_STATE_WTMSG)
    chThdYield();

  tp_control = chThdCreateStatic(SampleAndControl::waControlThread,
                          sizeof(waControlThread),
                          NORMALPRIO + 2,
                          reinterpret_cast<tfunc_t>(controlThread_),
                          0);
  if (!tp_control)
    m |= (1 << 1);

  return m;
}

msg_t SampleAndControl::Stop()
{
  msg_t m;

  chThdTerminate(tp_control);
  //chThdWait(tp_control);
  //while(tp_control->p_state != THD_STATE_FINAL)
    //chThdYield();
  tp_control = 0;

  chMsgSend(tp_write, 0); // send a message to end the write thread.
  m = chThdWait(tp_write);
  tp_write = 0;

  return m;
}

size_t SampleAndControl::getMessageSize(const Sample & s)
{
  pb_ostream_t sizestream = PB_OSTREAM_SIZING;
  pb_encode(&sizestream, Sample_fields, &s);
  return sizestream.bytes_written;
}

