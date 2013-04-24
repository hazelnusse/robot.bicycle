#include <cstring>

#include "SampleAndControl.h"
#include "chprintf.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "YawRateController.h"
#include "pb_encode.h"
#include "SystemState.h"
#include "WriteMessage.h"


const uint16_t SampleAndControl::buffer_size_;

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
  uint16_t unwritten_bytes = 0;
  uint16_t message_size;
  pb_ostream_t stream = pb_ostream_from_buffer(front_buffer_.data(),
                                               front_buffer_.size());
  WriteMessage write_message;
  write_message.m.bytes_to_write = 0;
  write_message.m.buffer_selector = 1;

  systime_t time = chTimeNow();     // Initial time
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
    s.SystemState |= systemstate::CollectionEnabled;
    // End post control data collection

    message_size = getMessageSize(s);
    if (unwritten_bytes + message_size + sizeof(message_size) > buffer_size_) {
      s.SystemState |= systemstate::FileSystemWriteTriggered;
      write_message.m.bytes_to_write = unwritten_bytes;
      chMsgSend(tp_write, write_message.message);
      unwritten_bytes = 0;
      
      // Change the stream to point to the appropriate buffer
      if (write_message.m.buffer_selector & 2) {
        stream = pb_ostream_from_buffer(front_buffer_.data(),
                                        front_buffer_.size());
      } else {
        stream = pb_ostream_from_buffer(back_buffer_.data(),
                                        back_buffer_.size());
      }
      // Toggle the buffer selector bit
      write_message.m.buffer_selector ^= (1 << 1);
    }
    // TODO: implement error checking on pb_write and pb_encode
    pb_write(&stream, reinterpret_cast<uint8_t *>(&message_size), sizeof(message_size));
    pb_encode(&stream, Sample_fields, &s);
    unwritten_bytes += message_size + sizeof(message_size);

    // Clear the sample for the next iteration
    // The first time through the loop, ComputationTime will be logged as zero,
    // subsequent times will be accurate but delayed by one sample period
    uint32_t temp = s.SystemTime;
    memset(&s, 0, sizeof(s));
    s.ComputationTime = STM32_TIM5->CNT - temp;

    // Go to sleep until next interval
    chThdSleepUntil(time);
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
  UINT bytes;
  WriteMessage wm;
  FRESULT res = FR_OK;
  uint8_t * b;
  uint32_t write_errors = 0;

  chRegSetThreadName("WriteThread");
  res = f_open(&f_, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK) {
    chSysHalt(); while (1) {}   // couldn't properly open the file!
  }

  while (1) {
    Thread * calling_thread = chMsgWait();
    wm.message = chMsgGet(calling_thread);
    chMsgRelease(calling_thread, res);

    if (wm.message == 0) break;

    if (wm.m.buffer_selector & 2) 
      b = back_buffer_.data();
    else
      b = front_buffer_.data();

    res = f_write(&f_, b, wm.m.bytes_to_write, &bytes);
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
  if (!tp_write)
    m = (1 << 0);

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
  chThdWait(tp_control);
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

