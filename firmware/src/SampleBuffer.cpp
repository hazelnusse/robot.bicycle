#include "ch.h"
#include "pb_encode.h"
#include "SampleBuffer.h"
#include "SystemState.h"

const uint16_t SampleBuffer::bytes_per_block_;
const uint16_t SampleBuffer::blocks_per_buffer_;
const uint8_t SampleBuffer::number_of_buffers_;

SampleBuffer::SampleBuffer()
  : tp_write_(0), tp_manage_(0)
{
}

// Caller: control_thread_
void SampleBuffer::initialize(const char * filename)
{
  if (tp_manage_ && !chThdTerminated(tp_manage_))
    chThdExit(systemstate::SampleBufferInitError);

  tp_manage_ = chThdCreateStatic(waManagementThread,
                                 sizeof(waManagementThread),
                                 NORMALPRIO + 1,
                                 manager_thread_,
                                 const_cast<char *>(filename));
}

// Caller: control_thread_
msg_t SampleBuffer::deinitialize()
{
  if (chThdTerminated(tp_manage_))
    chThdExit(tp_manage_->p_u.exitcode);

  msg_t m = chMsgSend(tp_manage_, -1);
  tp_manage_ = 0;
  return m;
}

// Caller: control_thread_
msg_t SampleBuffer::insert(Sample & s)
{
  if (chThdTerminated(tp_manage_))
    chThdExit(tp_manage_->p_u.exitcode);

  return chMsgSend(tp_manage_, reinterpret_cast<msg_t>(&s));
}

msg_t SampleBuffer::manager_thread(void * filename)
{
  chRegSetThreadName("manage");
  FRESULT res = f_open(&f_, static_cast<char *>(filename),
                       FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK)
    chThdExit(systemstate::FATFS_f_open_error);

  uint8_t current_buffer = 0;
  uint16_t i = 0;
  msg_t overflows = 0;
  while (1) {
    Thread * tp = chMsgWait(); 
    msg_t m = chMsgGet(tp);
    pb_ostream_t out = pb_ostream_from_buffer(&buffer_[current_buffer][i + 2],
                                              buffer_[current_buffer].size() - i - 2);
    Sample * s = reinterpret_cast<Sample *>(m);
    chMsgRelease(tp, pb_encode(&out, Sample_fields, s) ? 1 : 0);
    if (m == -1)
      break;

    // Comment to see whether timing is affected by encoding
    // continue;

    buffer_[current_buffer][i] = out.bytes_written;           // LSB
    buffer_[current_buffer][i + 1] = out.bytes_written >> 8;  // MSB
    uint16_t packet_size = out.bytes_written + 2;

    if (i + packet_size < bytes_per_buffer_) {
      i += packet_size;
    } else {
      if (!launch_write_thread(current_buffer))
        ++overflows;
      uint16_t excess_bytes = i + packet_size  - bytes_per_buffer_;
      uint8_t next_buffer = (current_buffer + 1) % number_of_buffers_;
      memcpy(&buffer_[next_buffer][0],
             &buffer_[current_buffer][bytes_per_buffer_],
             excess_bytes);
      i = excess_bytes;
      current_buffer = next_buffer;
    }
  }

  chThdWait(tp_write_);
  tp_write_ = 0;
  f_close(&f_);

  chThdExit(overflows);
  return overflows;
}

msg_t SampleBuffer::write_thread(void * arg)
{
  chRegSetThreadName("write");
  const uint8_t current_buffer = reinterpret_cast<uint32_t>(arg);
  UINT bytes_written = 0;
  msg_t write_errors = 0;

  FRESULT res = f_write(&f_, &buffer_[current_buffer][0],
                        bytes_per_buffer_, &bytes_written);
  if ((res != FR_OK) || (bytes_written != bytes_per_buffer_))
    ++write_errors;

  chThdExit(write_errors);
  return write_errors;
}

bool SampleBuffer::launch_write_thread(uint8_t current_buffer)
{
  if (tp_write_ && tp_write_->p_state != THD_STATE_FINAL)
    return false;

  tp_write_ = chThdCreateStatic(waWriteThread,
                                sizeof(waWriteThread),
                                NORMALPRIO + 1,
                                write_thread_,
                                reinterpret_cast<void *>(current_buffer));
  return true;
}

