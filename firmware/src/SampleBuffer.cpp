#include "ch.h"
#include "pb_encode.h"
#include "SampleBuffer.h"
#include "SystemState.h"

const uint16_t SampleBuffer::bytes_per_block_;
const uint16_t SampleBuffer::blocks_per_buffer_;
const uint8_t SampleBuffer::number_of_buffers_;

SampleBuffer::SampleBuffer()
  : tp_write_(0), tp_manage_(0), active_buffer_(0)
{
}

// Caller: control_thread_
void SampleBuffer::initialize(const char * filename)
{
  tp_manage_ = chThdCreateStatic(waManagementThread,
                                 sizeof(waManagementThread),
                                 NORMALPRIO + 2,
                                 manager_thread_,
                                 const_cast<char *>(filename));
}

// Caller: control_thread_
msg_t SampleBuffer::deinitialize()
{
  chMsgSend(tp_manage_, -1); // ends the manager thread
  msg_t write_errors = chThdWait(tp_manage_);
  tp_manage_ = 0;
  return write_errors;
}

msg_t SampleBuffer::manager_thread(void * filename)
{
  chRegSetThreadName("manage");
  FRESULT res = f_open(&f_, static_cast<char *>(filename),
                       FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK)
    chThdExit(systemstate::FATFS_f_open_error);

  chMtxInit(&buffer_mtx_);

  // Spawn the write thread
  tp_write_ = chThdCreateStatic(waWriteThread,
                                sizeof(waWriteThread),
                                NORMALPRIO + 1,
                                write_thread_,
                                NULL);

  uint16_t buffer_index = 0;
  while (1) {
    Thread * tp = chMsgWait(); 
    msg_t m = chMsgGet(tp);
    if (m == -1) {
      chMsgRelease(tp, 0);
      break;
    }
    pb_ostream_t out = pb_ostream_from_buffer(
        &buffer_[active_buffer_][buffer_index + 2],
        buffer_[active_buffer_].size() - buffer_index - 2);
    Sample * s = reinterpret_cast<Sample *>(m);
    chMsgRelease(tp, pb_encode(&out, Sample_fields, s));

    buffer_[active_buffer_][buffer_index] = out.bytes_written;           // LSB
    buffer_[active_buffer_][buffer_index + 1] = out.bytes_written >> 8;  // MSB
    uint16_t packet_size = out.bytes_written + 2;

    if (buffer_index + packet_size < bytes_per_buffer_) {
      buffer_index += packet_size;
    } else {  // we've filled or overflowed the buffer
      uint16_t excess_bytes = buffer_index + packet_size  - bytes_per_buffer_;
      uint8_t next_buffer = (active_buffer_ + 1) % number_of_buffers_;
      memcpy(&buffer_[next_buffer][0],
             &buffer_[active_buffer_][bytes_per_buffer_],
             excess_bytes);
      buffer_index = excess_bytes;
      chMtxLock(&buffer_mtx_);
        active_buffer_ = next_buffer;
      chMtxUnlock();
    }
  }
  chThdTerminate(tp_write_);        // request that write thread terminate
  msg_t write_errors = chThdWait(tp_write_);
  tp_write_ = 0;

  // Write last bytes of partially filled buffer, then close file
  UINT bytes_written = 0;
  res = f_write(&f_, &buffer_[active_buffer_][0],
                buffer_index, &bytes_written);
  if ((res != FR_OK))
    ++write_errors;
  if (bytes_written != bytes_per_buffer_)
    ++write_errors;
  if (f_close(&f_) != FR_OK)
    ++write_errors;

  chThdExit(write_errors);
  return write_errors;
}

msg_t SampleBuffer::write_thread(void *)
{
  chRegSetThreadName("write");
  UINT bytes_written = 0;
  msg_t write_errors = 0;
  uint8_t buffer_to_write = 0;

  while (1) {
    if (chThdShouldTerminate())
      break;
    chMtxLock(&buffer_mtx_);
      bool buffer_filled = (active_buffer_ != buffer_to_write);
    chMtxUnlock();
    if (buffer_filled) {
      FRESULT res = f_write(&f_, &buffer_[buffer_to_write][0],
                            bytes_per_buffer_, &bytes_written);
      if ((res != FR_OK) || (bytes_written != bytes_per_buffer_))
        ++write_errors;

      buffer_to_write = (buffer_to_write + 1) % number_of_buffers_;
    } else {
      chThdSleep(MS2ST(10));
    }
  }
  chThdExit(write_errors);
  return write_errors;
}

