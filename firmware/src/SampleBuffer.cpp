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

msg_t SampleBuffer::manager_thread(const char *filename)
{
  chRegSetThreadName("manage");
  FRESULT res = f_open(&f_, filename, FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK)
    chThdExit(systemstate::FATFS_f_open_error);

  chSemInit(&buffer_sem_, 0);
  uint8_t active_buffer = 0;

  // Spawn the write thread
  tp_write_ = chThdCreateStatic(waWriteThread,
                                sizeof(waWriteThread),
                                NORMALPRIO + 1,
                                write_thread_,
                                &active_buffer);

  uint16_t buffer_index = 0;
  uint16_t excess_bytes = 0;
  while (1) {
    Thread * tp = chMsgWait();
    msg_t m = chMsgGet(tp);
    if (m == -1) {
      chMsgRelease(tp, 0);
      break;
    }

    //     A    B   C
    //   --------------
    //   | -1 | 0 | 1 |
    //   --------------
    // When using three buffers, if the manage thread is filling buffer A and the
    // write thread is waiting for buffer A to be filled, sem counter is -1.
    // When buffer A is filled and the manage thread is filling buffer B, the
    // sem counter is increased to 0, and the write thread is resumed. If buffer
    // B is filled, manage thread switches to buffer C, and increments the sem
    // counter. If the write thread is still writing data from buffer A, we want
    // increment the sem counter and set the active buffer for the manage thread 
    // to buffer A, but prevent it from encoding any samples.
    if (buffer_overflow()) {
      chMsgRelease(tp, false); // pb_encode returns false on failure
      continue;
    }

    // if we aren't overflowing, but we weren't able to copy over excess bytes at
    // the previous time, ignore the new sample and copy over excess bytes.
    if (excess_bytes > 0) {
      chMsgRelease(tp, false);
    } else {
      pb_ostream_t out = pb_ostream_from_buffer(
          &buffer_[active_buffer][buffer_index + 2],
          buffer_[active_buffer].size() - buffer_index - 2);
      Sample * s = reinterpret_cast<Sample *>(m);
      chMsgRelease(tp, pb_encode(&out, Sample_fields, s));

      buffer_[active_buffer][buffer_index] = out.bytes_written & 0xff;    // LSB
      buffer_[active_buffer][buffer_index + 1] = out.bytes_written >> 8;  // MSB
      buffer_index += out.bytes_written + 2;

      if (buffer_index >= bytes_per_buffer_) {
        chSemSignal(&buffer_sem_);
        excess_bytes = buffer_index - bytes_per_buffer_;
        buffer_index = excess_bytes;
        active_buffer = (active_buffer + 1) % number_of_buffers_;
      }
    }

    if (!buffer_overflow() && (excess_bytes > 0)) {
      uint8_t prev_buffer = ((active_buffer + number_of_buffers_ - 1) %
                             number_of_buffers_);
      memcpy(&buffer_[active_buffer][0],
             &buffer_[prev_buffer][bytes_per_buffer_],
             excess_bytes);
      excess_bytes = 0;
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

msg_t SampleBuffer::write_thread(uint8_t *start_buffer)
{
  chRegSetThreadName("write");
  UINT bytes_written = 0;
  msg_t write_errors = 0;
  uint8_t buffer_to_write = *start_buffer;

  while (1) {
    if (chSemWaitTimeout(&buffer_sem_, write_thread_sleep_time_) != RDY_OK) {
      if (chThdShouldTerminate())
        break;
      continue;
    }
    FRESULT res = f_write(&f_, &buffer_[buffer_to_write][0],
                          bytes_per_buffer_, &bytes_written);
    write_errors += (res != FR_OK) || (bytes_written != bytes_per_buffer_);
    buffer_to_write = (buffer_to_write + 1) % number_of_buffers_;
  }
  chThdExit(write_errors);
  return write_errors;
}

