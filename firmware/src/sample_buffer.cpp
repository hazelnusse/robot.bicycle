#include <array>
#include <cstdint>

#include "ff.h"
#include "pb_encode.h"

#include "constants.h"
#include "sample_buffer.h"
#include "SystemState.h"

namespace logging {

const uint16_t bytes_per_block = 512;
const uint16_t blocks_per_buffer = 32;
const uint16_t bytes_per_buffer = bytes_per_block * blocks_per_buffer;
const uint16_t buffer_cushion_bytes = 256;
const uint8_t number_of_buffers = 3;
const systime_t write_thread_sleep_time = MS2ST(10);

SampleBuffer * SampleBuffer::instance_;
std::array<std::array<uint8_t,
                      bytes_per_buffer + buffer_cushion_bytes>,
           number_of_buffers> buffer;

// Caller: control thread (highest priority)
SampleBuffer::SampleBuffer(const char * filename)
  : tp_write_thread_{0}, buffer_index_{0}, active_buffer_{0}
{
  static WORKING_AREA(waWriteThread, 4096);
  instance_ = this;
  tp_write_thread_ = chThdCreateStatic(waWriteThread,
                                       sizeof(waWriteThread),
                                       chThdGetPriority() - 1,
                                       write_thread,
                                       const_cast<char *>(filename));
}

// Caller: control thread (highest priority)
// This method is designed under the assumption that it is executed at the
// highest priority and will not be pre-empted by another thread, in particular
// by the write thread.
bool SampleBuffer::insert(const Sample & s)
{
  pb_ostream_t out = pb_ostream_from_buffer(
          &buffer[active_buffer_][buffer_index_ + 2],
          buffer[active_buffer_].size() - (buffer_index_ + 2));
  bool result = pb_encode(&out, Sample_fields, &s);
      
  buffer[active_buffer_][buffer_index_] = out.bytes_written & 0xff;    // LSB
  buffer[active_buffer_][buffer_index_ + 1] = out.bytes_written >> 8;  // MSB
  buffer_index_ += out.bytes_written + 2;

  if (buffer_index_ >= bytes_per_buffer) {
    buffer_index_ -= bytes_per_buffer;
    uint8_t prev_buffer = active_buffer_;
    active_buffer_ = (active_buffer_ + 1) % number_of_buffers;
    if (buffer_index_)
      memcpy(&buffer[active_buffer_][0],
             &buffer[prev_buffer][bytes_per_buffer],
             buffer_index_);
  }
  return result;
}

// Caller: control thread (highest priority)
msg_t SampleBuffer::flush_and_close()
{
  if (tp_write_thread_) {
    chThdTerminate(tp_write_thread_);
    msg_t write_errors = chThdWait(tp_write_thread_);
    tp_write_thread_ = 0;
    return write_errors;
  }
  return -1;
}

// Caller: write thread
msg_t SampleBuffer::exec(const char * filename)
{
  FIL f;
  FRESULT res = f_open(&f, filename, FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK) {
    tp_write_thread_ = 0;
    return systemstate::FATFS_f_open_error;
  }
  uint8_t buffer_to_write = 0;
  UINT bytes_written = 0;
  msg_t write_errors = 0;

  while (1) {
    if (chThdShouldTerminate())
      break;
    if (buffer_to_write == active_buffer_) {
      chThdSleep(chTimeNow() + MS2ST(constants::loop_period_ms));
      continue;
    }
    res = f_write(&f, &buffer[buffer_to_write][0],
                  bytes_per_buffer, &bytes_written);
    write_errors += (res != FR_OK) || (bytes_written != bytes_per_buffer);
    buffer_to_write = (buffer_to_write + 1) % number_of_buffers;
  }

  res = f_write(&f, &buffer[active_buffer_][0], buffer_index_, &bytes_written);
  write_errors += (res != FR_OK) || (bytes_written != bytes_per_buffer);
  res = f_close(&f);
  write_errors += (res != FR_OK);

  return write_errors;
}

}
