#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

#include "ch.h"
#include "sample.pb.h"

namespace logging {

class SampleBuffer {
 public:
  SampleBuffer(const char * filename);
  ~SampleBuffer();
  bool insert(const Sample & s);
  msg_t flush_and_close();

 private:
  static msg_t write_thread(void * arg);
  msg_t exec(const char * filename);

  static SampleBuffer * instance_;

  Thread * tp_write_thread_;
  // The folowing members are shared between the write thread and the control
  // thread.  They are both read only in the write thread.
  uint16_t buffer_index_;  // Write thread uses once collection ends.
  uint8_t active_buffer_;  // File writes are triggered when this changes.
};

}

#include "sample_buffer-inl.h"

#endif

