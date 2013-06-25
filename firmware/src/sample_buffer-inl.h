#ifndef SAMPLE_BUFFER_INL_H
#define SAMPLE_BUFFER_INL_H

namespace logging {

// Caller: initially called by OS when thread is created.
// Forwards call to the single SampleBuffer instance method and runs at
// priority one less than control thread
inline
msg_t SampleBuffer::write_thread(void * arg)
{
  chRegSetThreadName("write");
  return instance_->exec(static_cast<const char *>
                         (const_cast<const void *>(arg)));
}

// Caller: control thread (highest priority)
// Write thread must be terminated prior to the destructor call
inline
SampleBuffer::~SampleBuffer()
{
  instance_ = 0;
  tp_write_thread_ = 0;
}

}

#endif

