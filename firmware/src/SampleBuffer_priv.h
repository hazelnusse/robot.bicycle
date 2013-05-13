#ifndef SAMPLEBUFFER_PRIV_H
#define SAMPLEBUFFER_PRIV_H

inline
msg_t SampleBuffer::write_thread_(void * arg)
{
  return SampleBuffer::Instance().write_thread(static_cast<uint8_t*>(arg));
}

inline
msg_t SampleBuffer::manager_thread_(void * arg)
{
  return SampleBuffer::Instance().manager_thread(static_cast<char *>(arg));
}

// Caller: control_thread_
inline
msg_t SampleBuffer::insert(Sample & s)
{
  return chMsgSend(tp_manage_, reinterpret_cast<msg_t>(&s));
}

inline
bool SampleBuffer::buffer_overflow()
{
  return chSemGetCounterI(&buffer_sem_) > number_of_buffers_ - 2;
}

#endif

