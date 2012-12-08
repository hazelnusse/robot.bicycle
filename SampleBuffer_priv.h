#ifndef SAMPLEBUFFER_PRIV_H
#define SAMPLEBUFFER_PRIV_H

#include <cstring>

inline
msg_t SampleBuffer::Increment()
{
  return chMsgSend(tp_, 0);
}

inline
msg_t SampleBuffer::Reset()
{
  return chMsgSend(tp_, -1);
}

inline
msg_t SampleBuffer::setFile(const char * filename)
{
  return chMsgSend(tp_, reinterpret_cast<msg_t>(filename));
}


inline
Sample & SampleBuffer::CurrentSample()
{
  return FrontBuffer()[i_];
}


inline
Sample * SampleBuffer::BackBuffer()
{
  return (i_ < (NUMBER_OF_SAMPLES/2)) ? buffer1_ : buffer0_;
}

inline
Sample * SampleBuffer::FrontBuffer()
{
  return (i_ < (NUMBER_OF_SAMPLES/2)) ? buffer0_ : buffer0_;
}

inline
msg_t SampleBuffer::WriteThread_(__attribute__((unused))void * arg)
{
  return SampleBuffer::Instance().WriteThread();
}

#endif
