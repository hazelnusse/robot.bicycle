#include "SampleBuffer.h"
#include "ch.h"

SampleBuffer::SampleBuffer()
  : buffer0_(reinterpret_cast<uint8_t *>(buffer_)),
    buffer1_(reinterpret_cast<uint8_t *>(buffer_ + NUMBER_OF_SAMPLES/2)),
    tp_(0), f_(0), i_(0)
{
  tp_ = chThdCreateStatic(waWriteThread,
                          sizeof(waWriteThread),
                          NORMALPRIO, (tfunc_t) WriteThread, NULL);
}

Sample & SampleBuffer::CurrentSample()
{
  return buffer_[i_];
}

Sample & SampleBuffer::PreviousSample()
{
  return (i_ == 0) ? buffer_[NUMBER_OF_SAMPLES - 1] : buffer_[i_ - 1];
}

SampleBuffer & SampleBuffer::operator++()
{
  if (++i_ == NUMBER_OF_SAMPLES) {
    i_ = 0;
    chMsgSend(tp_, 0);  // triggers a call to f_write() on back buffer
  } else if (i_ == NUMBER_OF_SAMPLES / 2) {
    chMsgSend(tp_, 0);  // triggers a call to f_write() on back buffer
  }

  return *this;
}

void SampleBuffer::Flush()
{
  chMsgSend(tp_, 1);  // triggers a call to f_write on front buffer
}

uint8_t * SampleBuffer::BackBuffer()
{
  return (i_ < (NUMBER_OF_SAMPLES/2)) ? buffer1_ : buffer0_;
}

uint8_t * SampleBuffer::FrontBuffer()
{
  return (i_ < (NUMBER_OF_SAMPLES/2)) ? buffer0_ : buffer0_;
}

void SampleBuffer::setWriteThread(Thread * tp)
{
  tp_ = tp;
}

__attribute__((noreturn))
void SampleBuffer::WriteThread(__attribute__((unused))void * arg)
{
  UINT bytes;
  chRegSetThreadName("WriteThread");
  SampleBuffer & sb = SampleBuffer::Instance();

  while (true) {
    if (!sb.File()) {
      chThdYield();
      continue;
    }
    Thread * messaging_tp = chMsgWait();
    msg_t message = chMsgGet(messaging_tp);
    chMsgRelease(messaging_tp, 0);
    if (message == 0) { // write the back buffer
      f_write(sb.File(), sb.BackBuffer(),
              sizeof(Sample)*NUMBER_OF_SAMPLES/2, &bytes);
      f_sync(sb.File());
    } else if (message == 1) { // flush the front buffer
      f_write(sb.File(), sb.FrontBuffer(),
              sizeof(Sample)*sb.Count(), &bytes);
    }
  }
}
