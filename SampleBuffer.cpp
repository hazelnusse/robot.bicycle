#include "SampleBuffer.h"
#include "ch.h"

SampleBuffer * SampleBuffer::instance_ = 0;

Sample SampleBuffer::buffer[NUMBER_OF_SAMPLES];
WORKING_AREA(SampleBuffer::waWriteThread, 1024);

SampleBuffer::SampleBuffer()
  : buffer0(reinterpret_cast<uint8_t *>(buffer)),
    buffer1(reinterpret_cast<uint8_t *>(buffer + NUMBER_OF_SAMPLES/2)),
    i_(0), tp_(NULL), f_(NULL)
{
  tp_ = chThdCreateStatic(waWriteThread,
                          sizeof(waWriteThread),
                          NORMALPRIO, (tfunc_t) WriteThread, NULL);


}

Sample & SampleBuffer::CurrentSample()
{
  return buffer[i_];
}

Sample & SampleBuffer::PreviousSample()
{
  return (i_ == 0) ? buffer[NUMBER_OF_SAMPLES - 1] : buffer[i_ - 1];
}

void SampleBuffer::HoldMagnetometer()
{
  for (int i = 0; i < 3; ++i)
    CurrentSample().mag[i] = PreviousSample().mag[i];
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

void * SampleBuffer::operator new(std::size_t, void * location)
{
  return location;
}

SampleBuffer & SampleBuffer::Instance()
{
  static uint8_t allocation[sizeof(SampleBuffer)];

  if (instance_ == 0)
      instance_ = new (allocation) SampleBuffer;

  return *instance_;
}

uint8_t * SampleBuffer::BackBuffer()
{
  return (i_ < (NUMBER_OF_SAMPLES/2)) ? buffer1 : buffer0;
}

uint8_t * SampleBuffer::FrontBuffer()
{
  return (i_ < (NUMBER_OF_SAMPLES/2)) ? buffer0 : buffer0;
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
