#include "SampleBuffer.h"
#include "ch.h"

SampleBuffer * SampleBuffer::instance_ = 0;

SampleBuffer::SampleBuffer()
  : buffer0(reinterpret_cast<uint8_t *>(&(buffer[0]))),
    buffer1(reinterpret_cast<uint8_t *>(&(buffer[NUMBER_OF_SAMPLES/2]))),
    i_(0), tp_(NULL), LogData(true)
{

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
  ++i_;
  if (i_ == NUMBER_OF_SAMPLES) {
    i_ = 0;
    if (LogData)
      chMsgSend(tp_, 0);
  } else if (i_ == NUMBER_OF_SAMPLES / 2) {
    if (LogData)
      chMsgSend(tp_, 0);
  }

  return *this;
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

void SampleBuffer::EnableLogging()
{
  LogData = true;
}

void SampleBuffer::DisableLogging()
{
  LogData = false;
}

void SampleBuffer::setWriteThread(Thread * tp)
{
  tp_ = tp;
}
