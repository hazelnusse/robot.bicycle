#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

#include "SampleBuffer.h"

inline
bool SampleAndControl::isRunning() const
{
  return Running_;
}

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
void SampleAndControl::Start(const char * filename)
{
  SampleBuffer::Instance().setFile(filename); // Blocks until msg received
  StartCollection();
}

inline
void SampleAndControl::StartCollection()
{
  chMsgSend(tp_, 1);            // Blocks until Control release message
}

inline
void SampleAndControl::Stop()
{
  StopCollection();
  SampleBuffer::Instance().Reset();  // Blocks until file is closed
}

inline
void SampleAndControl::StopCollection()
{
  chMsgSend(tp_, 0);
}
#endif
