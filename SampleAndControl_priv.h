#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

#include "SampleBuffer.h"

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
msg_t SampleAndControl::Stop()
{
  msg_t m;
  chThdTerminate(tp_control);
  m = chThdWait(tp_control);
  tp_control = 0;
  return m;
}

__attribute__((noreturn))
inline
void SampleAndControl::controlThread_(__attribute__((unused))void * arg)
{
  SampleAndControl::Instance().controlThread();
}

__attribute__((noreturn))
inline
void SampleAndControl::writeThread_(__attribute__((unused)) void * arg)
{
  SampleAndControl::Instance().writeThread();
}


#endif
