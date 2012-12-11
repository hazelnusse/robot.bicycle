#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

// Total size of buffer.
#define NUMBER_OF_SAMPLES 128

#include <cstddef>

#include "ff.h"

#include "Sample.h"
#include "Singleton.h"

class Thread;

class SampleBuffer : public Singleton<SampleBuffer> {
  friend class Singleton<SampleBuffer>;
 public:
  Sample & CurrentSample();
  msg_t Increment();
  msg_t setFile(const char * filename);
  msg_t Reset();

 private:
  SampleBuffer();
  SampleBuffer(const SampleBuffer &) = delete;
  SampleBuffer & operator=(const SampleBuffer &) = delete;

  Sample * BackBuffer();
  Sample * FrontBuffer();
  static msg_t WriteThread_(void * arg);
  msg_t WriteThread();

  Sample buffer0_[NUMBER_OF_SAMPLES/2];
  Sample buffer1_[NUMBER_OF_SAMPLES/2];
  WORKING_AREA(waWriteThread, 1024);
  FIL f_;
  Thread * tp_;
  uint32_t i_;
};

#include "SampleBuffer_priv.h"

#endif // SAMPLEBUFFER_H
