#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

// Total size of buffer.
#define NUMBER_OF_SAMPLES 128

#include <cstddef>

#include "ff.h"

#include "Sample.h"
#include "Singleton.h"

class Thread;
// class FIL;

class SampleBuffer : public Singleton<SampleBuffer> {
  friend class Singleton<SampleBuffer>;
 public:
  Sample & CurrentSample();
  Sample & PreviousSample();
  SampleBuffer & operator++();
  void Flush();
  uint8_t Count() const {return i_;}
  uint8_t * BackBuffer();
  uint8_t * FrontBuffer();
  void setWriteThread(Thread * tp);
  inline void File(FIL * f) {f_ = f;}
  inline FIL * File(void) const {return f_;}

 private:
  SampleBuffer();
  SampleBuffer(const SampleBuffer &) = delete;
  SampleBuffer & operator=(const SampleBuffer &) = delete;

  static void WriteThread(void * arg) __attribute__((noreturn));

  uint8_t *buffer0_, *buffer1_;
  Thread * tp_;
  FIL * f_;
  Sample buffer_[NUMBER_OF_SAMPLES];
  WORKING_AREA(waWriteThread, 1024);
  uint32_t i_;
};
#endif // SAMPLEBUFFER_H
