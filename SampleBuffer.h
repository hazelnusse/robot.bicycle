#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

// Total size of buffer.
#define NUMBER_OF_SAMPLES 128

#include <cstddef>

#include "ff.h"

#include "Sample.h"
class Thread;
// class FIL;

class SampleBuffer {
 public:
  static SampleBuffer & Instance();
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
  ~SampleBuffer();
  SampleBuffer & operator=(const SampleBuffer &) = delete;
  SampleBuffer(const SampleBuffer &) = delete;

  static void * operator new(std::size_t, void * location);
  static void WriteThread(void * arg) __attribute__((noreturn));

  uint8_t *buffer0, *buffer1;
  uint8_t i_;
  Thread * tp_;
  FIL * f_;
  static SampleBuffer * instance_;
  static Sample buffer[NUMBER_OF_SAMPLES];
  static WORKING_AREA(waWriteThread, 1024);
};
#endif // SAMPLEBUFFER_H
