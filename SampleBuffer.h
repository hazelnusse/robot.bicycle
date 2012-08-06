#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

// Total size of buffer.
#define NUMBER_OF_SAMPLES 128

#include <cstddef>
#include "Sample.h"
class Thread;

class SampleBuffer {
 public:
  static SampleBuffer & Instance();
  Sample & CurrentSample();
  Sample & PreviousSample();
  void HoldMagnetometer();
  SampleBuffer & operator++();
  uint8_t * BackBuffer();
  void DisableLogging();
  void EnableLogging();
  void setWriteThread(Thread * tp);

 private:
  SampleBuffer();
  ~SampleBuffer();
  SampleBuffer & operator=(const SampleBuffer &) = delete;
  SampleBuffer(const SampleBuffer &) = delete;
  static void * operator new(std::size_t, void * location);
  static SampleBuffer * instance_;
  Sample buffer[NUMBER_OF_SAMPLES];
  uint8_t *buffer0, *buffer1;
  uint8_t i_;
  Thread * tp_;
  bool LogData;
};
#endif // SAMPLEBUFFER_H
