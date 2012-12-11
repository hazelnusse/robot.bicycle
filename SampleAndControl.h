#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <cstddef>

#include "ch.h"
#include "ff.h"

#include "Sample.h"
#include "Singleton.h"


#define NUMBER_OF_SAMPLES 128

class SampleAndControl : public Singleton<SampleAndControl> {
  friend class Singleton<SampleAndControl>;
 public:
  msg_t Start(const char * filename);  // need to implement a way to pass filename const char *);
  msg_t Stop();

  static void controlThread_(void * arg);
  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  SampleAndControl();
  SampleAndControl(const SampleAndControl &) = delete;
  SampleAndControl & operator=(const SampleAndControl &) = delete;

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  void controlThread();
  static void writeThread_(void * arg);
  void writeThread();

  WORKING_AREA(waControlThread, 1024);
  WORKING_AREA(waWriteThread, 1024);
  Sample samples[NUMBER_OF_SAMPLES];
  FIL f_;
  char filename_[24];
  Thread * tp_control;
  Thread * tp_write;
};

#include "SampleAndControl_priv.h"
#endif // SAMPLEANDCONTROL_H
