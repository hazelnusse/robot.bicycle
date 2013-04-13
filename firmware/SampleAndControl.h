#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <cstddef>

#include "ch.h"
#include "hal.h"
#include "ff.h"

#include "Sample.h"
#include "Singleton.h"
#include "MPU6050.h"
#include "RearWheel.h"
#include "YawRateController.h"

#define NUMBER_OF_SAMPLES 256

class SampleAndControl : public Singleton<SampleAndControl> {
  friend class Singleton<SampleAndControl>;
 public:
  msg_t Start(const char * filename);
  msg_t Stop();

  const char * fileName() const;
  void sampleMotorState(Sample &s) const;
  uint32_t systemState() const;
  void enableSensorsMotors() const;
  void disableSensorsMotors() const;

  static void controlThread_(char *filename);
  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  SampleAndControl();
  SampleAndControl(const SampleAndControl &) = delete;
  SampleAndControl & operator=(const SampleAndControl &) = delete;

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  void controlThread(char* filename);
  static void writeThread_(void * arg);
  void writeThread();
  Sample* get_buffer(uint32_t index) const;
  FRESULT write_last_samples(uint32_t index);

  // Data collection related
  void sampleTimers(Sample & s);

  WORKING_AREA(waControlThread, 4096);
  WORKING_AREA(waWriteThread, 4096);
  Sample samples[NUMBER_OF_SAMPLES];
  FIL f_;
  Thread * tp_control;
  Thread * tp_write;
  uint32_t state_;
};

#include "SampleAndControl_priv.h"
#endif // SAMPLEANDCONTROL_H
