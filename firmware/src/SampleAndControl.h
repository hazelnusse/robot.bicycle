#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <array>
#include <cstddef>

#include "ch.h"
#include "hal.h"

#include "sample.pb.h"
#include "Singleton.h"
#include "MPU6050.h"
#include "RearWheel.h"
#include "YawRateController.h"
#include "SystemState.h"

class SampleAndControl : public Singleton<SampleAndControl> {
  friend class Singleton<SampleAndControl>;
 public:
  msg_t Start(const char * filename);
  msg_t Stop();

  const char * fileName() const;
  void sampleMotorState(Sample & s) const;
  uint32_t systemState() const;
  static void enableSensorsMotors();
  static void disableSensorsMotors();

  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  SampleAndControl();
  SampleAndControl(const SampleAndControl &) = delete;
  SampleAndControl & operator=(const SampleAndControl &) = delete;

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static void controlThread_(void * arg);
  void controlThread(const char * filename);

  // Data collection related
  static void sampleTimers(Sample & s);
  void sampleSetPoints(Sample & s);

  WORKING_AREA(waControlThread, 4096);
  
  Thread * tp_control_;
  uint32_t state_;
};

#include "SampleAndControl_priv.h"
#endif // SAMPLEANDCONTROL_H
