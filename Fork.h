#ifndef FORK_H
#define FORK_H

#include "ch.h"
#include "chprintf.h"
#include "Singleton.h"

class Fork : public Singleton<Fork> {
  friend class Singleton<Fork>;
 public:
  static void calibrate(BaseSequentialStream * chp, int argc, char * argv[]);
  static void home(BaseSequentialStream * chp, int argc, char * argv[]);
  void setMotorEnabled(bool state);

 private:
  Fork();
  void calibrateEncoder(BaseSequentialStream * chp);
  void homeFork(BaseSequentialStream * chp);

  int32_t offset_; // Calibration constant
};

#endif
