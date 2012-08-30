#ifndef YAWRATECONTROLLER_H
#define YAWRATECONTROLLER_H
#include <cstddef>
#include <cstdint>
#include "ch.h"

class Sample;

class YawRateController {
 public:
  static YawRateController & Instance();
  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  void Enable() { EnableSteerMotor(); Enabled_ = true; }
  bool Enabled() const { return Enabled_; }
  void Disable() { DisableSteerMotor(); Enabled_ = false; }
  bool Disabled() const { return !Enabled_; }
  void SetPoint(const float speed) { SetPoint_ = speed; }
  float SetPoint() const { return SetPoint_; }
  float MinSetPoint() const { return MinSetPoint_; }
  void MinSetPoint(float min) { MinSetPoint_ = min; }
  void Update(Sample & s);

 private:
  YawRateController();
  ~YawRateController();
  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static void *operator new(std::size_t, void * location);
  static void EnableSteerMotor();
  static void DisableSteerMotor();

  bool Enabled_;
  float SetPoint_;
  float MinSetPoint_;
  static YawRateController * instance_;
  static const float A[2]; // diagonal entries of 2x2
  static const float B[2]; // 2x1 column vector
  static const float C[2]; // 1x2 row vector
  static const float D;    // 1x1 scalar
  static float x[2];       // 2x1 column vector
  static float u;          // 1x1 scalar
};

#endif // YAWRATECONTROLLER_H

