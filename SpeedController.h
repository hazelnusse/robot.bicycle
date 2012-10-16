#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H
#include <cstddef>
#include <cstdint>
#include "ch.h"
#include "Singleton.h"

class Sample;

class SpeedController : public Singleton<SpeedController> {
  friend class Singleton<SpeedController>;
 public:
  void setEnabled(bool state);
  bool isEnabled() const { return Enabled_; }

  void SetPoint(const float speed) { SetPoint_ = speed; }
  float SetPoint() const { return SetPoint_; }

  void Update(Sample & s);

  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  SpeedController();
  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);

  float SetPoint_;  // Angular velocity set point [rad / s]

  float A[2];       // diagonal entries of 2x2 A matrix
  float B[2];       // 2x1 column vector
  float C[2];       // 1x2 row vector
  float D;          // 1x1 scalar
  float x[2];       // 2x1 column vector
  float u;          // 1x1 scalar

  bool Enabled_;
};

#endif // SPEEDCONTROLLER_H
