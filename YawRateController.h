#ifndef YAWRATECONTROLLER_H
#define YAWRATECONTROLLER_H
#include <cstddef>
#include <cstdint>
#include "ch.h"
#include "Singleton.h"

class Sample;

class YawRateController : public Singleton<YawRateController> {
  friend class Singleton<YawRateController>;
 public:
  void setEnabled(bool state);
  bool isEnabled() const { return Enabled_; }

  void SetPoint(const float speed) { SetPoint_ = speed; }
  float SetPoint() const { return SetPoint_; }

  void Update(Sample & s);
  
  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  YawRateController();
  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);

  float SetPoint_;
  
  bool Enabled_;
};

#endif // YAWRATECONTROLLER_H
