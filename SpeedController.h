#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H
#include <cstddef>
#include <cstdint>
#include "ch.h"
class Sample;

template <class T>
class SpeedController {
 public:
  static SpeedController & Instance();
  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  void Enable();
  bool Enabled() const;
  void Disable();
  bool Disabled() const;
  void SetPoint(const T speed);
  T SetPoint() const;
  T MinSetPoint() const;
  void MinSetPoint(T min);
  void Update(Sample & s);

 private:
  SpeedController();
  ~SpeedController();
  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static void *operator new(std::size_t, void * location);
  static void EnableHubMotor();
  static void DisableHubMotor();

  bool Enabled_;
  T SetPoint_;
  T MinSetPoint_;
  static SpeedController * instance_;

};

typedef SpeedController<int32_t> IntSpeedController;
typedef SpeedController<float> FloatSpeedController;
typedef SpeedController<double> DoubleSpeedController;
#endif // SPEEDRATECONTROLLER_H
