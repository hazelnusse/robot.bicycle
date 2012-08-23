#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H
#include <cstddef>
#include <cstdint>
#include "ch.h"
//#include "DiscreteStateSpace.h"

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
  static const float A[2]; // diagonal entries of 2x2
  static const float B[2]; // 2x1 column vector
  static const float C[2]; // 1x2 row vector
  static const float D;    // 1x1 scalar
  static float x[2];       // 2x1 column vector
  static float u;          // 1x1 scalar
  //DiscreteStateSpace<float, 1, 1, 2> PID;

};

typedef SpeedController<int32_t> IntSpeedController;
typedef SpeedController<float> FloatSpeedController;
typedef SpeedController<double> DoubleSpeedController;
#endif // SPEEDRATECONTROLLER_H
