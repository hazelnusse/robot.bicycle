#ifndef REARWHEEL_H
#define REARWHEEL_H

#include <cstdint>
#include "Singleton.h"
#include "bitband.h"

class RearWheel : public Singleton<RearWheel> {
  friend class Singleton<RearWheel>;
 public:
  void turnOn();
  void turnOff();
  bool isEnabled();

  void RateCommanded(float rearwheel_rate);
  float RateCommanded() const;

  bool hasFault();

  uint32_t PWM_CCR() const;
  uint32_t QuadratureCount() const;
  void setCurrent(float current); // make this private!!!

  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  RearWheel();

  void setDirPositive();
  void setDirNegative();

  void QuadratureCount(uint32_t count);
  void PWM_CCR(uint32_t ccr);

  uint32_t CurrentToCCR(float current);

  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);

  float u_, /*! Applied current */
        r_; /*! Commanded rate */
};

#include "RearWheel_priv.h"
#endif
