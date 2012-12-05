#ifndef REARWHEEL_H
#define REARWHEEL_H

#include <cstdint>
#include "Singleton.h"

class RearWheel : public Singleton<RearWheel> {
  friend class Singleton<RearWheel>;
 public:
  void turnOn();
  void turnOff();
  bool isEnabled() const;

  float RateCommanded() const;
  void RateCommanded(float rate);

  uint32_t RotationDir() const;
  uint32_t CurrentDir() const;

  void ProportionalGain(float kp);
  void IntegralGain(float ki);
  void DerivativeGain(float kd);

  void Update(uint32_t N, uint32_t cnt);

  bool hasFault();

  uint32_t PWM_CCR() const;

  uint32_t QuadratureCount() const;
  void QuadratureCount(uint32_t count);

  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  RearWheel();

  void setCurrentDirPositive();
  void setCurrentDirNegative();
  void setCurrent(float current);

  void Reset();

  void PWM_CCR(uint32_t ccr);

  uint32_t CurrentToCCR(float current);

  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);

  float u_,     /*! Applied current */
        r_,     /*! Commanded rear wheel rate */
        Kp_,    /*! Proportional gain */
        Ki_,    /*! Integral gain */
        Kd_,    /*! Derivative gain */
        e_int_; /*! Integral of error */

  uint32_t N_,  /*! System timer counts at most recent update */
           cnt_;/*! Wheel quadrature counts at most recent update */
};

#include "RearWheel_priv.h"
#endif
