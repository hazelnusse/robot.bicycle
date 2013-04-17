#ifndef REARWHEEL_H
#define REARWHEEL_H

#include <cstdint>
#include "Singleton.h"
#include "Sample.h"

class RearWheel : public Singleton<RearWheel> {
  friend class Singleton<RearWheel>;
 public:
  void turnOn();
  void turnOff();
  bool isEnabled() const;

  void Reset();

  float RateCommanded() const;
  void RateCommanded(float rate);

  float RateEstimate() const;

  bool RotationDir() const;
  bool CurrentDir() const;

  void ProportionalGain(float kp);
  void IntegralGain(float ki);
  void DerivativeGain(float kd);

  void Update(const Sample & s);

  bool hasFault();

  uint32_t PWM_CCR() const;

  uint32_t QuadratureCount() const;
  void QuadratureCount(uint32_t count);

  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  RearWheel();

  void setCurrentDirPositive();
  void setCurrentDirNegative();
  void setCurrent(float current);

  void PWM_CCR(uint32_t ccr);

  uint32_t CurrentToCCR(float current);

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

  float u_,     /*! Applied current */
        r_,     /*! Commanded rear wheel rate */
        Kp_,    /*! Proportional gain */
        Ki_,    /*! Integral gain */
        e_int_, /*! Integral of error */
        z_;     /*! Most recent speed estimate */

  uint32_t SystemTime_prev_,    /*! System timer counts at previous update */
           RearWheelAngle_prev_;/*! Wheel quadrature counts at previous update */
};

#include "RearWheel_priv.h"
#endif

