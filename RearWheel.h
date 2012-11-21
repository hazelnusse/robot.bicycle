#ifndef REARWHEEL_H
#define REARWHEEL_H

#include <cstdint>
#include "Singleton.h"
#include "bitband.h"

class RearWheel : public Singleton<RearWheel> {
  friend class Singleton<RearWheel>;
 public:
  // new stuff here
  void turnOn();
  void turnOff();
  bool isEnabled();

  void RateCommanded(float rearwheel_rate);
  float RateCommanded() const;
  float RateEstimate() const;

  bool hasFault();

  uint32_t PWM_CCR();
  uint32_t QuadratureCount();
  void setCurrent(float current); // make this private!!!

  void Update(uint32_t N_c);          // called whenever control law is updated 
  void Update(uint32_t N_m, float z); // called when a new measurement occurs

  static void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  RearWheel();

  float A(uint32_t dN) const;
  float B(uint32_t dN) const;
  float Q(uint32_t dN) const;

  void setDirPositive();
  void setDirNegative();

  void QuadratureCount(uint32_t count);
  void PWM_CCR(uint32_t ccr);

  uint32_t CurrentToCCR(float current);

  void cmd(BaseSequentialStream *chp, int argc, char *argv[]);

  uint32_t  N_,     /*! Timer count at most recent state estimate */
            N_c_;   /*! Timer count at fixed frequency update */
  float     r_,     /*! Reference angular velocity */
            u_,     /*! Applied current */
            x_,     /*! Most recent state estimate */
            x_c_,   /*! State estimate at fixed frequency update */
            K_,     /*! Kalman gain */
            P_;     /*! State estimate error covariance*/
  const float Q_,     /*! Process noise covariance (continuous) */
              R_;     /*! Measurement noise covariance (continuous) */
};

#include "RearWheel_priv.h"
#endif
