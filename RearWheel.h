#ifndef REARWHEEL_H
#define REARWHEEL_H

#include <cstdint>
#include "Singleton.h"

class RearWheel : public Singleton<RearWheel> {
  friend class Singleton<RearWheel>;
 public:
  void Predict(uint32_t n);
  void PredictAndCorrect(uint32_t n, uint32_t counts);
  void SetCurrent(float current);
  float SpeedEstimate() const { return x_; }
  void setMotorEnabled(bool state);

 private:
  RearWheel();

  float A(uint32_t n);    /*! discrete state transition matrix              */
  float B(uint32_t n);    /*! discrete control input matrix                 */

  float x_,    /*! state estimate               [rad / s]                   */
        u_,    /*! input torque                 [N * m]                     */
        K_,    /*! Kalman gain                  [ ]                         */
        P_,    /*! error covariance             [rad^2 / s^2]               */
        Q_,    /*! process noise covariance     [rad^2 / s^2]               */
        R_;    /*! measurement noise covariance [rad^2 / s^2]               */

  uint32_t n_;          /*! time at last state prediction                   */
};
#endif
