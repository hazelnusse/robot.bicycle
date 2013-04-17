#ifndef YAWRATECONTROLLER_H
#define YAWRATECONTROLLER_H

#include <array>
#include <cstdint>
#include "ch.h"
#include "Sample.h"
#include "Singleton.h"
#include "ControllerGains.h"

class YawRateController : public Singleton<YawRateController> {
  friend class Singleton<YawRateController>;
 public:
  void turnOn();
  void turnOff();
  bool isEnabled() const;

  void Reset();

  float RateCommanded() const;
  void RateCommanded(float yaw_rate);

  float EstimationThreshold() const;
  void EstimationThreshold(float thresh);

  float ControlThreshold() const;
  void ControlThreshold(float thresh);

  bool isPIEnabled() const;
  void EnablePI();
  void DisablePI();

  bool RotationDir() const;
  bool CurrentDir() const;
  int32_t SteerOffset() const;

  bool hasFault();

  uint32_t PWM_CCR() const;

  void Update(const Sample & s);

  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);
  static void calibrateSteerEncoder_(BaseSequentialStream * chp,
                                    int argc, char * argv[]);
  static void homeFork_(BaseSequentialStream * chp, int argc, char * argv[]);
  static void setEstimationThreshold(BaseSequentialStream * chp, int argc, char * argv[]);
  static void setControlThreshold(BaseSequentialStream * chp, int argc, char * argv[]);
  static void togglePI(BaseSequentialStream * chp, int argc, char * argv[]);

 private:
  YawRateController();

  void setCurrentDirPositive();
  void setCurrentDirNegative();
  void setCurrent(float current);

  void PWM_CCR(uint32_t ccr);
  uint32_t CurrentToCCR(float current);

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

  void calibrateSteerEncoder(BaseSequentialStream * chp);
  void homeFork(BaseSequentialStream * chp);

  void SteerOffset(int32_t N);

  bool lu_bounds(float theta_R_dot);
  bool state_estimate_update(float theta_R_dot, const float input[cg::b_cols]);
  float control_output_update();
  void interpolate_PI_gains(float & Kp, float & Ki) const;


  static CH_IRQ_HANDLER(CalibrationISR_);
  static CH_IRQ_HANDLER(homeISR_);

  static const std::array<cg::ControllerGains, cg::num_gains> gains_;

  int32_t offset_;  /*! Calibration constant */
  bool homed_,      /*! Whether the fork has been homed */
       estimation_triggered_,   /*! Whether estimator has been triggered */
       control_triggered_,      /*! Whether controller has been triggered */
       PI_enabled_;
  float u_,                 /*! Applied torque (output) */
        r_,                 /*! Reference yaw rate (input) */
        x_[cg::a_cols],     /*! Controller state */
        estimator_theta_R_dot_threshold_,  /*! Rear wheel rate estimator threshold */
        controller_theta_R_dot_threshold_; /*! Rear wheel rate controller threshold */

  const cg::ControllerGains * ar_[2];
  float alpha_;
  float x_pi_;
  uint32_t SystemTime_prev_;
};

#include "YawRateController_priv.h"

#endif // YAWRATECONTROLLER_H
