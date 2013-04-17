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

  uint32_t RotationDir() const;
  uint32_t CurrentDir() const;
  int32_t SteerOffset() const;

  bool hasFault();

  uint32_t PWM_CCR() const;

  void Update(const Sample & s);

  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);
  static void calibrateSteerEncoder_(BaseSequentialStream * chp,
                                    int argc, char * argv[]);
  static void homeFork_(BaseSequentialStream * chp, int argc, char * argv[]);

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

  static CH_IRQ_HANDLER(CalibrationISR_);
  static CH_IRQ_HANDLER(homeISR_);

  int32_t offset_;  /*! Calibration constant */
  bool homed_,      /*! Whether the fork has been homed */
       estimation_triggered_,   /*! Whether estimator has been triggered */
       control_triggered_;      /*! Whether controller has been triggered */
  float u_,                 /*! Applied torque (output) */
        r_,                 /*! Reference yaw rate (input) */
        x_[cg::a_cols],     /*! Controller state */
        estimator_theta_R_dot_threshold_,  /*! Rear wheel rate estimator threshold */
        controller_theta_R_dot_threshold_; /*! Rear wheel rate controller threshold */

  static const std::array<cg::ControllerGains, cg::num_gains> gains_;
  const cg::ControllerGains * ar_[2];
  float alpha_;
};

#include "YawRateController_priv.h"

#endif // YAWRATECONTROLLER_H
