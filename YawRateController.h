#ifndef YAWRATECONTROLLER_H
#define YAWRATECONTROLLER_H

#include <cstdint>
#include "ch.h"
#include "Singleton.h"

class Sample;

class YawRateController : public Singleton<YawRateController> {
  friend class Singleton<YawRateController>;
 public:
  void turnOn();
  void turnOff();
  bool isEnabled() const;
  void set_disturb_enabled(bool enable);
  bool disturb_enabled() const;

  void Reset();

  float RateCommanded() const;
  void RateCommanded(float yaw_rate);
  void set_disturb_amp(float amp);
  float disturb_amp() const;
  void set_disturb_freq(float freq);
  float disturb_freq() const;

  uint32_t RotationDir() const;
  uint32_t CurrentDir() const;
  int32_t SteerOffset() const;

  bool hasFault();

  uint32_t PWM_CCR() const;

  void Update(const Sample & s);

  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);
  static void ShellCmdDisturb_(BaseSequentialStream *chp, int argc, char *argv[]);
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
  void ShellCmdDisturb(BaseSequentialStream *chp, int argc, char *argv[]);

  void calibrateSteerEncoder(BaseSequentialStream * chp);
  void homeFork(BaseSequentialStream * chp);

  void SteerOffset(int32_t N);

  static CH_IRQ_HANDLER(CalibrationISR_);
  static CH_IRQ_HANDLER(homeISR_);

  int32_t offset_; /*! Calibration constant */
  bool homed_;     /*! Whether the fork has been homed */
  float u_,        /*! Applied current */
        r_,        /*! Commanded yaw rate */
        x_[5];     /*! Controller state */

  /* variables for controller disturbance */
  bool disturb_enabled_;
  float disturb_amp_;
  float disturb_freq_;
};

#include "YawRateController_priv.h"

#endif // YAWRATECONTROLLER_H
