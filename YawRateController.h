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
  
  float RateCommanded() const;
  void RateCommanded(float yaw_rate);

  bool hasFault();
  
  uint32_t PWM_CCR() const;

  void Update(const Sample & s);
  
  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);
  static void calibrateSteerEncoder(BaseSequentialStream * chp,
                                    int argc, char * argv[]);
  static void homeFork(BaseSequentialStream * chp, int argc, char * argv[]);

 private:
  YawRateController();
  
  void setCurrentDirPositive();
  void setCurrentDirNegative();
  void setCurrent(float current);

  void Reset();

  void PWM_CCR(uint32_t ccr);
  uint32_t CurrentToCCR(float current);

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

  void calibrateSteerEncoder(BaseSequentialStream * chp);
  void homeFork(BaseSequentialStream * chp);

  int32_t offset_; /*! Calibration constant */
  bool homed_;     /*! Whether the fork has been homed */
  float u_,        /*! Applied current */
        r_,        /*! Commanded yaw rate */
        x_[5];     /*! Controller state */
};

#include "YawRateController_priv.h"

#endif // YAWRATECONTROLLER_H
