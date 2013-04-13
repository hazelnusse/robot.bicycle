#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
void SampleAndControl::controlThread_(char *filename)
{
  SampleAndControl::Instance().controlThread(filename);
}

inline
void SampleAndControl::writeThread_(void *)
{
  SampleAndControl::Instance().writeThread();
}

inline
void SampleAndControl::sampleTimers(Sample & s)
{
  s.SystemTime = STM32_TIM5->CNT;
  s.RearWheelAngle = STM32_TIM8->CNT;
  s.SteerAngle = static_cast<int16_t>(STM32_TIM3->CNT);
  s.FrontWheelAngle = STM32_TIM4->CNT;
}

inline
uint32_t SampleAndControl::systemState() const
{
  return state_;
}

inline
void SampleAndControl::sampleMotorState(Sample & s) const
{
  RearWheel & rw = RearWheel::Instance();
  YawRateController & yc = YawRateController::Instance();

  // motor flags
  if (rw.isEnabled())
    s.SystemState |= Sample::RearWheelMotorEnable;
  if (yc.isEnabled())
    s.SystemState|= Sample::SteerMotorEnable;
  if (rw.hasFault())
    s.SystemState |= Sample::HubMotorFault;
  if (yc.hasFault())
    s.SystemState |= Sample::SteerMotorFault;
  if (rw.RotationDir())
    s.SystemState |= Sample::RearWheelEncoderDir;
  if (yc.RotationDir())
    s.SystemState |= Sample::SteerEncoderDir;
  if (rw.CurrentDir())
    s.SystemState |= Sample::RearWheelMotorCurrentDir;
  if (yc.CurrentDir())
    s.SystemState |= Sample::SteerMotorCurrentDir;

  // set points
  s.RearWheelRate_sp = rw.RateCommanded();
  s.YawRate_sp = yc.RateCommanded();

  // duty cycle
  s.CCR_rw = STM32_TIM1->CCR[0];      // RW PWM duty cycle
  s.CCR_steer = STM32_TIM1->CCR[1];   // Steer PWM duty cycle
}

inline
void SampleAndControl::enableSensorsMotors() const
{
  MPU6050 & imu = MPU6050::Instance();
  if (!imu.Initialize(&I2CD2)) {
    chSysHalt(); while(1) {}    // couldn't initialize the MPU6050
  }

  RearWheel & rw = RearWheel::Instance();
  rw.Reset();
  rw.turnOn();
  YawRateController & yc = YawRateController::Instance();
  yc.Reset();
  yc.turnOn();
}

inline
void SampleAndControl::disableSensorsMotors() const
{
  MPU6050 & imu = MPU6050::Instance();
  RearWheel & rw = RearWheel::Instance();
  YawRateController & yc = YawRateController::Instance();
  imu.DeInitialize();
  rw.turnOff();
  yc.turnOff();
}

#endif
