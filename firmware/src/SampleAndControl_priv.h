#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
void SampleAndControl::controlThread_(void*)
{
  SampleAndControl::Instance().controlThread();
}

inline
void SampleAndControl::writeThread_(char* filename)
{
  SampleAndControl::Instance().writeThread(filename);
}

inline
void SampleAndControl::sampleTimers(Sample & s)
{
  s.SystemTime = STM32_TIM5->CNT;
  s.encoder.RearWheelCount = STM32_TIM8->CNT;
  s.encoder.RearWheel = static_cast<int16_t>(s.encoder.RearWheelCount) * cf::Wheel_rad_per_quad_count;
  s.encoder.Steer = static_cast<int16_t>(STM32_TIM3->CNT) * cf::Steer_rad_per_quad_count;
  s.encoder.FrontWheel = static_cast<int16_t>(STM32_TIM4->CNT) * cf::Wheel_rad_per_quad_count;
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
    s.SystemState |= systemstate::RearWheelMotorEnable;
  if (yc.isEnabled())
    s.SystemState|= systemstate::SteerMotorEnable;
  if (rw.hasFault())
    s.SystemState |= systemstate::HubMotorFault;
  if (yc.hasFault())
    s.SystemState |= systemstate::SteerMotorFault;
  if (rw.RotationDir())
    s.SystemState |= systemstate::RearWheelEncoderDir;
  if (yc.RotationDir())
    s.SystemState |= systemstate::SteerEncoderDir;
  if (rw.CurrentDir())
    s.SystemState |= systemstate::RearWheelMotorCurrentDir;
  if (yc.CurrentDir())
    s.SystemState |= systemstate::SteerMotorCurrentDir;

  // Motor current
  s.motorcurrent.RearWheel = rw.CurrentCommanded();
  s.motorcurrent.Steer = yc.CurrentCommanded();
}

inline
void SampleAndControl::enableSensorsMotors()
{
  if (!MPU6050::Instance().Initialize(&I2CD2))
    chSysHalt();

  RearWheel::Instance().turnOn();
  YawRateController::Instance().turnOn();
}

inline
void SampleAndControl::disableSensorsMotors()
{
  MPU6050::Instance().DeInitialize();
  RearWheel::Instance().turnOff();
  YawRateController::Instance().turnOff();
}

inline
void SampleAndControl::sampleSetPoints(Sample & s)
{
  s.setpoint.theta_R_dot = RearWheel::Instance().RateCommanded();
  s.setpoint.psi_dot = YawRateController::Instance().RateCommanded();
}

#endif

