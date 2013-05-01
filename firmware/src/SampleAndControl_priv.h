#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
void SampleAndControl::controlThread_(void * arg)
{
  SampleAndControl::Instance().controlThread(arg);
}

inline
void SampleAndControl::sampleTimers(Sample & s)
{
  s.system_time = STM32_TIM5->CNT;
  s.encoder.rear_wheel_count = STM32_TIM8->CNT;
  s.encoder.rear_wheel = static_cast<int16_t>(s.encoder.rear_wheel_count) * cf::Wheel_rad_per_quad_count;
  s.encoder.steer = static_cast<int16_t>(STM32_TIM3->CNT) * cf::Steer_rad_per_quad_count;
  s.encoder.front_wheel = static_cast<int16_t>(STM32_TIM4->CNT) * cf::Wheel_rad_per_quad_count;
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
    s.system_state |= systemstate::RearWheelMotorEnable;
  if (yc.isEnabled())
    s.system_state|= systemstate::SteerMotorEnable;
  if (rw.hasFault())
    s.system_state |= systemstate::HubMotorFault;
  if (yc.hasFault())
    s.system_state |= systemstate::SteerMotorFault;
  if (rw.RotationDir())
    s.system_state |= systemstate::RearWheelEncoderDir;
  if (yc.RotationDir())
    s.system_state |= systemstate::SteerEncoderDir;
  if (rw.CurrentDir())
    s.system_state |= systemstate::RearWheelMotorCurrentDir;
  if (yc.CurrentDir())
    s.system_state |= systemstate::SteerMotorCurrentDir;

  // Motor current
  s.motor_current.rear_wheel = rw.CurrentCommanded();
  s.motor_current.steer = yc.CurrentCommanded();
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
  s.set_point.theta_R_dot = RearWheel::Instance().RateCommanded();
  s.set_point.psi_dot = YawRateController::Instance().RateCommanded();
}

#endif

