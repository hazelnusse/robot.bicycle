#ifndef SAMPLEANDCONTROL_PRIV_H
#define SAMPLEANDCONTROL_PRIV_H

inline
void SampleAndControl::shellcmd_(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

inline
msg_t SampleAndControl::Stop()
{
  msg_t m;
  chThdTerminate(tp_control);
  m = chThdWait(tp_control);
  tp_control = 0;
  return m;
}

__attribute__((noreturn))
inline
void SampleAndControl::controlThread_(__attribute__((unused))void * arg)
{
  SampleAndControl::Instance().controlThread();
}

__attribute__((noreturn))
inline
void SampleAndControl::writeThread_(__attribute__((unused)) void * arg)
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
  s.CCR_rw = STM32_TIM1->CCR[0];      // RW PWM duty cycle
  s.CCR_steer = STM32_TIM1->CCR[1];    // Steer PWM duty cycle
}

inline
const char * SampleAndControl::fileName() const
{
  return filename_;
}

inline
uint32_t SampleAndControl::systemState() const
{
  return state_;
}

inline
uint32_t SampleAndControl::sampleSystemState() const
{
  uint32_t s;
  RearWheel & rw = RearWheel::Instance();
  YawRateController & yc = YawRateController::Instance();

  if (rw.isEnabled())
    s = Sample::RearWheelMotorEnable;
  if (yc.isEnabled())
    s |= Sample::SteerMotorEnable;
  if (rw.hasFault())
    s |= Sample::HubMotorFault;
  if (yc.hasFault())
    s |= Sample::SteerMotorFault;
  s |= rw.RotationDir();
  s |= yc.RotationDir();
  //TODO s |= fw.RotationDir();//TODO
  s |= rw.CurrentDir();
  s |= yc.CurrentDir();

  return s;
}

#endif
