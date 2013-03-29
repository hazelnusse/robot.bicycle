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

  return s;
}

#endif
