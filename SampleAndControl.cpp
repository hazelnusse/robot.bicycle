#include <cstring>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "ff.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "SampleAndControl.h"
#include "SampleBuffer.h"
//#include "YawRateController.h"

SampleAndControl::SampleAndControl()
  : Control_tp_(0), Enabled_(false)
{
  setFilename("samples.dat");
}

void SampleAndControl::Control_(__attribute__((unused))void * arg)
{
  chRegSetThreadName("Control");
  SampleAndControl::Instance().Control();
}

void SampleAndControl::Control()
{
  MPU6050 & imu = MPU6050::Instance();
  imu.Initialize(&I2CD2);

  SampleBuffer & sb = SampleBuffer::Instance();
  RearWheel & rw = RearWheel::Instance();
  rw.QuadratureCount(0);
  //YawRateController & yc = YawRateController::Instance();

  systime_t time = chTimeNow();     // Initial time
  uint32_t state = 0;
  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(con::T_ms);                        // Next deadline
    // palTogglePad(IOPORT6, GPIOF_TIMING_PIN); // Sanity check for loop timing

    Sample & s = sb.CurrentSample();

    imu.Acquire(s);
    s.RearWheelAngle = rw.QuadratureCount();
    s.FrontWheelAngle = STM32_TIM4->CNT;
    s.SteerAngle = STM32_TIM3->CNT;
    s.SystemTime = STM32_TIM5->CNT;


    s.RearWheelRate_sp = rw.RateCommanded();
    s.YawRate_sp = 0.0; // need to implement yaw rate controller

    // Compute new speed control action if controller is enabled.
    if (rw.isEnabled()) {
      state |= Sample::SpeedControl;
      if (i % con::RW_N == 0) {    // only update control law every RW_N times
        rw.Update(s.SystemTime, s.RearWheelAngle);
      }
    }

    // Compute new steer control action if yaw controller is enabled.
    //if (yc.isEnabled()) {
    //  state |= Sample::YawRateControl;
    //  yc.Update(s);
    //}

    s.CCR_rw = rw.PWM_CCR();
    s.CCR_steer = STM32_TIM1->CCR[1]; // Save steer duty

    bool faults = false;
    // Check for motor faults
    if (rw.hasFault()) {
      state |= Sample::HubMotorFault;
      faults = true;
    }
    
    if (!palReadPad(GPIOF, GPIOF_STEER_FAULT)) {
      state |= Sample::SteerMotorFault;
      faults = true;
    }

    s.SystemState = state;
    state = 0;

    // Increment the sample buffer
    ++sb;

    // Turn of motors if there is a fault condition
    if (faults) {
      rw.turnOff();
      //yc.turnOff();
      break;
    }

    // Go to sleep until next 5ms interval
    chThdSleepUntil(time);
  }
  sb.Flush();  // ensure all data is written to disk
  imu.DeInitialize();
  chThdExit(0);
}

void SampleAndControl::chshellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      setEnabled(false);
      chprintf(chp, "Sample & Control thread disabled.\r\n");
    } else {
      setEnabled(true);
      chprintf(chp, "Sample & Control thread enabled.\r\n");
    }
  } else if (argc == 1) { // change filename
    if (isEnabled()) {
      setEnabled(false);
      chprintf(chp, "Sample & Control thread disabled.\r\n");
    }
    setFilename(argv[0]);
    chprintf(chp, "Filename changed to %s.\r\n", Filename_);
  }
  return;
}

/*
 * This function should:
 * 1) Open a FIL object in write mode with filename Filename_
 *   a) if unable, return without Enabling
 * 2) Get a reference to SampleBuffer call connectToFile(&f);
 *   a) file must be Open and Writable for this to work.  It is up to client of
 *      SampleBuffer to ensure this is the case, no error checking is
 *      performed.
 * 3) Start the control thread.
 */
void SampleAndControl::setEnabled(bool state)
{
  if (state) {
    if (f_open(&f_, Filename_, FA_CREATE_ALWAYS | FA_WRITE)) return;

    SampleBuffer::Instance().File(&f_);

    Control_tp_ = chThdCreateStatic(SampleAndControl::waControlThread,
                                    sizeof(waControlThread),
                                    NORMALPRIO, (tfunc_t) Control_, 0);

    Enabled_ = true;
  } else {
    // RearWheel::Instance().turnOff();
    // YawRateController::Instance().setEnabled(false);
    chThdTerminate(Control_tp_);
    chThdWait(Control_tp_);
    Control_tp_ = 0;
    Enabled_ = false;
  }
}

void SampleAndControl::setFilename(const char * name)
{
  std::strcpy(Filename_, name);
}
