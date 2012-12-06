#include <cstring>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "ff.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "SampleAndControl.h"
#include "SampleBuffer.h"
// #include "YawRateController.h"

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
  Enabled_ = true;
  MPU6050 & imu = MPU6050::Instance();
  imu.Initialize(&I2CD2);

  SampleBuffer & sb = SampleBuffer::Instance();

  RearWheel & rw = RearWheel::Instance();
  rw.QuadratureCount(0);
  rw.turnOn();

  // YawRateController & yc = YawRateController::Instance();
  // yc.turnOn();

//  uint32_t rw_fault_count = 0;
//  uint32_t steer_fault_count = 0;
  STM32_TIM4->CNT = 0;              // zero out front wheel count
  STM32_TIM5->CNT = 0;              // zero out the free running timer

  systime_t time = chTimeNow();     // Initial time
  // Main loop
  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(con::T_ms);                        // Next deadline

    Sample & s = sb.CurrentSample();
    uint32_t state = 0;
    
    s.SystemTime = STM32_TIM5->CNT;

    imu.Acquire(s);
    
    // Get rear wheel angle and rear wheel rotation direction
    s.RearWheelAngle = rw.QuadratureCount();
    state |= rw.RotationDir() | rw.CurrentDir();


    s.FrontWheelAngle = STM32_TIM4->CNT;
    s.SteerAngle = STM32_TIM3->CNT;

    s.RearWheelRate_sp = rw.RateCommanded();
    // s.YawRate_sp = yc.RateCommanded();

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
    //s.CCR_steer = yc.PWM_CCR();

    // Check for motor faults
//    if (rw.hasFault()) {
//      state |= Sample::HubMotorFault;
//      ++rw_fault_count;
//      if (rw_fault_count > 10)  // debounce
//        break;
//    } else {
//      rw_fault_count = 0;
//    }
//    
//    if (yc.hasFault()) {
//      state |= Sample::SteerMotorFault;
//      ++steer_fault_count;
//      if (steer_fault_count > 10)  // debounce
//        break;
//    } else {
//      steer_fault_count = 0;
//    }

    s.SystemState = state;

    // Increment the sample buffer
    ++sb;

    // Go to sleep until next 5ms interval
    chThdSleepUntil(time);
  }
  sb.Flush();  // ensure all data is written to disk
  imu.DeInitialize();
  rw.turnOff();
  // yc.turnOff();
  Enabled_ = false;
  f_close(&f_);
  chThdExit(0);
}

void SampleAndControl::chshellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[] __attribute__((unused)))
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      setEnabled(false);
      chprintf(chp, "Sample & Control thread disabled.\r\n");
    } else {
      setEnabled(true);
      chprintf(chp, "Sample & Control thread enabled.\r\n");
    }
  } else {
    chprintf(chp, "Invalid usage.\r\n");
  }
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

  } else {
    chThdTerminate(Control_tp_);
  }
}

void SampleAndControl::setFilename(const char * name)
{
  std::strcpy(Filename_, name);
}
