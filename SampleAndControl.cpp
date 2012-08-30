#include <cstring>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "ff.h"

#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5843.h"

#include "SampleAndControl.h"
#include "SampleBuffer.h"
#include "SpeedController.h"
#include "YawRateController.h"

// Class static data
SampleAndControl * SampleAndControl::instance_ = 0;
WORKING_AREA(SampleAndControl::waControlThread, 1024);
FIL SampleAndControl::f_;
EncoderTimers SampleAndControl::timers = {{0, 0, 0}};

SampleAndControl::SampleAndControl()
  : Control_tp_(NULL), Enabled_(false)
{
  SetFilename("samples.dat");
}

__attribute__((noreturn))
void SampleAndControl::Control(__attribute__((unused))void * arg)
{
  chRegSetThreadName("Control");

  /*
   * Initialize the StickIMU Sensors
   */
  ITG3200Init();
  ADXL345Init();
  HMC5843Init();

  SpeedController & speedControl = SpeedController::Instance();
  YawRateController & yawControl = YawRateController::Instance();
  SampleBuffer & sb = SampleBuffer::Instance();

  systime_t time = chTimeNow();     // Initial time

  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(5);            // Next deadline
    palTogglePad(IOPORT3, GPIOC_TIMING_PIN); // Sanity check for loop timing

    Sample & s = sb.CurrentSample();

    s.SystemTime = chTimeNow();

    ITG3200Acquire(s);
    ADXL345Acquire(s);
    // copy magnetometer signal to the current sample
    if (i % 4 == 0) {
      HMC5843Acquire(s);
    } else {
      sb.HoldMagnetometer();
    }

    s.SteerAngle = STM32_TIM3->CNT; // Capture encoder angle
    s.RearWheelRate = SampleAndControl::timers.Clockticks[0];
    s.FrontWheelRate = SampleAndControl::timers.Clockticks[1];
    s.SteerRate = SampleAndControl::timers.Clockticks[2];
    s.RearWheelRate_sp = speedControl.SetPoint();
    s.YawRate_sp = 0.0; // need to implement yaw rate controller

    // Compute new speed control action if controller is enabled.
    if (speedControl.Enabled() && ((i % 10) == 0))
      speedControl.Update(s);

    if (yawControl.Enabled()) {
      yawControl.Update(s);
    }

    s.CCR_rw = STM32_TIM1->CCR[1];    // Save rear wheel duty
    s.CCR_steer = STM32_TIM1->CCR[0]; // Save steer duty

    s.SystemState = 0;


    ++sb;                   // Increment to the next sample

    // Go to sleep until next 5ms interval
    chThdSleepUntil(time);
  }
  sb.Flush();  // ensure all data is written to disk
  chThdExit(0);
}

void SampleAndControl::chshellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (Enabled()) {
      Disable();
      if (Disabled()) {
        chprintf(chp, "Sample & Control thread disabled.\r\n");
      } else {
        chprintf(chp, "Unable to disable Sample & Control thread.\r\n");
      }
    } else {
      Enable();
      if (Enabled()) {
        chprintf(chp, "Sample & Control thread enabled.\r\n");
      } else {
        chprintf(chp, "Unable to enable Sample & Control thread.\r\n");
      }
    }
  } else if (argc == 1) { // change filename
    if (Enabled()) {
      chprintf(chp, "Disable control thread before changing files.\r\n");
    } else {
      SetFilename(argv[0]);
      chprintf(chp, "Filename changed to %s.\r\n", Filename_);
    }
  }
  return;
}

SampleAndControl & SampleAndControl::Instance()
{
  static uint8_t allocation[sizeof(SampleAndControl)];

  if (instance_ == 0)
    instance_ = new (allocation) SampleAndControl;

  return *instance_;
}


void * SampleAndControl::operator new(std::size_t, void * location)
{
  return location;
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
void SampleAndControl::Enable()
{
  if (f_open(&SampleAndControl::f_, Filename_, FA_CREATE_ALWAYS | FA_WRITE))
    return;

  SampleBuffer::Instance().File(&SampleAndControl::f_);

  nvicEnableVector(TIM4_IRQn, CORTEX_PRIORITY_MASK(7));

  Control_tp_ = chThdCreateStatic(SampleAndControl::waControlThread,
                                  sizeof(waControlThread),
                                  NORMALPRIO, (tfunc_t) Control, NULL);

  Enabled_ = true;
}

void SampleAndControl::Disable()
{
  SpeedController::Instance().Disable();
  chThdTerminate(Control_tp_);
  chThdWait(Control_tp_);
  Control_tp_ = NULL;
  nvicDisableVector(TIM4_IRQn);
  Enabled_ = false;
}

void SampleAndControl::SetFilename(const char * name)
{
  std::strcpy(Filename_, name);
}
