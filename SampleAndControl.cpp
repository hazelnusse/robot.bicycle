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

SampleAndControl * SampleAndControl::instance_ = 0;

SampleAndControl::SampleAndControl()
  : Control_tp_(0), Write_tp_(0), Enabled_(false)
{
  SetFilename("samples.dat");
}

msg_t SampleAndControl::Control(void * arg)
{
  chRegSetThreadName("Control");

  /*
   * Initialize the StickIMU Sensors
   */
  ITG3200Init();
  ADXL345Init();
  HMC5843Init();

  FloatSpeedController & speedControl = FloatSpeedController::Instance();
  SampleBuffer & sb = SampleBuffer::Instance();

  systime_t time = chTimeNow();     // Initial time

  for (int i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(5);            // Next deadline
    palTogglePad(IOPORT3, GPIOC_TIMING_PIN); // Sanity check for loop timing

    Sample & s = sb.CurrentSample();
    s.systemTime = chTimeNow();            // Store ChibioOS/RT clock
    ITG3200Acquire(s);
    ADXL345Acquire(s);
    (i++ % 4 == 0) ? HMC5843Acquire(s) : sb.HoldMagnetometer();
    s.steerAngle = STM32_TIM3->CNT; // Capture encoder angle
    s.steerRate = 0;                // need to setup place to store values of CCR1
    s.rearWheelRate = 0;            // need to setup place to store values of CCR2
    s.frontWheelRate = 0;           // need to setup place to store values of CCR1

    // Compute new speed control action if controller is enabled.
    if (speedControl.isEnabled() && (i % 20 == 0))
      speedControl.Update(s);

    ++sb;                   // Increment to the next sample

    // Otherwise go to sleep until next 5ms interval
    chThdSleepUntil(time);
  }
  return 0;
}

/*
 * SD Card writing thread, awoken from SampleBuffer increment operation
 */
msg_t SampleAndControl::WriteThread(void * arg)
{
  UINT bytes;
  static FIL f;
  SampleBuffer & sampleBuff = SampleBuffer::Instance();
  chRegSetThreadName("WriteThread");

  msg_t res = f_open(&f, static_cast<char *>(arg), FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK)
    return res;

  while (!chThdShouldTerminate()) {
    
    chMsgRelease(chMsgWait(), 0); // wait until there is data to write
    f_write(&f, sampleBuff.BackBuffer(), sizeof(Sample)*NUMBER_OF_SAMPLES/2, &bytes);
    f_sync(&f);
  }
  return f_close(&f);
}

void SampleAndControl::chshellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  SampleAndControl::Instance().shellcmd(chp, argc, argv);
}

void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) { // toggle enabled/disabled
    if (isEnabled()) {
      Disable();
      chprintf(chp, "Sample & Control thread disabled.\r\n");
    } else {
      Enable();
      chprintf(chp, "Sample & Control thread enabled.\r\n");
    }
  } else if (argc == 1) { // change filename
    SetFilename(argv[0]);
    chprintf(chp, "Filename changed to %s.\r\n", Filename_);
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

void SampleAndControl::Enable()
{
  static WORKING_AREA(waWriteThread, 256);
  Write_tp_ = chThdCreateStatic(waWriteThread,
                                sizeof(waWriteThread),
                                NORMALPRIO, WriteThread, Filename_);
  SampleBuffer::Instance().setWriteThread(Write_tp_);
  static WORKING_AREA(waControlThread, 256);
  Control_tp_ = chThdCreateStatic(waControlThread, sizeof(waControlThread),
                                  NORMALPRIO, Control, this);
  Enabled_ = true;
}

bool SampleAndControl::isEnabled() const
{
  return Enabled_;
}

void SampleAndControl::Disable()
{
  chThdTerminate(Control_tp_);
  chThdWait(Control_tp_);
  chThdTerminate(Write_tp_);
  chThdWait(Write_tp_);
  Enabled_ = false;
}

bool SampleAndControl::isDisabled() const
{
  return !Enabled_;
}

void SampleAndControl::SetFilename(const char * name)
{
  std::strcpy(Filename_, name);
}
