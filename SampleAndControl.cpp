#include "ch.h"
#include "hal.h"
#include "chprintf.h"
// #include "ff.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "SampleAndControl.h"
#include "SampleBuffer.h"
// #include "YawRateController.h"

SampleAndControl::SampleAndControl()
  : tp_(NULL), Running_(false), stop_(false)
{
  tp_ = chThdCreateStatic(SampleAndControl::waControlThread,
                          sizeof(waControlThread),
                          NORMALPRIO, (tfunc_t) Control_, 0);
  // chMBInit(&mbox_, messages_, 10);
}

__attribute__((noreturn))
void SampleAndControl::Control_(__attribute__((unused))void * arg)
{
  SampleAndControl::Instance().Control();
}

void SampleAndControl::Control()
{
  // Things that get done only once
  chRegSetThreadName("Control");

  MPU6050 & imu = MPU6050::Instance();
  imu.Initialize(&I2CD2);

  RearWheel & rw = RearWheel::Instance();
  SampleBuffer & sb = SampleBuffer::Instance();
  // YawRateController & yc = YawRateController::Instance();

  // Runs forever, waiting for start or stop messages
  while (1) {
    Thread * messaging_tp = chMsgWait();    // wait until we get a message
    msg_t message = chMsgGet(messaging_tp); // retrieve the message
    chMsgRelease(messaging_tp, 0);          // release the thread that sent message

    if (message == 0) {
      Running_ = false;
      rw.turnOff();
    } else if (message == 1) { // Start collection
      Running_ = true;
      stop_ = false;
      // chMsgRelease(messaging_tp, 0);
      // zero out wheel encoders and system timer
      STM32_TIM4->CNT = STM32_TIM5->CNT = STM32_TIM8->CNT = 0;
      rw.turnOn();
      //yc.turnOn();
      systime_t time = chTimeNow();     // Initial time
      //uint32_t rw_fault_count = 0;
      //uint32_t steer_fault_count = 0;
      // Sampling loop, runs at 200Hz
      uint32_t i = 0;
      while (!stop_) {
        time += MS2ST(con::T_ms);       // Next deadline
        // check to see if we are suppsed to end data collection
        chSysLock();
        stop_ = chMsgIsPendingI(chThdSelf());
        chSysUnlock();

        // Get a sample to populate
        Sample & s = sb.CurrentSample();

        // Begin data collection
        s.SystemTime = STM32_TIM5->CNT;
        // s.RearWheelAngle = rw.QuadratureCount();
        // imu.Acquire(s);
        // End data collection

        // Increment the sample buffer
        // sb.Increment();
        ++i;

        // Go to sleep until next 5ms interval
        chThdSleepUntil(time);
      } // while @ 200Hz
    } // start collection
  } // while (1)
}

// Caller: Shell thread
void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[] __attribute__((unused)))
{
  if (argc == 0) {        // Start/Stop data collection, default filename
    if (isRunning()) {    // Data collection enabled
      chprintf(chp, "running.\r\n");
      Stop();             // Stop it
      chprintf(chp, "Data collection and control terminated.\r\n");
    } else {              // Data collecton disabled
      chprintf(chp, "not running.\r\n");
      Start();            // Start data collection to default file "samples.dat"
      chprintf(chp, "Data collection and control initiated.\r\n");
    }
  }
//  else if (argc == 1) { // Start/Stop data collection, with filename
//    if (isRunning()) {    // Data collection enabled
//      chprintf(chp, "running.\r\n");
//      Stop();             // Stop it, ignoring the argument
//    } else {              // Data collection is disabled
//      chprintf(chp, "not running.\r\n");
//      Start(argv[0]);     // Start data collection to filename in argv[0]
//    }
//  }
  else {
    chprintf(chp, "Invalid usage.\r\n");
  }
}

//void SampleAndControl::setFilename(const char * name)
//{
//  std::strcpy(Filename_, name);
//}


//    Sample & s = sb.CurrentSample();
//    uint32_t state = 0;
//    
//    s.SystemTime = STM32_TIM5->CNT;

    
    // Get rear wheel angle and rear wheel rotation direction
    // s.RearWheelAngle = rw.QuadratureCount();
    // state |= rw.RotationDir() | rw.CurrentDir();


//    s.FrontWheelAngle = STM32_TIM4->CNT;
//    s.SteerAngle = STM32_TIM3->CNT;
//
//    s.RearWheelRate_sp = rw.RateCommanded();
    // s.YawRate_sp = yc.RateCommanded();

    // Compute new speed control action if controller is enabled.
    // if (rw.isEnabled()) {
//      state |= Sample::RearWheelMotorEnable;
//      if (i % con::RW_N == 0) {    // only update control law every RW_N times
//        rw.Update(s.SystemTime, s.RearWheelAngle);
//      }
//     }

    // Compute new steer control action if yaw controller is enabled.
    //if (yc.isEnabled()) {
    //  state |= Sample::YawRateControl;
    //  yc.Update(s);
    //}

//    s.CCR_rw = rw.PWM_CCR();
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

//    s.SystemState = state;
  // sb.Reset();  // Throw away the data in the partially filled front buffer
  // rw.turnOff();
  // yc.turnOff();
  // imu.DeInitialize();
