#include <cstring>

#include "SampleAndControl.h"
#include "chprintf.h"

#include "MPU6050.h"
#include "RearWheel.h"
#include "YawRateController.h"

SampleAndControl::SampleAndControl()
  : tp_control(0), tp_write(0)
{
}

void SampleAndControl::controlThread()
{
  chRegSetThreadName("Control");
  FRESULT res;
  msg_t msg;

  res = f_open(&f_, filename_, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK) {
    chSysHalt(); while (1) {}   // couldn't properly open the file!
  }

  for (uint32_t i = 0; i < NUMBER_OF_SAMPLES; ++i)
    clearSample(samples[i]);

  MPU6050 & imu = MPU6050::Instance();
  imu.Initialize(&I2CD2);

  RearWheel & rw = RearWheel::Instance();
  rw.Reset();
  rw.turnOn();
  YawRateController & yc = YawRateController::Instance();
  yc.Reset();
  yc.turnOn();

  // zero out wheel encoders and system timer
  STM32_TIM4->CNT = STM32_TIM5->CNT = STM32_TIM8->CNT = 0;
  bool data = false;                // flag to indicate we have data to write
  uint32_t write_errors = 0;
  //uint32_t rw_fault_count = 0;
  //uint32_t steer_fault_count = 0;

  systime_t time = chTimeNow();     // Initial time
  for (uint32_t i = 0; !chThdShouldTerminate(); ++i) {
    time += MS2ST(con::T_ms);       // Next deadline

    // Get a sample to populate
    Sample & s = samples[i % NUMBER_OF_SAMPLES];

    // Begin data collection
    sampleTimers(s);  // sample system time/encoder counts/PWM duty cycle
    imu.Acquire(s);   // sample rate gyro, accelerometer and temperature sensors
    // sampleStates(s);  // sample various system states
    state_ = sampleSystemState() | Sample::CollectionEnabled;

    s.RearWheelRate_sp = rw.RateCommanded();
    s.YawRate_sp = yc.RateCommanded();
    // End data collection

    // Begin control
    if (rw.isEnabled() & (i % con::RW_N == 0))
      rw.Update(s.SystemTime, s.RearWheelAngle);

    if (yc.isEnabled() & (i % con::YC_N == 0))
      yc.Update(s);
    // End control
    
    // Begin data logging
    if (data && (i % (NUMBER_OF_SAMPLES/2) == 0)) {
      msg_t buffer;
      if ((i / (NUMBER_OF_SAMPLES/2)) & 1) {  // i is an odd multiple NUMBER_OF_SAMPLES/2  (64, 192, 320, ...)
        buffer = reinterpret_cast<msg_t>(samples);   // log data in samples[0:63]
      } else {  // i is an even multiple of NUMBER_OF_SAMPLES/2 (0, 128, 256, ...)
        buffer = reinterpret_cast<msg_t>(samples + (NUMBER_OF_SAMPLES/2)); // log data in samples[63:127]
      }
      state_ |= Sample::FileSystemWriteTriggered;
      msg = chMsgSend(tp_write, buffer);
      if (static_cast<FRESULT>(msg) != FR_OK) {
        ++write_errors;
      }
    }
    // End data logging 

    data = true;

    // Save system state;
    s.SystemState = state_;
    // Measure computation time
    s.ComputationTime = STM32_TIM5->CNT - s.SystemTime;
    // Go to sleep until next 5ms interval
    chThdSleepUntil(time);
  } // for i @ 200Hz
  
  // Clean up
  imu.DeInitialize();
  rw.turnOff();
  yc.turnOff();
  rw.Reset();
  yc.Reset();
  // End cleanup

  msg = chMsgSend(tp_write, 0); // send a message to end the write thread.
  if (static_cast<FRESULT>(msg) != FR_OK) {
    ++write_errors;
  }
  chThdYield();           // yield this time slot so write thread can finish

  f_close(&f_);           // close the file
  for (int i = 0; i < 24; ++i) // clear the filename
    filename_[i] = 0;       // clear the filename

  chThdExit(write_errors);
}

// Caller: Shell thread
void SampleAndControl::shellcmd(BaseSequentialStream *chp, int argc, char *argv[] __attribute__((unused)))
{
  if (argc == 0) {         // Start/Stop data collection, default filename
    if (tp_control) {      // Data collection enabled
      msg_t m = Stop();    // Stop it
      chprintf(chp, "Data collection and control terminated with %d errors.\r\n", m);
    } else {               // Data collecton disabled
      msg_t m = Start("samples.dat");// Start data collection to default file "samples.dat"
      if (m == 0) {
        chprintf(chp, "Data collection and control initiated.\r\n");
      } else {
        chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
      }
    }
  } else if (argc == 1) { // Start/Stop data collection, with filename
    if (tp_control) {     // Data collection enabled
      msg_t m = Stop();   // Stop it, ignoring the argument
      chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
    } else {              // Data collection is disabled
      msg_t m = Start(argv[0]);// Start data collection to file in argv[0]
      if (m == 0) {
        chprintf(chp, "Data collection and control initiated.\r\n");
      } else {
        chprintf(chp, "Errors starting threads with error:  %d.\r\n", m);
      }
    }
  } else {
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

void SampleAndControl::writeThread()
{
  UINT bytes;
  msg_t m = -1;
  FRESULT res = FR_OK;
  uint8_t *b;

  while (1) {
    Thread * calling_thread = chMsgWait();
    m = chMsgGet(calling_thread);
    chMsgRelease(calling_thread, res);

    if (m == 0) break;

    b = reinterpret_cast<uint8_t *>(m);

    res = f_write(&f_, b, sizeof(Sample)*NUMBER_OF_SAMPLES/2, &bytes);
  }
  chThdExit(0);
}

msg_t SampleAndControl::Start(const char * filename)
{
  msg_t m = 0;
  std::strcpy(filename_, filename);   // save the filename
  tp_write = chThdCreateStatic(SampleAndControl::waWriteThread,
                          sizeof(waWriteThread),
                          NORMALPRIO,
                          reinterpret_cast<tfunc_t>(writeThread_),
                          0);
  if (!tp_write) {
    m = (1 << 0);
  }
  tp_control = chThdCreateStatic(SampleAndControl::waControlThread,
                          sizeof(waControlThread),
                          NORMALPRIO,
                          reinterpret_cast<tfunc_t>(controlThread_),
                          0);
  if (!tp_control) {
    m |= (1 << 1);
  }

  return m;
}

