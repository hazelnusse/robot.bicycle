#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>

class Sample {
 public:
  // System Time
  uint32_t SystemTime;
  // IMU raw sensor data
  int16_t MPU6050[7];
  // Angular position encoder steer data
  uint32_t RearWheelAngle, SteerAngle, FrontWheelAngle;
  // speed estimates from timer capture compare registers
  uint32_t RearWheelRate, SteerRate, FrontWheelRate;
  // Set points for rear wheel and steer rate
  float RearWheelRate_sp, YawRate_sp;
  // pwm outputs for rear wheel and steer
  uint16_t CCR_rw, CCR_steer;
  // System state bitfield
  uint32_t SystemState;

  enum StateFlags {SpeedControl =    1,
                   YawRateControl =  2,
                   HubMotorFault =   4,
                   SteerMotorFault = 8,
                   RearWheelEncoderB = 16,
                   SteerEncoderB = 32,
                   FrontWheelEncoderB = 64};
};

#endif
