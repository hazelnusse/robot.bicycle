#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>

class Sample {
 public:
  // System Time
  uint32_t SystemTime;
  // Angular position encoder steer data
  uint32_t RearWheelAngle;
  int32_t SteerAngle;
  uint32_t FrontWheelAngle;
  // System state bitfield
  uint32_t SystemState;
  // Set points for rear wheel and steer rate
  float RearWheelRate_sp, YawRate_sp;
  // pwm outputs for rear wheel and steer
  uint16_t CCR_rw, CCR_steer;
  // IMU raw sensor data
  int16_t MPU6050[7];

  enum StateFlags {SpeedControl =    1,
                   YawRateControl =  2,
                   HubMotorFault =   4,
                   SteerMotorFault = 8,
                   RearWheelEncoderDir = 16,
                   SteerEncoderDir = 32,
                   FrontWheelEncoderDir = 64};
};

#endif
