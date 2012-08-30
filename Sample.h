#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>

class Sample {
 public:
  // System Time
  uint32_t SystemTime;
  // IMU raw sensor data
  int16_t Gyroscope[4], Accelerometer[3], Magnetometer[3];
  // Angular position encoder steer data
  uint32_t SteerAngle;
  // speed estimates from timer capture compare registers
  uint32_t RearWheelRate, FrontWheelRate, SteerRate;
  // Set points for rear wheel and steer rate
  float RearWheelRate_sp, YawRate_sp;
  // pwm outputs for rear wheel and steer
  uint16_t CCR_rw, CCR_steer;
  // System state bitfield
  uint32_t SystemState;

  enum StateFlags {SpeedControl = 0,
                   YawRateControl,
                   HubMotorFault,
                   SteerMotorFault};
};

#endif
