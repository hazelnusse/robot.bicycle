#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>

class Sample {
 public:
  // System Time
  uint32_t SystemTime;
  // Control loop computation time
  uint32_t ComputationTime;
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

  enum StateFlags {RearWheelMotorEnable         = 0x0001,
                   SteerMotorEnable             = 0x0002,
                   HubMotorFault                = 0x0004,
                   SteerMotorFault              = 0x0008,
                   RearWheelEncoderDir          = 0x0010,
                   SteerEncoderDir              = 0x0020,
                   FrontWheelEncoderDir         = 0x0040,
                   RearWheelMotorCurrentDir     = 0x0080,
                   SteerMotorCurrentDir         = 0x0100,
                   FileSystemWriteTriggered     = 0x0200,
                   CollectionEnabled            = 0x8000};// used primarily for GUI
};

void clearSample(Sample & s);
#endif
