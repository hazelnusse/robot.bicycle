#ifndef SAMPLE_H
#define SAMPLE_H

#include <cstdint>
#include <cstring>

class Sample {
 public:
  void clear() { memset(this, 0, sizeof(*this)); }

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

  enum StateFlags {RearWheelMotorEnable     = 0x00000001,
                   SteerMotorEnable         = 0x00000002,
                   HubMotorFault            = 0x00000004,
                   SteerMotorFault          = 0x00000008,
                   RearWheelEncoderDir      = 0x00000010,
                   SteerEncoderDir          = 0x00000020,
                   FrontWheelEncoderDir     = 0x00000040,
                   RearWheelMotorCurrentDir = 0x00000080,
                   SteerMotorCurrentDir     = 0x00000100,
                   FileSystemWriteTriggered = 0x00000200,
                   CollectionEnabled        = 0x00008000,// used primarily for GUI
                   I2C_Bus_Error            = 0x00010000,
                   I2C_Arbitration_Lost     = 0x00020000,
                   I2C_ACK_Failure          = 0x00040000,
                   I2C_Overrun              = 0x00080000,
                   I2C_PEC_Error            = 0x00100000,
                   I2C_Hardware_Timeout     = 0x00200000,
                   I2C_SMB_Alert            = 0x00400000,
                   I2C_Software_Timeout     = 0x00800000};
};

#endif

