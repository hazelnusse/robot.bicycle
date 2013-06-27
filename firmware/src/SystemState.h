#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H

namespace systemstate {

enum StateFlags {RearWheelMotorEnable     = 0x00000001,
                 SteerMotorEnable         = 0x00000002,
                 RearWheelMotorFault      = 0x00000004,
                 SteerMotorFault          = 0x00000008,
                 RearWheelEncoderDir      = 0x00000010,
                 SteerEncoderDir          = 0x00000020,
                 FrontWheelEncoderDir     = 0x00000040,
                 RearWheelMotorCurrentDir = 0x00000080,
                 SteerMotorCurrentDir     = 0x00000100,
                 FileSystemWriteTriggered = 0x00000200,
                 SampleBufferOverflow     = 0x00000400,
                 SampleBufferEncodeError  = 0x00000800,
                 SampleBufferInitError    = 0x00001000,
                 HWButton                 = 0x00004000,
                 CollectionEnabled        = 0x00008000,// used primarily for GUI
                 I2C_Bus_Error            = 0x00010000,
                 I2C_Arbitration_Lost     = 0x00020000,
                 I2C_ACK_Failure          = 0x00040000,
                 I2C_Overrun              = 0x00080000,
                 I2C_PEC_Error            = 0x00100000,
                 I2C_Hardware_Timeout     = 0x00200000,
                 I2C_SMB_Alert            = 0x00400000,
                 I2C_Software_Timeout     = 0x00800000,
                 FATFS_f_open_error       = 0x01000000};

}

#endif

