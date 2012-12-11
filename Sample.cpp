#include "Sample.h"

void clearSample(Sample & s) {
  s.SystemTime = 0;
  s.ComputationTime = 0;
  s.RearWheelAngle = 0;
  s.SteerAngle = 0;
  s.FrontWheelAngle = 0;
  s.SystemState = 0;
  s.RearWheelRate_sp = 0;
  s.YawRate_sp = 0;
  s.CCR_rw = 0;
  s.CCR_steer = 0;
  s.MPU6050[0] = s.MPU6050[1]
               = s.MPU6050[2]
               = s.MPU6050[3]
               = s.MPU6050[4]
               = s.MPU6050[5]
               = s.MPU6050[6]
               = 0;
}


