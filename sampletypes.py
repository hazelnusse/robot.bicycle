import numpy as np

sample_t = np.dtype([('T', np.float64),
                     ('accx', np.float64),
                     ('accy', np.float64),
                     ('accz', np.float64),
                     ('Temperature', np.float64),
                     ('gyrox', np.float64),
                     ('gyroy', np.float64),
                     ('gyroz', np.float64),
                     ('RearWheelAngle', np.float64),
                     ('SteerAngle', np.float64),
                     ('FrontWheelAngle', np.float64),
                     ('RearWheelRate_sp', np.float64),
                     ('YawRate_sp', np.float64),
                     ('I_rw', np.float64),
                     ('I_steer', np.float64),
                     ('SystemState', np.uint32)])


SpeedControl = np.uint32(0x0001)
YawRateControl = np.uint32(0x0002)
HubMotorFault = np.uint32(0x0004)
SteerMotorFault = np.uint32(0x0008)
RearWheelEncoderDir = np.uint32(0x0010)
SteerEncoderDir = np.uint32(0x0020)
FrontWheelEncoderDir = np.uint32(0x0040)
RearWheelMotorCurrentDir = np.uint32(0x0080)
SteerMotorCurrentDir = np.uint32(0x0100)
