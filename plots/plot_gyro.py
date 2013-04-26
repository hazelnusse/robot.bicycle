import numpy as np
import matplotlib.pyplot as plt
from message_np import Message_np

samples_np = Message_np('Sample_pb2')
samples_np.load_messages_from_file('samples.dat', 'Sample')
data = samples_np.set_messages_np()

fig, ax = plt.subplots(1)
ax.plot(data['SystemTime'], data['mpu6050']['GyroscopeX'],
        data['SystemTime'], data['mpu6050']['GyroscopeY'],
        data['SystemTime'], data['mpu6050']['GyroscopeZ'])
ax.legend(('x', 'y', 'z'), loc=0)
ax.set_xlabel('time [s]')
ax.set_ylabel('angular velocity [rad/s^2]')
ax.set_title('MPU-6050 rate gyroscope measurements')
plt.show()
