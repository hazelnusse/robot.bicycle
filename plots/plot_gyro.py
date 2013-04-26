import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "proto"))

import numpy as np
import matplotlib.pyplot as plt
from message_np import Message_np

samples_np = Message_np('sample_pb2', 'Sample', 'samples.dat')
data = samples_np.get_messages_np()

fig, ax = plt.subplots(1)
ax.plot(data['system_time'], data['mpu6050']['gyroscope_x'],
        data['system_time'], data['mpu6050']['gyroscope_y'],
        data['system_time'], data['mpu6050']['gyroscope_z'])
ax.legend(('x', 'y', 'z'), loc=0)
ax.set_xlabel('time [s]')
ax.set_ylabel('angular velocity [rad/s^2]')
ax.set_title('MPU-6050 rate gyroscope measurements')
plt.show()
