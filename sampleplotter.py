import numpy as np
import sampletypes as st
import matplotlib.pyplot as plt

class Samples(object):
    def __init__(self, datafile):
        self.data = np.fromfile(datafile, dtype=st.sample_t)
    
    def plotEncoders(self):
        f, ax = plt.subplots(3, sharex=True, sharey=False)
        data = self.data
        ax[0].plot(data['T'], data['RearWheelAngle'],
                   data['T'], data['RearWheelRate'])
        ax[0].set_title('Rear wheel, steer, front wheel optical encoder signals (top to bottom)')
        ax[0].legend(('$\\theta_R$', '$\dot{\\theta}_R$'), loc=0)
        ax[0].set_ylabel('[rad], [rad / s]')
        ax[1].plot(data['T'], data['SteerAngle'],
                   data['T'], data['SteerRate'])
        ax[1].legend(('$\\delta$', '$\dot{\\delta}$'), loc=0)
        ax[1].set_ylabel('[rad], [rad / s]')
        ax[2].plot(data['T'], data['FrontWheelAngle'],
                   data['T'], data['FrontWheelRate'])
        ax[2].legend(('$\\theta_F$', '$\dot{\\theta}_F$'), loc=0)
        ax[2].set_xlabel('time [s]')
        ax[2].set_ylabel('[rad], [rad / s]')

        f.subplots_adjust(hspace=0)
        plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
        plt.axis('tight')

    def plotAccelerometer(self):
        f, ax = plt.subplots(1)
        data = self.data
        ax.plot(data['T'], data['accx'],
                data['T'], data['accy'],
                data['T'], data['accz'])
        ax.set_xlabel('time [s]')
        ax.set_ylabel('acceleration [m/s^2]')
        ax.set_title('MPU-6050 accelerometer measurements')
        plt.axis('tight')

    def plotGyroscope(self):
        f, ax = plt.subplots(1)
        data = self.data
        ax.plot(data['T'], data['gyrox'],
                data['T'], data['gyroy'],
                data['T'], data['gyroz'])
        ax.set_xlabel('time [s]')
        ax.set_ylabel('angular velocity [rad/s^2]')
        ax.set_title('MPU-6050 rate gyroscope measurements')
        plt.axis('tight')

    def plotTemperature(self):
        f, ax = plt.subplots(1)
        data = self.data
        ax.plot(data['T'], data['Temperature'])
        ax.set_ylabel('temperature [C]')
        ax.set_xlabel('time [s]')
        ax.set_title('MPU-6050 temperature measurement')
        plt.axis('tight')

