import numpy as np
import sampletypes as st
import matplotlib.pyplot as plt
from lowpassfilter import lpf

def get_color():
    for item in ['r', 'g', 'b']:
        yield item

class Samples(object):
    def __init__(self, datafile):
        self.data = np.fromfile(datafile, dtype=st.sample_t)
    
    def plotEncoders(self, v_est=None):
        f, ax = plt.subplots(3, sharex=True, sharey=False)
        data = self.data

        #  Rear wheel subplot
        rw_data = np.c_[data['RearWheelAngle'], data['RearWheelRate']]

        if v_est=='LPP':
            F = lpf([[ 0.703795508985816, 0.084013518175189],
                     [-0.084013518175188, 0.995679608367640]],
                    [-1.291217054014590, 0.031818210898906],
                    [-1.291217054014589, -0.031818210898905],
                    [0])
            F.input(data['RearWheelAngle'])

            #v = np.zeros(len(data['T']))
            #dt = data['T'][1] - data['T'][0]    # Assumes evenly spaced time
            #for i in range(len(v) - 1):
            #    v[i + 1] = (data['RearWheelAngle'][i + 1] - data['RearWheelAngle'][i])/dt

            v = F.output()

            rw_data = np.c_[rw_data, v]
        lines = ax[0].plot(data['T'], rw_data)
        lines[0].set_label('$\\theta_R$')
        lines[1].set_label('$\dot{\\theta}_R$')
        
        if v_est=='LPP':
            lines[2].set_label('$\dot{\\theta}_{R LPP}$')

        ax[0].legend()
        ax[0].set_title('Rear wheel, steer, front wheel optical encoder signals (top to bottom)')
        ax[0].set_ylabel('[rad], [rad / s]')



        #  Steer subplot
        steer_data = np.c_[data['SteerAngle'], data['SteerRate']]

        if v_est=='LPP':
            v = np.zeros(len(data['T']))
            dt = data['T'][1] - data['T'][0]    # Assumes evenly spaced time
            for i in range(len(v) - 1):
                v[i + 1] = (data['SteerAngle'][i + 1] - data['SteerAngle'][i])/dt
            ax[1].plot(data['T'], v)

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

