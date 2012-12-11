import numpy as np
import sampletypes as st
import matplotlib.pyplot as plt

def get_color():
    for item in ['r', 'g', 'b']:
        yield item

class Samples(object):
    def __init__(self, datafile):
        self.data = np.fromfile(datafile, dtype=st.sample_t)
    
    def plotRearWheel(self, N=None):
        f, ax = plt.subplots(1, sharex=False, sharey=False)
        data = self.data
        ax = [ax]

        #  Rear wheel subplot
        lines = ax[0].step(data['T'], data['RearWheelAngle'],
                                      label='$\\theta_R$')
        if N is not None:
            theta_N = data['RearWheelAngle'][::N]
            t_N = data['T'][::N]
            v = np.zeros(len(t_N))
            for i in range(len(t_N) - 1):
                dt = t_N[i + 1] - t_N[i]
                theta_new = theta_N[i + 1]
                theta_old = theta_N[i]
                v[i] = (theta_new - theta_old) / dt
                if i:
                    if theta_new - theta_old > 1.5*np.pi:  # underflow occurred
                        print("uflow@{0}, {1}, {2}".format(t_N[i+1],
                                                           theta_new,
                                                           theta_old))
                        v[i] = (theta_new - 2.0*np.pi - theta_old) / dt
                    elif theta_new - theta_old < -1.5*np.pi: # overflow occured
                        print("oflow@{0}, {1}, {2}".format(t_N[i+1],
                                                           theta_new,
                                                           theta_old))
                        v[i] = (theta_new + 2.0*np.pi - theta_old) / dt

            ax[0].step(t_N, v, label='$\\Delta\\theta_R/\\Delta t$')
        
        ax[0].step(data['T'], data['I_rw'], label='I')
        ax[0].step(data['T'], data['RearWheelRate_sp'], label='$\\theta_r$')
        ax[0].legend(loc=0)
        ax[0].set_title('Rear wheel')
        ax[0].set_ylabel('[rad], [rad / s], [A]')
        ax[0].set_xlabel('time [s]')


        #  Steer subplot
        #steer_data = np.c_[data['SteerAngle'], data['SteerRate']]


        #ax[1].legend(('$\\delta$', '$\dot{\\delta}$'), loc=0)
        #ax[1].set_ylabel('[rad], [rad / s]')

        #ax[2].plot(data['T'], data['FrontWheelAngle'],
        #           data['T'], data['FrontWheelRate'])
        #ax[2].legend(('$\\theta_F$', '$\dot{\\theta}_F$'), loc=0)
        #ax[2].set_xlabel('time [s]')
        #ax[2].set_ylabel('[rad], [rad / s]')

        #f.subplots_adjust(hspace=0)
        #plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
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

    def plotTime(self):
        f, ax = plt.subplots(1)
        data = self.data
        N = len(data['T'])
        dt = np.zeros(N - 1)
        for i in range(N - 1):
            dt[i] = data['T'][i + 1] - data['T'][i]
        n, bins, patches = ax.hist(dt, bins=20)
        ax.set_ylabel('Occurances')
        ax.set_xlabel('Sample period')
        ax.set_title('mean = {0}, std dev. = {1}'.format(dt.mean(),
            dt.std()))

    def plotState(self):
        f, ax = plt.subplots(1)
        data = self.data
        sc = np.c_[np.bitwise_and(data['SystemState'], st.SpeedControl),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.YawRateControl), st.YawRateControl),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.HubMotorFault), st.HubMotorFault),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.SteerMotorFault), st.SteerMotorFault),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.RearWheelEncoderDir), st.RearWheelEncoderDir),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.SteerEncoderDir), st.SteerEncoderDir),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.FrontWheelEncoderDir), st.FrontWheelEncoderDir),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.RearWheelMotorCurrentDir), st.RearWheelMotorCurrentDir),
               np.right_shift(np.bitwise_and(data['SystemState'],
                   st.SteerMotorCurrentDir), st.SteerMotorCurrentDir)]
        lines = ax.matshow(sc.T[:, 4400:4500], aspect=1)

        #ax[0].legend(loc=0)
        #ax.set_title('System state flags')
        #ax.set_xlabel('time [s]')
