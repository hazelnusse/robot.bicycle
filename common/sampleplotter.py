import inspect
import sys
import numpy as np
import sampletypes as st
import matplotlib.pyplot as plt
from scipy import fft, arange

def get_color():
    for item in ['r', 'g', 'b']:
        yield item

def class_objects(prefix):
    d = {}
    for name, obj in inspect.getmembers(sys.modules[__name__]):
        if inspect.isclass(obj) and name.startswith(prefix):
            d[name] = obj
    return d

class Samples(object):
    def __init__(self, datafile, args):
        self.load_file(datafile)
        self.plots = []
        self.set_plots(args)

    def load_file(self, datafile):
        self.data = np.fromfile(datafile, dtype=st.sample_t)

    def set_plots(self, args):
        self.plots = []
        plot_classes = class_objects("Plot")
        if not args:
            args = [name for name in plot_classes.keys() if
                    name != 'PlotBase']
        for p in args:
            if p in plot_classes:
                plot = plot_classes[p](self)
                setattr(self, p[len('Plot'):].lower(), plot)
                self.plots.append(plot)

    def draw_plots(self):
        for p in self.plots:
            try:
                p.axes.clear()
                p.axes.autoscale(tight=True)
            except AttributeError as e: # if multiple axes
                for i in range(len(p.axes)):
                    p.axes[i].clear()
            p.plot_data()


class PlotBase(object):
    def __init__(self, parent):
        self.parent = parent
        self.figure, self.axes = self.create_figure_axes()

    def create_figure_axes(self):
        figure, axes = plt.subplots(1)
        axes.autoscale(tight=True)
        return figure, axes

    def plot_data(self):
        raise NotImplementedError("Subclass must implement abstract method")


class PlotWheel(PlotBase):
    def __init__(self, parent):
        super(PlotWheel, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.step(data['T'], data['RearWheelAngle'],
                               label='$\\theta_R$')
        ax.step(data['T'], data['FrontWheelAngle'],
                               label='$\\theta_F$')
        ax.step(data['T'], data['I_rw'], label='I')
        ax.step(data['T'], data['RearWheelRate_sp'],
                       label='$\\dot{\\theta}_{Rc}$')
        ax.step(data['T'], data['theta_R_dot'], label='$\\dot{\\theta}_R$')
        ax.legend(loc=0)
        ax.set_title('Rear wheel')
        ax.set_ylabel('[rad], [rad / s], [A]')
        ax.set_xlabel('time [s]')


class PlotAccelerometer(PlotBase):
    def __init__(self, parent):
        super(PlotAccelerometer, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.plot(data['T'], data['accx'],
                data['T'], data['accy'],
                data['T'], data['accz'])
        ax.legend(('x', 'y', 'z'), loc=0)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('acceleration [m/s^2]')
        ax.set_title('MPU-6050 accelerometer measurements')


class PlotGyroscope(PlotBase):
    def __init__(self, parent):
        super(PlotGyroscope, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.plot(data['T'], data['gyrox'],
                data['T'], data['gyroy'],
                data['T'], data['gyroz'])
        ax.legend(('x', 'y', 'z'), loc=0)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('angular velocity [rad/s^2]')
        ax.set_title('MPU-6050 rate gyroscope measurements')


class PlotTemperature(PlotBase):
    def __init__(self, parent):
        super(PlotTemperature, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.plot(data['T'], data['Temperature'])
        ax.set_ylabel('temperature [C]')
        ax.set_xlabel('time [s]')
        ax.set_title('MPU-6050 temperature measurement')


class PlotTime(PlotBase):
    def __init__(self, parent):
        super(PlotTime, self).__init__(parent)

    def create_figure_axes(self):
        figure, axes = plt.subplots(2, sharex=False, sharey=False)
        return figure, axes

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        N = len(data['T'])
        dt = np.zeros(N - 1)
        for i in range(N - 1):
            dt[i] = data['T'][i + 1] - data['T'][i]
        n, bins, patches = ax[0].hist(dt, bins=20)
        ax[0].set_ylabel('Occurances')
        ax[0].set_xlabel('Sample period')
        ax[0].set_title('mean = {0}, std dev. = {1}'.format(dt.mean(),
                                                            dt.std()))

        n, bins, patches = ax[1].hist(data['T_c'], bins=20)
        ax[1].set_ylabel('Occurances')
        ax[1].set_xlabel('Computation time')
        ax[1].set_title('mean = {0}, std dev. = {1}'.format(data['T_c'].mean(),
                                                            data['T_c'].std()))


class PlotState(PlotBase):
    def __init__self(parent):
        super(PlotState, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        bitset = lambda x, y: 1 if x & y else 0
        bitset_v = np.vectorize(bitset)
        bitset_state = lambda x: bitset_v(data['SystemState'], x)

        sc = np.c_[bitset_state(st.SpeedControl),
                   bitset_state(st.YawRateControl),
                   bitset_state(st.HubMotorFault),
                   bitset_state(st.SteerMotorFault),
                   bitset_state(st.RearWheelEncoderDir),
                   bitset_state(st.SteerEncoderDir),
                   bitset_state(st.FrontWheelEncoderDir),
                   bitset_state(st.RearWheelMotorCurrentDir),
                   bitset_state(st.SteerMotorCurrentDir)]
        m, n = sc.shape
        lines = ax.matshow(sc.T, aspect=m/n, cmap='Greys')

        #ax[0].legend(loc=0)
        ax.set_title('System state flags')
        ax.set_xlabel('time [s]')

class PlotSteerAngle(PlotBase):
    def __init__(self, parent):
        super(PlotSteerAngle, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.step(data['T'], data['SteerAngle'], label='$\\delta$')
        ax.legend(loc=0)
        ax.set_ylabel('[rad]')
        ax.set_xlabel('time [s]')
        ax.set_title('Steer angle')


class PlotRollRate(PlotBase):
    def __init__(self, parent):
        super(PlotRollRate, self).__init__(parent)
    
    def create_figure_axes(self):
        figure, axes = plt.subplots(2, 1, sharex=False, sharey=False)
        return figure, axes

    def plot_data(self):
        data = self.parent.data
        ax = self.axes

        # Time series
        ax[0].step(data['T'], data['phi_dot'], label='$\\dot{\\phi}$')
        ax[0].legend(loc=0)
        ax[0].set_ylabel('[rad / s]')
        ax[0].set_xlabel('time [s]')
        ax[0].set_title('Estimated roll rate')
        
        # FFT 
        N = len(data) # length of the signal

        dt = np.zeros(N - 1)
        for i in range(N - 1):
            dt[i] = data['T'][i + 1] - data['T'][i]
        Ts = dt.mean() * N
        k = arange(N)
        frq = k/Ts # two sides frequency range
        frq = frq[:N//2] # one side frequency range

        Y = fft(data['phi_dot'])/N # fft computing and normalization
        Y = Y[:N//2]
         
        ax[1].plot(frq, abs(Y), 'r') # plotting the spectrum
        ax[1].set_xlabel('$\\omega$ [Hz]')
        ax[1].set_ylabel('$|\\phi(\\omega)|$')


class PlotControllerStates(PlotBase):
    def __init__(self, parent):
        super(PlotControllerStates, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.step(data['T'], data['x'][:, 0], label='$\\phi$')
        ax.step(data['T'], data['x'][:, 1], label='$\\delta$')
        ax.step(data['T'], data['x'][:, 2], label='$\\dot{\\phi}$')
        ax.step(data['T'], data['x'][:, 3], label='$\\dot{\\delta}$')
        ax.step(data['T'], data['x'][:, 4], label='$x_I$')
        ax.legend(loc=0)
        ax.set_ylabel('[rad, rad/s]')
        ax.set_xlabel('time [s]')
        ax.set_title('State estimates')


class PlotSteerTorqueAndCurrent(PlotBase):
    def __init__(self, parent):
        super(PlotSteerTorqueAndCurrent, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.step(data['T'], data['I_steer'], label='$I_\\delta$ (actual)')
        ax.step(data['T'], data['steer_current'], label='$I_\\delta$')
        ax.legend(loc=0)
        ax.set_ylabel('[N m, A]')
        ax.set_xlabel('time [s]')
        ax.set_ylim((-12, 12))
        ax.set_title('Steer torque and current')

