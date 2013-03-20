import inspect
import sys
import numpy as np
import sampletypes as st
import matplotlib.pyplot as plt

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
                    (name != 'PlotBase' and name != 'PlotState')]
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


class PlotRearWheel(PlotBase):
    def __init__(self, parent):
        super(PlotRearWheel, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        lines = ax.step(data['T'], data['RearWheelAngle'],
                               label='$\\theta_R$')
        ax.step(data['T'], data['I_rw'], label='I')
        ax.step(data['T'], data['RearWheelRate_sp'],
                       label='$\\theta_r$')
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
        sc = np.c_[np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.SpeedControl),
                        st.SpeedControl),
                   np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.YawRateControl),
                        st.YawRateControl),
                   np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.HubMotorFault),
                        st.HubMotorFault),
                   np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.SteerMotorFault),
                        st.SteerMotorFault),
                   np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.RearWheelEncoderDir),
                        st.RearWheelEncoderDir),
                   np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.SteerEncoderDir),
                        st.SteerEncoderDir),
                   np.right_shift(
                        np.bitwise_and(data['SystemState'],
                                       st.FrontWheelEncoderDir),
                        st.FrontWheelEncoderDir),
                   np.right_shift(
                       np.bitwise_and(data['SystemState'],
                                      st.RearWheelMotorCurrentDir),
                       st.RearWheelMotorCurrentDir),
                   np.right_shift(
                       np.bitwise_and(data['SystemState'],
                                      st.SteerMotorCurrentDir),
                       st.SteerMotorCurrentDir)]
        lines = ax.matshow(sc.T[:, 4400:4500], aspect=1)

        #ax[0].legend(loc=0)
        ax.set_title('System state flags')
        ax.set_xlabel('time [s]')

class PlotSteer(PlotBase):
    def __init__(self, parent):
        super(PlotSteer, self).__init__(parent)

    def plot_data(self):
        data = self.parent.data
        ax = self.axes
        ax.step(data['T'], data['SteerAngle'], label='$\\delta$')
        ax.legend(loc=0)
        ax.set_ylabel('[rad]')
        ax.set_xlabel('time [s]')
        ax.set_title('Steer angle')
