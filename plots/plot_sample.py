#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from itertools import cycle
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate
from plot_data import PlotData, PLOTY_SEP

SECONDS_PER_CLOCK = 0.25e-6
WHEEL_RADIUS = 0.3359
STATE_FLAGS = [
    ('rear_wheel_motor_enable', 0x00000001),
    ('steer_motor_enable', 0x00000002),
    ('rear_wheel_motor_fault', 0x00000004),
    ('steer_motor_fault', 0x00000008),
    ('rear_wheel_encoder_dir', 0x00000010),
    ('steer_encoder_dir', 0x00000020),
    ('front_wheel_encoder_dir', 0x00000040),
    ('rear_wheel_motor_current_dir', 0x00000080),
    ('steer_motor_current_dir', 0x00000100),
    ('file_system_write_triggered', 0x00000200),
    ('sample_buffer_overflow', 0x00000400),
    ('sample_buffer_encode_error', 0x00000800),
    ('sample_buffer_init_error', 0x00001000),
    ('hw_button', 0x00004000),
    ('collection_enabled', 0x00008000),
    ('I2C_Bus_Error', 0x00010000),
    ('I2C_Arbitration_Lost', 0x00020000),
    ('I2C_ACK_Failure', 0x00040000),
    ('I2C_Overrun', 0x00080000),
    ('I2C_PEC_Error', 0x00100000),
    ('I2C_Hardware_Timeout', 0x00200000),
    ('I2C_SMB_Alert', 0x00400000),
    ('I2C_Software_Timeout', 0x00800000),
    ('FATFS_f_open_error', 0x01000000)
]

class PlotSample(PlotData):
    def __init__(self, datafile=None):
        super(PlotSample, self).__init__('sample_pb2', 'Sample', datafile)
        self.filename = datafile
        self.default_x = None
        self._correct_system_time()
        self._add_sample_period()
        self._add_forward_speed()
        self._add_threshold_speed()
        self._parse_system_state()
        self._mask_fields(['computation_time', 'sample_period'])
        self._convert_clocks_to_seconds(['system_time_c', 'computation_time',
                                         'sample_period'])
        self._estimate_lean()
        self.set_default_x('system_time_s')

    def _correct_system_time(self):
        field = 'system_time_c'
        systime_data = self.get_field_data('system_time')
        self.dtype_c[field] = np.uint64
        self.data_c[field] = np.ma.empty(systime_data.shape,
                                      dtype=self.dtype_c[field])
        prev_t = 0
        offset = 0
        for i, t in enumerate(systime_data):
            if prev_t > t:
                offset += 2**32
            self.data_c[field][i] = (systime_data[i] + offset)
            prev_t = t

    def _add_sample_period(self):
        field = 'sample_period'
        systime_data = self.get_field_data('system_time_c')
        systime_delay = np.roll(systime_data, 1)
        systime_dt = systime_data - systime_delay
        self.data_c[field] = systime_dt
        self.dtype_c[field] = type(self.data_c[field][0])

    def _add_forward_speed(self):
        field = 'forward_speed'
        d = self.get_field_data('encoder.rear_wheel_rate')
        self.dtype_c[field] = np.float32
        self.data_c[field] = -WHEEL_RADIUS * d.astype(np.float32)

    def _add_threshold_speed(self):
        for f in self.expand_field('threshold'):
            d = self.get_field_data(f)
            field = f.replace('.', '_speed_')
            self.dtype_c[field] = np.float32
            self.data_c[field] = -WHEEL_RADIUS * d.astype(np.float32)

    def _parse_system_state(self):
        data = self.get_field_data('system_state')
        state_bit = np.empty(data.shape, dtype=np.uint32)
        for field, bit in STATE_FLAGS:
            field = 'system_state_' + field
            state_bit.fill(bit)
            self.dtype_c[field] = np.bool
            self.data_c[field] = np.bitwise_and(data,
                                                state_bit).astype(np.bool)
    def _estimate_lean(self):
        """Add estimates of lean angle using measurements of accelerometer
        and gyroscope. For the accelerometer calculation, the system is
        assumed static.
        """
        data_ax = self.get_field_data('mpu6050.accelerometer_x')
        data_ay = self.get_field_data('mpu6050.accelerometer_y')
        data_az = self.get_field_data('mpu6050.accelerometer_z')
        data_axayaz = np.vstack([data_ax, data_ay, data_az])
        magnitude = np.apply_along_axis(np.linalg.norm, 0, data_axayaz)
        self.dtype_c['lean_accel'] = np.float32
        #self.data_c['lean_accel']= np.arcsin(data_ax / magnitude) * 180 / np.pi
        self.data_c['lean_accel']= np.arcsin(data_ax / magnitude)

        data_gyro = self.get_field_data('mpu6050.gyroscope_x')
        data_time = self.get_field_data('system_time_s')
        self.dtype_c['lean_gyro'] = np.float32
        init_lean = self.get_field_data('lean_accel')[0]
        self.data_c['lean_gyro'] = (scipy.integrate.cumtrapz(data_gyro,
                                                             data_time,
                                                             initial=0) +
                                    init_lean)

    def _mask_fields(self, field_list):
        """Mask the first element in the given fields since they are invalid.
        """
        for field in field_list:
            self.get_field_data(field)[0] = np.ma.masked

    def _convert_clocks_to_seconds(self, fields):
        for field in fields:
            data = self.get_field_data(field)
            if field == 'system_time_c':
                field = 'system_time'
            field_s = field + '_s'
            self.data_c[field_s] = data * SECONDS_PER_CLOCK
            self.dtype_c[field_s] = type(self.data_c[field_s][0])

    def set_default_x(self, field):
        self.default_x = field;

    def plot_d(self, *args, **kwargs):
        """Calls plot(), using self.default_x as the x axis.
        Refer to plot() for use of options.
        """
        return self.plot(self.default_x, *args, **kwargs)

    def _plotyy(self, x, y1, y2, scale1=None, scale2=None, axes=None):
        if axes is None:
            fig, ax = plt.subplots(1)
        else:
            ax = axes
            fig = ax.get_figure()
        if scale1 is None:
            scale1 = 1
        if scale2 is None:
            scale2 = 1

        if isinstance(y1, list):
            y1 = [item for sublist in map(self.expand_field, y1)
                  for item in sublist]
        else:
            y1 = self.expand_field(y1)
        if isinstance(y2, list):
            y2 = [item for sublist in map(self.expand_field, y2)
                  for item in sublist]
        else:
            y2 = self.expand_field(y2)
        self._set_color_cycle(ax, len(y1 + y2) + 2)

        xdata = self.get_field_data(self.expand_field(x)[0])
        add_plot = lambda ax, y, s: ax.plot(xdata, s * self.get_field_data(y),
                                            label=y)
        for y in y1:
            add_plot(ax, y, scale1)
        axt = ax.twinx()
        for y in y2:
            add_plot(axt, y, scale2)

        ax.set_xlabel(x)
        ax.set_ylabel(', '.join(y1))
        axt.set_ylabel(', '.join(y2))
        handles, labels = ax.get_legend_handles_labels()
        handles2, labels2 = axt.get_legend_handles_labels()
        ax.legend(handles + handles2, labels + labels2, loc=2)
        ax.grid(True)
        return fig, ax

    def plot_estimates(self):
        fig, ax = plt.subplots(nrows=2, sharex=True)
        self._plotyy('system_time_s',
                     ['encoder.steer', 'estimate.steer',
                      'estimate.lean', 'gyro_lean.angle', 'gyro_lean.startup'],
                     ['forward_speed', 'motor_torque.steer',
                      'system_state_hw_button'],
                     axes=ax[0])
        self._plotyy('system_time_s',
                     ['mpu6050.gyroscope_x', 'estimate.lean_rate',
                      'estimate.steer_rate', 'encoder.steer_rate'],
                     ['forward_speed', 'motor_torque.steer',
                      'system_state_hw_button'],
                     axes=ax[1])
        ax[0].set_title("estimates, file '{0}'".format(self.filename))
        plt.show()
        return fig, ax

    def plot_forward_speed(self):
        return self.plot_d(['forward_speed', 'threshold_speed_estimation',
                            'threshold_speed_control'])

    def plot_rear_wheel(self):
        fig, ax = self._plotyy('system_time_s',
                               ['set_point.theta_R_dot',
                                'encoder.rear_wheel_rate',
                                'threshold.estimation'],
                               ['motor_torque.rear_wheel',
                                'motor_torque.desired_rear_wheel'],
                               scale1=-WHEEL_RADIUS, scale2=-1)
        ax.set_title("rear wheel, file '{0}'".format(self.filename))
        return fig, ax

    def plot_system_state(self, enable_all=False, disable_func=None):
        """Plot the system state boolean fields. Fields are scaled from 0.75
        to 1.25 such that different fields will not overlap when HIGH. User
        can specify which state fields are displayed by passing a function
        to 'disable_func' to exclude certain state fields. By default,
        'disable_func' is set to:
        lambda f: f.startswith('I2C') or f.endswith('_dir')

        Alternatively, all state fields can be displayed by setting
        'enable_all' to True.
        """
        if disable_func is None:
            disable_func = lambda f: f.startswith('I2C') or f.endswith('_dir')
        if enable_all:
            disable_func = lambda f: False

        fig, ax = plt.subplots(1)
        xdata = self.get_field_data(self.expand_field(self.default_x)[0])
        state_fields = [f for f in self.data_c.iterkeys()
                        if (f.startswith('system_state_') and
                            not disable_func(f.replace('system_state_', '')))]

        scale_fields = np.linspace(0.75, 1.25, len(state_fields))
        self._set_color_cycle(ax, len(state_fields) + 1)
        lscycler = cycle(['-', '--'])

        for fields, scale in zip(sorted(state_fields), scale_fields):
            ax.plot(xdata, self.get_field_data(fields) * scale,
                    label=fields.replace('system_state_', ''),
                    linestyle=next(lscycler))

        ax.legend(loc=2)
        ax.grid(True)
        ax.set_xlabel(self.default_x)
        ax.set_ylabel('system state')
        ax.set_title("system state, file '{0}'".format(self.filename))
        plt.show()
        return fig, ax

    def fft_window(self, field, start=None, stop=None, step=None):
        """Plot a series of Fourier transforms for 'field' using different
        time windows. First window begins at time 'start' and last window ends
        at time 'stop'.'step' specifies the length for each window in the
        series.
        By default, the series spans the all data collected and uses the
        last time value for 'step'.
        """
        fig, ax = plt.subplots(1)
        if not isinstance(field, list):
            field = [field]
        ex_fields = [ef for f in field for ef in self.expand_field(f)]
        T = np.mean(self.get_field_data('sample_period_s'))
        t = self.get_field_data('system_time_s')

        if start is None:
            start = t[0]
        if stop is None:
            stop = t[-1]
        if step is None:
            step = stop

        times = np.arange(start, stop, step)
        self._set_color_cycle(ax, len(times) * len(ex_fields) + 1)

        for ef in ex_fields:
            time_start = start
            for time_stop in np.append(times, stop)[1:]:
                data = self.get_field_data(ef)
                data_slice = data[np.logical_and(t >= time_start, t < time_stop)]
                Y = abs(np.fft.fft(data_slice))
                f = np.fft.fftfreq(len(data_slice), T)
                scale = np.max(Y)
                if scale < 1e-12:
                    scale = 1.0
                ax.plot(f[range(len(data_slice)/2)],
                        Y[range(len(data_slice)/2)] / scale,
                        label="field:{0}\ntime: [{1}, {2})".format(ef,
                                                                   time_start,
                                                                   time_stop))
                time_start = time_stop

        field_str = PLOTY_SEP.join(field)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("|Y(freq)|")
        ax.set_title(("Fourier series in windows of {0} s from {1} s to " +
                      "{2} s for {3}").format(step, start, stop, field_str))
        ax.legend()
        plt.show()
        return fig
