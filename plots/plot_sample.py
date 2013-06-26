#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "proto"))

import matplotlib.pyplot as plt
import numpy as np
from plot_data import PlotData, PLOTY_SEP

SECONDS_PER_CLOCK = 0.25e-6
WHEEL_RADIUS = 0.3359

class PlotSample(PlotData):
    def __init__(self, datafile=None):
        super(PlotSample, self).__init__('sample_pb2', 'Sample', datafile)
        self.default_x = None
        self._correct_system_time()
        self._add_sample_period()
        self._add_forward_speed()
        self._add_threshold_speed()
        self._mask_fields(['computation_time', 'sample_period'])
        self._convert_clocks_to_seconds(['system_time_c', 'computation_time',
                                         'sample_period'])
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

    def _plotyy(self, x, y1, y2, axes=None):
        if axes is None:
            fig, ax = plt.subplots(1)
        else:
            ax = axes
            fig = ax.get_figure()

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
        self._set_color_cycle(ax, len(y1 + y2) + 1)

        xdata = self.get_field_data(self.expand_field(x)[0])
        add_plot = lambda ax, y: ax.plot(xdata, self.get_field_data(y),
                                         label=y)
        for y in y1:
            add_plot(ax, y)
        axt = ax.twinx()
        for y in y2:
            add_plot(axt, y)

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
                     ['encoder.steer', 'estimate.phi', 'estimate.delta'],
                     ['forward_speed', 'motor_current.steer'],
                     axes=ax[0])
        self._plotyy('system_time_s',
                     ['mpu6050.gyroscope_y', 'estimate.phi_dot', 'estimate.delta_dot'],
                     ['forward_speed', 'motor_current.steer'],
                     axes=ax[1])
        plt.show()
        return fig, ax
        #fig, ax = plt.subplots(nrows=2, sharex=True)

    def plot_forward_speed(self):
        return self.plot_d(['forward_speed', 'threshold_speed_estimation',
                            'threshold_speed_control'])

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
