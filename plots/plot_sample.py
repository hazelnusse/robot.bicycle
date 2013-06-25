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

class PlotSample(PlotData):
    def __init__(self, datafile=None):
        super(PlotSample, self).__init__('sample_pb2', 'Sample', datafile)
        self.default_x = None
        self._correct_system_time()
        self._add_sample_period()
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
