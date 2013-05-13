#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "proto"))

import numpy as np
from plot_data import PlotData

SECONDS_PER_CLOCK = 0.25e-6

class PlotSample(PlotData):
    def __init__(self, datafile=None):
        super(PlotSample, self).__init__('sample_pb2', 'Sample', datafile)
        self.default_x = None
        self._correct_system_time()
        self._add_sample_period()
        self._convert_clocks_to_seconds(['system_time_c', 'computation_time',
                                         'sample_period'])
        self.set_default_x('system_time_s')

    def _correct_system_time(self):
        field = 'system_time_c'
        systime_data = self.get_field_data('system_time')
        self.dtype_c[field] = np.uint64
        self.data_c[field] = np.empty(systime_data.shape,
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
        systime_dt[0] = 0;
        self.data_c[field] = systime_dt
        self.dtype_c[field] = type(self.data_c[field][0])

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

    def plot_d(self, *arg):
        print(arg)
        self.plot(self.default_x, *arg)
