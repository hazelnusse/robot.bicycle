#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "proto"))

import numpy as np
from plot_data import PlotData

class PlotSample(PlotData):
    def __init__(self, datafile=None):
        super(PlotSample, self).__init__('sample_pb2', 'Sample', datafile)
        self.default_x = None
        self._fix_system_time()
        self._add_real_time()
        self.set_default_x('real_time')

    def _fix_system_time(self):
        type_ = 'system_time_c'
        st_data = self.get_field_data('system_time')
        self.dtype_c[type_] = np.uint64
        self.data_c[type_] = np.empty(st_data.shape,
                                      dtype=self.dtype_c[type_])
        prev_t = 0
        offset = 0
        for i, t in enumerate(st_data):
            if prev_t > t:
                offset += 2**32
            self.data_c[type_][i] = (st_data[i] + offset)
            prev_t = t

    def _add_real_time(self):
        type_ = 'real_time'
        self.data_c[type_] = self.data_c['system_time_c'] * 0.25e-6
        self.dtype_c[type_] = type(self.data_c[type_][0])

    def set_default_x(self, field):
        self.default_x = field;

    def plot_d(self, *arg):
        print(arg)
        self.plot(self.default_x, *arg)
