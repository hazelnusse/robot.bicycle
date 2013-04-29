#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "proto"))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as mplcm
from message_np import Message_np


SUBTYPE_SEPERATOR = '.'
PLOTY_SEPERATOR = ', '

class PlotData:
    def __init__(self, pb_file=None, message_type=None, datafile=None):
        self.samples_np = Message_np(pb_file, message_type, datafile)
        self.data = self.samples_np.get_messages_np()
        self.dtype = self.samples_np.message_npdtype_map[message_type]
        self.cm = plt.get_cmap('gist_rainbow')

    def print_fields(self):
        self._print_fields_util(self.dtype, '', '    ')

    def _print_fields_util(self, message_type, prefix, child_prefix):
        for name, type in getattr(message_type, 'fields').iteritems():
            print(prefix + name)
            if getattr(message_type[name], 'fields'):
                self._print_fields_util(message_type[name],
                                        prefix + child_prefix, child_prefix)

    def _expand_nested_data(self, data, dtype, type):
        if SUBTYPE_SEPERATOR in type:
            supertype, _, subtype = type.partition(SUBTYPE_SEPERATOR)
            requested_data = self._expand_nested_data(data[supertype],
                                                      dtype[supertype],
                                                      subtype)
            return [(supertype + SUBTYPE_SEPERATOR + type_name, type_data)
                    for type_name, type_data in requested_data]
        subtypes = dtype[type].fields
        if subtypes:
            requested_data = []
            for subtype in subtypes.keys():
                subtype_data = self._expand_nested_data(data[type],
                                                        dtype[type],
                                                        subtype)
                requested_data.extend(
                    [(type + SUBTYPE_SEPERATOR + sub2type_name, sub2type_data)
                     for sub2type_name, sub2type_data in subtype_data])
            return requested_data
        return [(type, data[type])]

    def _set_color_cycle(self, axes, num_colors):
        c_norm = colors.Normalize(vmin=0, vmax=num_colors-1)
        scalar_map = mplcm.ScalarMappable(norm=c_norm, cmap=self.cm)
        axes.set_color_cycle([scalar_map.to_rgba(i) for i in range(num_colors)])

    def plot(self, x, ys, options=None):
        if options is None:
            options = {}
        fig, ax = plt.subplots(1)
        datas_x = self._expand_nested_data(self.data, self.dtype, x)
        type_x, data_x = datas_x[0] # get first elem if there are more than one
        if not isinstance(ys, list):
            ys = [ys]

        plots_y = []
        for y in ys: # get number of plots
            datas_y = self._expand_nested_data(self.data, self.dtype, y)
            for type_y, data_y in datas_y:
                plots_y.append((type_y, data_y))
        self._set_color_cycle(ax, len(plots_y))
        plots_y.sort()
        for type_y, data_y in plots_y:
            ax.plot(data_x, data_y, label=type_y, **options)

        ylabel = PLOTY_SEPERATOR.join(y for y in ys)
        ax.set_xlabel(type_x)
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel + ' vs. ' + type_x)
        ax.legend()
        plt.show()

if __name__ == '__main__':
    p = PlotData('sample_pb2', 'Sample', 'samples.dat')
    p.plot('system_time', 'mpu6050')
    p.plot('system_time', 'mpu6050.accelerometer_x')
    p.plot('system_time', ['estimate', 'yaw_rate_PI'])
