#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Class used for plotting data of a Message_np object. 

    Examples
    ========
    p = PlotData('sample_pb2', 'Sample', 'samples.dat')
    p.print_fields()
    p.plot('system_time', 'mpu6050', norm=True)
    p.plot('system_time', 'mpu6050.accelerometer_x')
    p.plot('system_time', ['estimate', 'yaw_rate_PI'])
"""
from collections import OrderedDict
import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "proto"))

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as mplcm
import numpy as np
from message_np import Message_np


SUBTYPE_SEP = '.'
PLOTY_SEP = ', '

class PlotData(object):
    """Class used for plotting data of a Message_np object.
    """
    def __init__(self, pb_file=None, message_type=None, datafile=None):
        self.samples_np = Message_np(pb_file, message_type, datafile)
        self.data = np.ma.array(self.samples_np.get_messages_np())
        self.dtype = self.samples_np.message_npdtype_map[message_type]
        self.full_fields = []
        self.cm = plt.get_cmap('gist_rainbow')
        self.dtype_c = {}
        self.data_c = {}
        self._set_full_fields()

    def _get_field_type_data(self, field):
        """Returns the datatype and data for the specified field.
        """
        if field in self.dtype_c:
            return (self.dtype_c[field], self.data_c[field])

        rdata = self.data
        rdtype = self.dtype
        for subfield in field.split(SUBTYPE_SEP):
            rdata = rdata[subfield]
            rdtype = rdtype[subfield]
        return (rdtype, rdata)

    def get_field_type(self, field):
        """Returns the datatype for the specified field.
        """
        return self._get_field_type_data(field)[0]

    def get_field_data(self, field):
        """Returns the data for the specified field.
        """
        return self._get_field_type_data(field)[1]

    def _set_full_fields(self):
        """Set the list of full field names.

        Sets a list of the field names for the message type. If a field has
        subfields, the an entry is added for each subfield, where the field and
        subfield with a seperator between the two. The superfield is excluded
        from this list.

        Example
        =======

        Message A has fields a1, a2, a3 where a1 is of type Message B.
        Message B has fields b1, b2.

        self.full_fields = ['a1.b1', 'a1.b2', 'a2', 'a3']
        """
        self.full_fields = []
        field_stack = self.dtype.fields.keys()

        while field_stack:
            field = field_stack.pop()
            subfields = self.get_field_type(field).fields
            if not subfields:
                self.full_fields.insert(0, field)
            else:
                for subfield in sorted(subfields.keys()):
                    field_stack.append(SUBTYPE_SEP.join([field, subfield]))

    def expand_field(self, field):
        """Returns a list of all the full field names for each subfield in
        'field'. Returns a list with single element 'field' if it has no
        subfields.
        """
        if field in self.dtype_c:
            return [field]

        subfields = self.get_field_type(field).fields
        if subfields:
            return [ff for ff in self.full_fields if ff.startswith(field)]
        return [field]

    def print_fields(self):
        """Print the data fields.
        """
        def print_stats(data):
            r = "\t"*4
            return r + "\t".join(str(x) for x in [np.min(data), np.max(data),
                                                  np.mean(data), np.std(data)])
        print("\t"*6 + "min\tmax\tmean\tstd")

        subfield_prefix = "\t"
        print_fields = OrderedDict()
        for ff in self.full_fields:
            superfield = ""
            for subfield in ff.split(SUBTYPE_SEP):
                superfield = (SUBTYPE_SEP.join([superfield, subfield])
                              if superfield else subfield)
                print_fields[superfield] = True

        initial_offset = -1
        if self.dtype_c.keys():
            print("Initial fields:")
            initial_offset += 1

        for pf in print_fields:
            subfields = self.get_field_type(pf).fields
            if subfields:
                stats = ""
            else:
                stats = print_stats(self.get_field_data(pf))

            subfields = pf.split(SUBTYPE_SEP)
            print(subfield_prefix * (len(subfields) + initial_offset) +
                  subfields[-1] + stats)

        if self.dtype_c.keys():
            print("\nCreated fields:")
        for field in sorted(self.dtype_c.keys()):
            stats = print_stats(self.data_c[field])
            print(subfield_prefix + field + stats)

    def _set_color_cycle(self, axes, num_colors):
        """Set the color cycle for the plots in an axes.
        """
        c_norm = colors.Normalize(vmin=0, vmax=num_colors-1)
        scalar_map = mplcm.ScalarMappable(norm=c_norm, cmap=self.cm)
        axes.set_color_cycle([scalar_map.to_rgba(i) for i in range(num_colors)])

    def plot(self, x, y, data=[], norm=False, *args, **kwargs):
        """Returns the figure used in plotting field 'y' vs field 'x'.

        'x' and 'y' are fields as shown in print_fields().  'y' can be a list of
        fields and will result in all plots displayed on the same figure.
        'data' is an array of data that can be plotted against 'x'.
        'norm' can be used to normalize each 'y'.
        'args' and 'kwargs' are passed to axes.plot().
        """
        fig, ax = plt.subplots(1)
        x_field = self.expand_field(x)[0]
        x_data = self.get_field_data(x_field)
        if not isinstance(y, list):
            y = [y]

        y_fields = [y_field for y_ in y for y_field in self.expand_field(y_)]
        self._set_color_cycle(ax, len(y_fields) + len(data) + 1)

        for y_field in y_fields:
            y_data = self.get_field_data(y_field)
            mag = 1.0
            if norm:
                mag = np.float(np.amax(np.absolute(y_data)))
                if mag < 1e-12:
                    mag = 1.0
            ax.plot(x_data, y_data / mag, label=y_field, *args, **kwargs)
        for d in data:
            ax.plot(x_data, d, *args, **kwargs)

        y_label = PLOTY_SEP.join(y) + (" (normalized)" if norm else "")
        ax.set_xlabel(x_field)
        ax.set_ylabel(y_label)
        ax.set_title(y_label + ' vs. ' + x_field)
        ax.legend()
        plt.show()
        return fig

    def hist(self, field=None, data=None, *args, **kwargs):
        """Plots a histogram for the specified 'field' or with 'data' using the
        same arguments as matplotlib.pyplot.hist().
        """
        if field is None and data is None:
            return
        fig, ax = plt.subplots(1)
        if field:
            efield = self.expand_field(field)[0]
            data = self.get_field_data(efield)
        ax.hist(data, *args, **kwargs)
        ax.set_xlabel(efield)
        ax.set_ylabel('occurances')
        ax.set_title('histogram of {0}\nmean = {1}, stddev = {2}'.format(
                efield, np.mean(data), np.std(data)))
        plt.show()
        return fig
