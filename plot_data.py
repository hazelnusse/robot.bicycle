#!/usr/bin/env python3
"""Command line tool for plotting data acquired from robot.bicycle
Requires data to be converted.
"""

import argparse
import sys
import matplotlib.pyplot as plt
import sampleplotter as sp


def plot(args):
    samples = sp.Samples(args.datafile)
    if (args.wheel):
        samples.rearwheel.plot_data()
    if (args.time):
        samples.time.plot_data()
    if (args.accel):
        samples.accelerometer.plot_data()
    if (args.gyro):
        samples.gyroscope.plot_data()
    if (args.temp):
        samples.temperature.plot_data()
#    if (args.state):
#        samples.state.plot_data()
    plt.show()

def interpret_args(args):
    if not (args.wheel or args.time or args.accel or
            args.gyro or args.temp or args.state):
        args.wheel = True
        args.time = True
        args.accel = True
        args.gyro = True
        args.temp = True
        args.state = True
    return args


if __name__ == "__main__":
    class Parser(argparse.ArgumentParser):
        def error(self, message):
            sys.stderr.write("error: %s\n" % message)
            self.print_help()
            sys.exit(2)

    parser = Parser(description="Plot data acquired from robot.bicycle",
                epilog="All plots are displayed if no option(s) is selected.",
                usage="%(prog)s [options] datafile")
    parser.add_argument("datafile", help="Data to plot")
    parser.add_argument("-w", "--wheel", action="store_true",
                        help="Plot Rear Wheel")
    parser.add_argument("-t", "--time", action="store_true",
                        help="Plot Time")
    parser.add_argument("-a", "--accel", action="store_true",
                        help="Plot Accelerometer")
    parser.add_argument("-g", "--gyro", action="store_true",
                        help="Plot Gyroscope")
    parser.add_argument("-T", "--temp", action="store_true",
                        help="Plot Temperature")
    parser.add_argument("-s", "--state", action="store_true",
                        help="Plot State")
    args = parser.parse_args()
    plot(interpret_args(args))
