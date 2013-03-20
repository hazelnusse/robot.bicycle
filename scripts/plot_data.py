#!/usr/bin/env python3
"""Command line tool for plotting data acquired from robot.bicycle
Requires data to be converted.
"""

import os
import sys
sys.path.append(os.path.join(os.getcwd(), "..", "..", "common"))

import argparse
import sys
import matplotlib.pyplot as plt
import sampleplotter as sp


def plot(args):
    samples = sp.Samples(args.datafile, args.plots)
    samples.draw_plots()
    plt.show()
    return


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
    parser.add_argument("-w", "--wheel", dest="plots", action="append_const",
                        const="PlotRearWheel", help="Plot Rear Wheel")
    parser.add_argument("-t", "--time", dest="plots", action="append_const",
                        const="PlotTime", help="Plot Time")
    parser.add_argument("-a", "--accel", dest="plots", action="append_const",
                        const="PlotAccelerometer", help="Plot Accelerometer")
    parser.add_argument("-g", "--gyro", dest="plots", action="append_const",
                        const="PlotGyroscope", help="Plot Gyroscope")
    parser.add_argument("-T", "--temp", dest="plots", action="append_const",
                        const="PlotTemperature", help="Plot Temperature")
    parser.add_argument("-s", "--steer", dest="plots", action="append_const",
                        const="PlotSteerAngle", help="Plot Steer Angle")
    parser.add_argument("-S", "--state", dest="plots", action="append_const",
                        const="PlotState", help="Plot State")
    args = parser.parse_args()
    plot(args)
#plot(interpret_args(args))
