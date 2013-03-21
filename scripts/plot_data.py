#!/usr/bin/env python3
"""Command line tool for plotting data acquired from robot.bicycle
Requires data to be converted.
"""

import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "common"))

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
    parser.add_argument("--theta_R_dot", dest="plots", action="append_const",
                        const="PlotRearWheelRate", help="Plot estimated " +
                                                        "rear wheel rate")
    parser.add_argument("--phi_dot", dest="plots", action="append_const",
                        const="PlotRollRate", help="Plot estimated " +
                                                        "roll rate")
    parser.add_argument("--controller_states", dest="plots", action="append_const",
                        const="PlotControllerStates", help="Plot " +
                                                        "controller states")
    parser.add_argument("--steer_torque_current", dest="plots", action="append_const",
                        const="PlotSteerTorqueAndCurrent", help="Plot " +
                                                        "steer torque and current")
    args = parser.parse_args()
    plot(args)

