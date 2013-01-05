#!/usr/bin/env python
from sympy.physics.mechanics import ReferenceFrame
from sympy import symbols, pi, lambdify
import sampletypes as st
import numpy as np
import scipy.optimize as so
import sys

phi, theta, g = symbols('phi theta g')
# Invensense MPU-6050 accelerometer/gyroscope sensor is fixed to bicycle
# frame in roughly the following orientation:
# x - axis -- parallel to down tube, positive direction towards downtube
# y - axis -- perpendicular to down tube, positive towards ground
# z - axis -- perpendicular to frame plane, positive to right
A = ReferenceFrame('A')                    # Inertial frame, z is downward
B = A.orientnew('B', 'Axis', [phi, A.x])   # Lean frame
C = B.orientnew('C', 'Axis', [theta, B.y]) # Pitch frame
D = C.orientnew('D', 'Axis', [pi, C.z])    # Sensor intermediate frame
E = D.orientnew('E', 'Axis', [pi/2, D.x])  # Sensor frame

def compute_orientation(datafile):
    # Function that maps lean, pitch, and gravitational constant to idealized
    # accelerometer measurements
    f_estimate_g = lambdify((phi, theta, g),
                            [-g*A.z & ei for ei in [E.x, E.y, E.z]])

    d = np.fromfile(datafile, dtype=st.sample_t)
    acc_mean = np.array([d['accx'], d['accy'], d['accz']]).mean(axis=1).T
    acc_std = np.array([d['accx'], d['accy'], d['accz']]).std(axis=1).T

    def residual_estimate_g(beta, y):
        return f_estimate_g(beta[0], beta[1], beta[2]) - y

    # Initial guess for lean, pitch, and gravity
    x0 = [0.0, np.pi/4.0, 9.81]

    x, cov_x, infodict, mesg, ier = so.leastsq(residual_estimate_g,
                                               x0,
                                               args=acc_mean,
                                               maxfev=10000,
                                               full_output=True)
    print("Orientation computed:")
    print("phi = {0}\ntheta = {1} deg\ng = {2} m/s^2".format(x[0]*180/np.pi,
                                                             x[1]*180/np.pi,
                                                             x[2]))
    dcm = A.dcm(E).subs({phi:x[0], theta:x[1]})
    return x, dcm

def compute_offsets(datafile):
    d = np.fromfile(datafile, dtype=st.sample_t)
    gyro_mean = np.array([d['gyrox'], d['gyroy'], d['gyroz']]).mean(axis=1).T
    gyro_std = np.array([d['gyrox'], d['gyroy'], d['gyroz']]).std(axis=1).T
    print("Gyroscope offsets:")
    print("w_x = {0} +/- {1}".format(gyro_mean[0], gyro_std[0]))
    print("w_y = {0} +/- {1}".format(gyro_mean[1], gyro_std[1]))
    print("w_z = {0} +/- {1}".format(gyro_mean[2], gyro_std[2]))
    return gyro_mean

def generate_header(dcm, gyro_offsets):
    s = ("#ifndef IMU_CALIBRATION_H\n" +
         "#define IMU_CALIBRATION_H\n" +
         "class imu_calibration {\n" +
         " public:\n" +
         "  static constexpr float wx = {0}f;\n".format(gyro_offsets[0])  +
         "  static constexpr float wy = {0}f;\n".format(gyro_offsets[1])  +
         "  static constexpr float wz = {0}f;\n".format(gyro_offsets[2])  +
         "  static constexpr float dcm[6] = {" + "{0}f,\n".format(dcm[0, 0]) +
         "                                    {0}f,\n".format(dcm[1, 1]) +
         "                                    {0}f,\n".format(dcm[2, 2]) +
         "                                    {0}f,\n".format(dcm[0, 1]) +
         "                                    {0}f,\n".format(dcm[1, 2]) +
         "                                    {0}f".format(dcm[0, 2]) + "};\n" +
         "};\n" +
         "#endif")
    f = open("imu_calibration.h", 'w')
    f.write(s)
    f.close()

def main(filename):
    (phi, theta, g), dcm = compute_orientation(filename)
    print(dcm)
    gyro_offsets = compute_offsets(filename)
    # Generate header file with appropriately defined gyroscope offsets.
    generate_header(dcm, gyro_offsets)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Must supply a single filename")
        exit()
    else:
        main(sys.argv[1])
