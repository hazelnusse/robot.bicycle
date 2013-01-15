#!/usr/bin/env python
from sympy.physics.mechanics import ReferenceFrame
from sympy import symbols, pi, lambdify
import sampletypes as st
import numpy as np
import scipy.optimize as so
import sys

# Rotation angles and gravitational constant
psi, phi, theta = symbols('psi phi theta')
psi_s, phi_s, theta_s = symbols('psi_s phi_s theta_s')
g = symbols('g')

N = ReferenceFrame('N')                              # Inertial frame, z is downward
# Rear frame of bicycle reference frames
A = N.orientnew('A', 'Axis', [psi, N.z])        # Bicycle yaw frame
B = A.orientnew('B', 'Axis', [phi, A.x])        # Bicycle roll frame
C = B.orientnew('C', 'Axis', [theta, B.y])      # Bicycle pitch frame

# Invensense MPU-6050 accelerometer/gyroscope sensor is fixed to bicycle
# frame in roughly the following orientation:
# x - axis -- parallel to down tube, positive direction towards downtube
# y - axis -- perpendicular to down tube, positive towards ground
# z - axis -- perpendicular to frame plane, positive to right
# Begin with the sensor axes aligned with an instantaneous yaw frame
A_s = A.orientnew('A_s', 'Axis', [psi_s, A.z])  # Sensor yaw frame
B_s = A_s.orientnew('B_s', 'Axis', [phi_s, A_s.x])   # Sensor roll frame
C_s = B_s.orientnew('C_s', 'Axis', [theta_s, B_s.y]) # Sensor pitch frame
D_s = C_s.orientnew('D_s', 'Axis', [pi, C_s.z])      # Sensor intermediate frame
S = D_s.orientnew('S', 'Axis', [pi/2, D_s.x])    # Sensor frame
# The last two frames are purely for convenience so that roll and pitch of
# the sensor have the same meaning as roll and pitch of the bicycle frame


def compute_orientation(datafile):
    # Function that maps sensor roll, sensor pitch, and gravitational constant
    # to idealized accelerometer measurements
    f_estimate_g = lambdify((phi_s, theta_s, g),
                            [-g*N.z & ei for ei in [S.x, S.y, S.z]])

    d = np.fromfile(datafile, dtype=st.sample_t)
    acc_mean = np.array([d['accx'], d['accy'], d['accz']]).mean(axis=1).T
    acc_std = np.array([d['accx'], d['accy'], d['accz']]).std(axis=1).T

    def residual_estimate_g(beta, y):
        return f_estimate_g(beta[0], beta[1], beta[2]) - y

    # Initial guess for roll, pitch, and gravity
    x0 = [0.0, np.pi/4.0, 9.81]

    x, cov_x, infodict, mesg, ier = so.leastsq(residual_estimate_g,
                                               x0,
                                               args=acc_mean,
                                               maxfev=10000,
                                               full_output=True)
    print("Orientation computed:")
    print("phi_s = {0}\ntheta_s = {1} deg\ng = {2} m/s^2".format(x[0]*180/np.pi,
                                                             x[1]*180/np.pi,
                                                             x[2]))
    dcm = A.dcm(S).subs({phi_s:x[0], theta_s:x[1]})
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
    (phi_s, theta_s, g), dcm = compute_orientation(filename)
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
