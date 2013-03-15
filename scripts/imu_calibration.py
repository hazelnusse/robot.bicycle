#!/usr/bin/env python
from sympy.physics.mechanics import ReferenceFrame
from sympy import symbols, pi, Matrix, zeros
import numpy as np
import scipy.optimize as so
import os
import sys
sys.path.append(os.path.join(os.getcwd(), "..", "common"))
import sampletypes as st

def denormalize(x):
    return x * 16384.0 / 9.81

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

# Rotation angles and gravitational constant
psi, phi, theta = symbols('psi phi theta')
psi_s, phi_s, theta_s = symbols('psi_s phi_s, theta_s')
g = symbols('g')

# Unknown scale factors and biases
sxx, syy, szz = symbols('sxx syy szz')
sxy, syz, sxz = symbols('sxy syz sxz')
bx, by, bz = symbols('bx by bz')
s = Matrix([[sxx, sxy, sxz], [sxy, syy, syz], [sxz, syz, szz]])
b = Matrix([[bx], [by], [bz]])
no_cross_axis_sensitivity = {sxy : 0.0, sxz : 0.0, syz : 0.0}
# Inertial frame
N = ReferenceFrame('N')

# Orient bicycle intermediate frames
A = N.orientnew('A', 'Axis', [psi, N.z])        # Bicycle yaw frame
B = A.orientnew('B', 'Axis', [phi, A.x])        # Bicycle roll frame
C = B.orientnew('C', 'Axis', [theta, B.y])      # Bicycle pitch frame

# Frames which orient sensor relative to bicycle frame
D = C.orientnew('D', 'Axis', [-pi/2 + phi_s, C.x])
E = D.orientnew('E', 'Axis', [pi + theta_s, D.z])
S = E.orientnew('S', 'Axis', [psi_s, E.y])

# 6 static configurations to put bicycle in and collect acclerometer data
datadir = "../data/imu_calibration/"
configurations = [{phi : 0, theta : 0,
                   'files' : [datadir + "savup01_converted.dat",
                              datadir + "savup02_converted.dat"]},
                  {phi : 0, theta : pi/2,
                   'files' : [datadir + "sahfup01_converted.dat",
                              datadir + "sahfup02_converted.dat"]},
                  {phi : 0, theta : -pi/2,
                   'files' : [datadir + "sahfd01_converted.dat",
                              datadir + "sahfd02_converted.dat"]},
                  {phi : 0, theta : pi,
                   'files' : [datadir + "savdo01_converted.dat",
                              datadir + "savdo02_converted.dat"]},
                  {phi :  pi/2, theta : 0,
                   'files' : [datadir + "dsidedown01_converted.dat",
                              datadir + "dsidedown02_converted.dat"]},
                  {phi : -pi/2, theta : 0,
                   'files' : [datadir + "dsideup01_converted.dat",
                              datadir + "dsideup02_converted.dat"]}]

eqns = zeros(18,1)
w_mean = np.zeros((12,3))
for i, configuration in enumerate(configurations):
    p = configuration[phi]
    t = configuration[theta]
    d01 = np.fromfile(configuration['files'][0], dtype=st.sample_t)
    d02 = np.fromfile(configuration['files'][1], dtype=st.sample_t)
    a01_mean = denormalize(np.array([d01['accx'],
                                     d01['accy'],
                                     d01['accz']]).mean(axis=1).T)
    a02_mean = denormalize(np.array([d02['accx'],
                                     d02['accy'],
                                     d02['accz']]).mean(axis=1).T)
    a_measured = Matrix((a01_mean + a02_mean)/2.0).T
    a_sym_sensor = Matrix([(-g*A.z & si).subs({g : 9.81, phi : p, theta : t}) for si in S])
    eqns[3*i], eqns[3*i + 1], eqns[3*i + 2] = a_sym_sensor - s*a_measured - b

    # Checking rate gyroscope signals
    w_mean[2*i + 0, :] = np.array([d01['gyrox'], d01['gyroy'], d01['gyroz']])[:,10:].mean(axis=1).T
    w_mean[2*i + 1, :] = np.array([d02['gyrox'], d02['gyroy'], d02['gyroz']])[:,10:].mean(axis=1).T

print("Body fixed angular velocity means for each run:")
print(w_mean)
print("Rate gyroscope sample means:")
print(w_mean.mean(axis=0))
print("Rate gyroscope sample standard deviations:")
print(w_mean.std(axis=0))

unknowns = [phi_s, theta_s, psi_s, sxx, syy, szz, sxy, syz, sxz, bx, by, bz]
eqns_dx = eqns.jacobian(unknowns)

def residual(x):
    return np.array(eqns.subs(dict(zip(unknowns, x))).T.tolist()[0], np.float64)

def jacobian(x):
    return np.array(eqns_dx.subs(dict(zip(unknowns, x))).tolist(), np.float64)

s_nom = 9.81 / 16384.0
print("nominal sensitivity = {0}".format(s_nom))
x0 = [0.0, 0.0, 0.0,           # orientation angles
      s_nom, s_nom, s_nom,     # main x,y,z sensitivities
      0.0, 0.0, 0.0,           # xy, yz, xz cross-axis sensitivities
      0.0, 0.0, 0.0]           # biases

x, cov_x, infodict, mesg, ier = so.leastsq(residual,
                                           x0,
                                           Dfun=jacobian,
                                           maxfev=100,
                                           ftol=1e-16,
                                           xtol=1e-16,
                                           full_output=True)
#print("Solution:\n", x)
#print("Covariance:\n", cov_x)
#print("Information:\n", infodict)
#print("Message:\n", mesg)
#print("MINPACK INFO:", ier)

print("Bicycle frame to sensor frame Euler XZY - (-pi/2 + phi_s, pi + theta_s, psi_s) angles")
print("phi_s: {0} rad [{1} deg]".format(x[0], x[0]*180/np.pi))
print("theta_s: {0} rad [{1} deg]".format(x[1], x[1]*180/np.pi))
print("psi_s: {0} rad [{1} deg]".format(x[2], x[2]*180/np.pi))

s_numerical = np.array([[x[3], x[6], x[8]],[x[6], x[4], x[7]], [x[8], x[7], x[5]]])
print("Sensitivity Matrix, m/s^2/bit:\n", s_numerical)
print("Biases, m/s^2:\n", x[9:])

print("Residuals, m/s^2:\n", infodict['fvec'])

sensor_orienation_angles = {phi_s : x[0], theta_s : x[1], psi_s : x[2]}
dcm = C.dcm(S).subs(sensor_orienation_angles)
print("Direction cosine matrix (S to C):\n", dcm)

generate_header(dcm, w_mean.mean(axis=0))
