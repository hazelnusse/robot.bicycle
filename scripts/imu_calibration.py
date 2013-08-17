#!/usr/bin/env python
from sympy.physics.mechanics import ReferenceFrame
from sympy import symbols, pi, Matrix, zeros
import numpy as np
import scipy.optimize as so
import os
import sys
sys.path.append(os.path.join(os.getcwd(), "..", "common"))
import sampletypes as st

# Rotation angles and gravitational constant
psi, phi, theta = symbols('psi phi theta')
alpha, beta, gamma = symbols('alpha beta gamma')
g = symbols('g')
g_numerical = 9.81

# Unknown scale factors and biases
sxx, syy, szz = symbols('sxx syy szz')
sxy, syz, sxz = symbols('sxy syz sxz')
bx, by, bz = symbols('bx by bz')
S = Matrix([[sxx, sxy, sxz], [sxy, syy, syz], [sxz, syz, szz]])
b = Matrix([[bx], [by], [bz]])
no_cross_axis_sensitivity = {sxy : 0.0, sxz : 0.0, syz : 0.0}
# Inertial frame
N = ReferenceFrame('N')

# Orient bicycle intermediate frames
A = N.orientnew('A', 'Axis', [psi, N.z])        # Bicycle yaw frame
B = A.orientnew('B', 'Axis', [phi, A.x])        # Bicycle lean frame
C = B.orientnew('C', 'Axis', [theta, B.y])      # Bicycle pitch frame

# Frames which orient sensor relative to bicycle frame
D = C.orientnew('D', 'Axis', [-pi/2 + alpha, C.z])
E = D.orientnew('E', 'Axis', [beta, D.x])
F = E.orientnew('F', 'Axis', [gamma, E.y])

# 6 static configurations to put bicycle in and collect acclerometer data
datadir = "../data/imu_calibration/"
configurations = [{phi: 0, theta: 0,
                   'data_file': datadir + "phi_000_theta_000_mpu6050.npz"},
                  {phi: 0, theta: pi/2,
                   'data_file': datadir + "phi_000_theta_090_mpu6050.npz"},
                  {phi: 0, theta: pi,
                   'data_file': datadir + "phi_000_theta_180_mpu6050.npz"},
                  {phi: 0, theta: -pi/2,
                   'data_file': datadir + "phi_000_theta_270_mpu6050.npz"},
                  {phi: pi/2, theta: 0,
                   'data_file': datadir + "phi_090_theta_000_mpu6050.npz"},
                  {phi: -pi/2, theta: 0,
                   'data_file': datadir + "phi_270_theta_000_mpu6050.npz"}]

eqns = zeros(18, 1)
w_mean = np.zeros((6, 3))
for i, configuration in enumerate(configurations):
    p = configuration[phi]
    t = configuration[theta]
    data = np.load(configuration['data_file'])
    acc_mean = Matrix(np.array([data['accelerometer_x'],
                         data['accelerometer_y'],
                         data['accelerometer_z']]).mean(axis=1)).T
    gyro_mean = np.array([data['gyroscope_x'],
                          data['gyroscope_y'],
                          data['gyroscope_z']]).mean(axis=1).T
    a_sym_sensor = Matrix([(-g*A.z & uv).subs({g : g_numerical, phi : p, theta : t})
                           for uv in F])
    eqns[3*i], eqns[3*i + 1], eqns[3*i + 2] = a_sym_sensor - S*acc_mean - b

    # Checking rate gyroscope signals
    w_mean[i, :] = gyro_mean

print("Body fixed angular velocity means for each run:")
print(w_mean)
print("Rate gyroscope sample means:")
print(w_mean.mean(axis=0))
print("Rate gyroscope sample standard deviations:")
print(w_mean.std(axis=0))

unknowns = [alpha, beta, gamma, sxx, syy, szz, sxy, syz, sxz, bx, by, bz]
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

print("Bicycle frame to sensor frame Euler ZXY - "
      "(-pi/2 + alpha, beta, gamma) angles")
print("alpha: {0} rad [{1} deg]".format(x[0], x[0]*180/np.pi))
print("beta: {0} rad [{1} deg]".format(x[1], x[1]*180/np.pi))
print("gamma: {0} rad [{1} deg]".format(x[2], x[2]*180/np.pi))

s_numerical = np.array([[x[3], x[6], x[8]],
                        [x[6], x[4], x[7]],
                        [x[8], x[7], x[5]]])
print("Sensitivity Matrix, m/s^2/bit:\n", s_numerical)
b_numerical = x[9:]
print("Biases, m/s^2:\n", b_numerical)

print("Residuals, m/s^2:\n", infodict['fvec'])

sensor_orienation_angles = {alpha: x[0], beta: x[1], gamma: x[2]}
dcm_F_to_C = C.dcm(F).subs(sensor_orienation_angles)
print("Direction cosine matrix (sensor frame to bicycle pitch "
      "frame):\n{0}".format(dcm_F_to_C))

reference = np.load(datadir + "reference_mpu6050.npz")
acc_sensor_frame = np.array([reference['accelerometer_x'],
                             reference['accelerometer_y'],
                             reference['accelerometer_z']]).mean(axis=1)
acc_sensor_frame = np.dot(s_numerical, acc_sensor_frame) + b_numerical
print("Accelerometer sensor frame:")
print(acc_sensor_frame)

acc_pitch_frame = np.dot(np.array(dcm_F_to_C.tolist(), dtype=np.float64),
                         acc_sensor_frame)
print("Accelerometer pitch frame:")
print(acc_pitch_frame)
print(acc_pitch_frame[1])
print(type(acc_pitch_frame[1]))

gravity_in_pitch_frame = [-g*A.z & uv for uv in C]
print(gravity_in_pitch_frame)
# Sensed gravity vector expressed in bicycle pitch coordinates:
# [g*sin(theta)*cos(phi), -g*sin(phi), -g*cos(phi)*cos(theta)]
lean = np.arcsin(acc_pitch_frame[1]/-g_numerical)
print("Reference lean = {0}".format(lean))
pitch_1 = np.arcsin(acc_pitch_frame[0]/g_numerical/np.cos(lean))
pitch_2 = np.arccos(acc_pitch_frame[2]/-g_numerical/np.cos(lean))
print("Reference pitch 1 = {0} = {1}".format(pitch_1, pitch_1 * 180 / np.pi))
print("Reference pitch 2 = {0} = {1}".format(pitch_2, pitch_2 * 180 / np.pi))
pitch = (pitch_1 + pitch_2) / 2.0
print("Average reference pitch = {0}".format(pitch))

dcm_F_to_B = B.dcm(F).subs(sensor_orienation_angles).subs(theta, pitch)
print("Direction cosine matrix (sensor frame to bicycle lean "
      "frame):\n{0}".format(dcm_F_to_B))

dcm_B_to_A = A.dcm(B)
print("Direction cosine matrix (lean frame to yaw frame)"
      ":\n{0}".format(dcm_B_to_A))

