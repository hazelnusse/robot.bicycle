#!/usr/bin/env python
from sympy.physics.mechanics import ReferenceFrame
from sympy import symbols, pi, Matrix, zeros
import numpy as np
import scipy.optimize as so
import sampletypes as st

def denormalize(x):
    return x * 16384.0 / 9.81

# Rotation angles and gravitational constant
psi, phi, theta = symbols('psi phi theta')
psi_s, phi_s, theta_s = symbols('psi_s phi_s, theta_s')
g = symbols('g')

# Unknown scale factors and biases
sxx, syy, szz = symbols('sxx syy szz')
sxy, sxz, syx, syz, szx, szy = symbols('sxy sxz syx syz szx szy')
bx, by, bz = symbols('bx by bz')
s = Matrix([[sxx, sxy, sxz], [syx, syy, syz], [szx, szy, szz]])
b = Matrix([[bx], [by], [bz]])
no_cross_axis_sensitivity = {sxy : 0.0, sxz : 0.0, syx : 0.0, syz : 0.0, szx :
        0.0, szy : 0.0}

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
datadir = "./imu_calibration/"
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
for i, configuration in enumerate(configurations):
    p = configuration[phi]
    t = configuration[theta]
    d01 = np.fromfile(configuration['files'][0], dtype=st.sample_t)
    d02 = np.fromfile(configuration['files'][1], dtype=st.sample_t)
    a01_mean = denormalize(np.array([d01['accx'], d01['accy'],
        d01['accz']]).mean(axis=1).T)
    a02_mean = denormalize(np.array([d02['accx'], d02['accy'],
        d02['accz']]).mean(axis=1).T)
    a_measured = Matrix((a01_mean + a02_mean)/2).T
    a_sym_sensor = Matrix([(-g*A.z & si).subs({g : 9.81, phi : p, theta : t}) for si in S])
    eqns[3*i], eqns[3*i + 1], eqns[3*i + 2] = a_sym_sensor - s*a_measured - b

unknowns = [phi_s, theta_s, psi_s, sxx, sxy, sxz, syx, syy, syz, szx, szy, szz,
        bx, by, bz]
eqns_dx = eqns.jacobian(unknowns)

def residual(x):
    return np.array(eqns.subs(dict(zip(unknowns,
        x))).subs(no_cross_axis_sensitivity).T.tolist()[0], np.float64)
    #return np.array(eqns.subs({phi_s : x[0], theta_s : x[1], psi_s : x[2],
    #                  sxx : x[3], sxy : 0.0, sxz : 0.0,
    #                  syx : 0.0, syy : x[7], syz : 0.0,
    #                  szx : 0.0, szy : 0.0, szz : x[11],
    #                  bx : x[12], by : x[13], bz : x[14]}).T.tolist()[0],
    #                  np.float64)

def jacobian(x):
    df = np.array(eqns_dx.subs(dict(zip(unknowns, x))).subs(no_cross_axis_sensitivity).tolist(), np.float64)
    z = np.zeros((18,))
    df[:, 4] = z
    df[:, 5] = z
    df[:, 6] = z
    df[:, 8] = z
    df[:, 9] = z
    df[:, 10] = z
    return df

s_nom = 9.81 / 16384.0
print("nominal sensitivity = {0}".format(s_nom))
x0 = [0.0, 0.0, 0.0,  # orientation angles
      s_nom, 0., 0.,           # x-axis sensitivities
      0., s_nom, 0.,           # y-axis sensitivities
      0., 0., s_nom,           # z-axis sensitivities
      0., 0., 0.]              # biases

x, cov_x, infodict, mesg, ier = so.leastsq(residual,
                                           x0,
                                           Dfun=jacobian,
                                           maxfev=1000,
                                           ftol=1e-16,
                                           xtol=1e-16,
                                           full_output=True)
print("Solution:\n", x)
print("Covariance:\n", cov_x)
print("Information:\n", infodict)
print("Message:\n", mesg)
print("Iterations:\n", ier)

sensor_orienation_angles = {phi_s : x[0], theta_s : x[1], psi_s : x[2]}

print("Direction cosine matrix:\n", C.dcm(S).subs(sensor_orienation_angles))
