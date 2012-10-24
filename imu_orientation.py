from sympy.physics.mechanics import ReferenceFrame
from sympy import symbols, pi, lambdify
import sampletypes as st
import numpy as np
import scipy.optimize as so
import sys

def compute_orientation(datafiles):
    phi, theta, g = symbols('phi theta g')
    # Invensense MPU-6050 accelerometer/gyroscope sensor is fixed to bicycle frame
    # in the following orientation:
    # x - axis -- parallel to down tube, positive direction towards downtube
    # y - axis -- perpendicular to down tube, positive towards ground
    # z - axis -- perpendicular to frame plane, positive to right
    A = ReferenceFrame('A')                    # Inertial frame, z is downward
    B = A.orientnew('B', 'Axis', [phi, A.x])   # Lean frame
    C = B.orientnew('C', 'Axis', [theta, B.y]) # Pitch frame
    D = C.orientnew('D', 'Axis', [pi, C.z])    # Sensor intermediate frame
    E = D.orientnew('E', 'Axis', [pi/2, D.x])  # Sensor frame

    print([-g*A.z & ei for ei in [E.x, E.y, E.z]])
    # Function that maps lean, pitch, and gravitational constant to idealized
    # accelerometer measurements
    f = lambdify((phi, theta, g), [-g*A.z & ei for ei in [E.x, E.y, E.z]])

    for datafile in datafiles:
        data = np.fromfile(datafile, dtype=st.sample_t)
        data_mean = np.array([data['accx'], data['accy'], data['accz']]).mean(axis=1).T

        print("Mean of collected data:\n{0}".format(data_mean))

        def residual(beta, y):
            return f(beta[0], beta[1], beta[2]) - y

        # Initial guess for lean, pitch, and gravity
        x0 = [0.0, np.pi/4.0, 9.81]

        x, cov_x, infodict, mesg, ier= so.leastsq(residual, x0, args=data_mean,
                                                  maxfev=10000, full_output=True)
        print("phi, theta, g = {0} deg, {1} deg, {2} m/s^2".format(x[0]*180/np.pi,
                                                                   x[1]*180/np.pi,
                                                                   x[2]))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Must supply a filename or filenames")
        exit()
    else:
        compute_orientation(sys.argv[1:])
