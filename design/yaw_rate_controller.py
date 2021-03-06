"""Observer based yaw rate controller with integral action.

"""
from parameters import rear, w
import scipy.io as sio
from scipy.signal import cont2discrete, bode, lti, ss2tf, dstep, freqz, ss2zpk
from scipy.optimize import curve_fit
import numpy as np
from numpy import pi, dot
from numpy.matlib import zeros, empty, eye, asmatrix, diag
import numpy.linalg as la
np.set_printoptions(precision=4)
import matplotlib.pyplot as plt
from control_tools import dare
from controller_data import controller_t
from control.statefbk import ctrb, obsv
from control import ss, series, feedback

# Load data generated by continuous time model of robot bicycle
data = np.load('robotic_bicycle_linear_dynamics_vs_logspeed.npz')
radius = rear.R

# Load data generated by continuous time model of benchmark bicycle
#data = np.load('benchmark_bicycle_linear_dynamics_vs_logspeed.npz')
#radius = w.rR

skip = 1
theta_R_dot = data['theta_R_dot'][::skip]
A_w = data['A_w'][::skip]
B_w = data['B_w'][::skip]
C_w = data['C_w'][::skip]   # steer, roll rate, yaw rate measurements
N = len(theta_R_dot)
dt = 0.005

def plot_evals():
    evals = np.zeros((N, 4), dtype=np.complex128)
    for i, A in enumerate(A_w):
        evals[i], _ = la.eig(A)

    plt.figure()
    plt.plot(-theta_R_dot*radius, evals.real, 'k.')
    plt.plot(-theta_R_dot*radius, evals.imag, 'b.')

def compute_gains(Q, R, W, V, dt):
    """Given LQR Q and R matrices, and Kalman W and V matrices, and sample
    time, compute optimal feedback gain and optimal filter gains."""

    data = np.empty((N,), dtype=controller_t)

    # Loop over all speeds for which we have system dynamics
    for i in range(N):
        data['theta_R_dot'][i] = theta_R_dot[i]
        data['dt'][i] = dt
        # Convert the bike dynamics to discrete time using a zero order hold
        data['A'][i], data['B'][i], _, _, _ = cont2discrete(
                        (A_w[i], B_w[i, :], eye(4), zeros((4, 1))), dt)
        data['plant_evals_d'][i] = la.eigvals(data['A'][i])
        data['plant_evals_c'][i] = np.log(data['plant_evals_d'][i]) / dt
        
        # Bicycle measurement matrices
        # - steer angle
        # - roll rate
        data['C_m'][i] = C_w[i, :2, :]
        # - yaw rate
        data['C_z'][i] = C_w[i, 2, :]

        A = data['A'][i]
        B = data['B'][i, :, 2].reshape((4, 1))
        C_m = data['C_m'][i]
        C_z = data['C_z'][i]

        # Controllability from steer torque
        data['ctrb_plant'][i] = ctrb(A, B)
        u, s, v = la.svd(data['ctrb_plant'][i])
        assert(np.all(s > 1e-13))

        # Solve discrete algebraic Ricatti equation associated with LQI problem
        P_c = dare(A, B, R, Q)
        
        # Optimal feedback gain using solution of Ricatti equation
        K_c = -la.solve(R + dot(B.T, dot(P_c, B)),
                                dot(B.T, dot(P_c, A)))
        data['K_c'][i] = K_c
        data['A_c'][i] = A + dot(B, K_c)
        data['B_c'][i] = B
        data['controller_evals'][i] = la.eigvals(data['A_c'][i])
        data['controller_evals_c'][i] = np.log(data['controller_evals'][i]) / dt
        assert(np.all(abs(data['controller_evals'][i]) < 1.0))

        # Observability from steer angle and roll rate measurement
        # Note that (A, C_m * A) must be observable in the "current estimator"
        # formulation
        data['obsv_plant'][i] = obsv(A, dot(C_m, A))
        u, s, v = la.svd(data['obsv_plant'][i])
        assert(np.all(s > 1e-13))

        # Solve Riccati equation
        P_e = dare(A.T, C_m.T, V, W)
        # Compute Kalman gain
        K_e = dot(P_e, dot(C_m.T, la.inv(dot(C_m, dot(P_e, C_m.T)) + V)))
        data['K_e'][i] = K_e
        data['A_e'][i] = dot(eye(4) - dot(K_e, C_m), A)
        data['B_e'][i] = np.hstack((dot(eye(4) - dot(K_e, C_m), B), K_e))
        data['estimator_evals'][i] = la.eigvals(data['A_e'][i])
        data['estimator_evals_c'][i] = np.log(data['estimator_evals'][i]) / dt
        # Verify that Kalman estimator eigenvalues are stable
        assert(np.all(abs(data['estimator_evals'][i]) < 1.0))

        # Closed loop state space equations
        A_cl = np.zeros((8, 8))
        A_cl[:4, :4] = A
        A_cl[:4, 4:] = dot(B, K_c)
        A_cl[4:, :4] = dot(K_e, dot(C_m, A))
        A_cl[4:, 4:] = A - A_cl[4:, :4] + A_cl[:4, 4:]
        data['A_cl'][i] = A_cl
        data['closed_loop_evals'][i] = la.eigvals(A_cl)
        assert(np.all(abs(data['closed_loop_evals'][i]) < 1.0))

        B_cl = np.zeros((8, 1))
        B_cl[:4, 0] = B.reshape((4,))
        B_cl[4:, 0] = dot(eye(4) - dot(K_e, C_m), B).reshape((4,))
        data['B_cl'][i] = B_cl

        C_cl = np.hstack((C_z, np.zeros((1, 4))))
        data['C_cl'][i] = C_cl

        # Transfer functions from r to yaw rate
        num, den = ss2tf(A_cl, B_cl, C_cl, 0)
        data['w_r_to_psi_dot'][i], y = freqz(num[0], den)
        data['w_r_to_psi_dot'][i] /= (dt * 2.0 * np.pi)
        data['mag_r_to_psi_dot'][i] = 20.0 * np.log10(abs(y))
        data['phase_r_to_psi_dot'][i] = np.unwrap(np.angle(y)) * 180.0 / np.pi

        # Open loop transfer function from e to yaw rate (PI loop not closed,
        # but LQR/LQG loop closed.
        inner_cl = ss(A_cl, B_cl, C_cl, 0)
        pi_block = ss([[1]], [[1]], [[data['Ki_fit'][i]*dt]], [[data['Kp_fit'][i]]])
        e_to_psi_dot = series(pi_block, inner_cl)
        num, den = ss2tf(e_to_psi_dot.A, e_to_psi_dot.B, e_to_psi_dot.C, e_to_psi_dot.D)
        data['w_e_to_psi_dot'][i], y = freqz(num[0], den)
        data['w_e_to_psi_dot'][i] /= (dt * 2.0 * np.pi)
        data['mag_e_to_psi_dot'][i] = 20.0 * np.log10(abs(y))
        data['phase_e_to_psi_dot'][i] = np.unwrap(np.angle(y)) * 180.0 / np.pi




    return data

def design_controller():

    # Control input weighting
    R = 1.0

    # State weighting matrix Q
    Q = diag([(10*pi/180)**(-2), (10*pi/180)**(-2),
              (10*pi/180)**(-2), (10*pi/180)**(-2)]) / R

    # Always keep this at 1.0 and adjust the scaling of Q
    R = np.array([[1.0]])

    # State error covariance
    W = diag([0.0, 0.0, 1e-6, 1e-6]) * dt

    # Measurement error covariance
    V = .01*diag([(1.0/20000*2.0*pi)**2., # square of 1.0 count
              (0.00227631723111)**2]) # square of std deviation from static

    # Calculate closed loop eigenvalues and gains
    data = compute_gains(Q, np.array([[1.0]]), W, V, dt)

    matlab_data = {'A_cl_w' : data['A_cl'],
                   'B_cl_w' : data['B_cl'],
                   'C_cl_w' : data['C_cl'],
                   'A_w' : data['A'],
                   'B_c_w' : data['B_c'],
                   'C_m_w' : data['C_m'],
                   'C_z_w' : data['C_z'],
                   'K_c_w' : data['K_c'],
                   'A_e_w' : data['A_e'],
                   'B_e_w' : data['B_e']}
    sio.savemat("cl_data.mat", mdict=matlab_data)
    matlab = "/home/hazelnusse/usr/matlab2012b/bin/matlab"
    import os
    os.system(matlab + " -nosplash -nodesktop -nojvm -r 'tune_pi; exit;'")
    pi_gains = sio.loadmat('pi_gains.mat')
    data['Kp'] = pi_gains['Kp_w'].reshape((N,))
    data['Ki'] = pi_gains['Ki_w'].reshape((N,))
    data['Kp_fit'] = smoother(data['theta_R_dot'], data['Kp'])
    data['Ki_fit'] = smoother(data['theta_R_dot'], data['Ki'])

    form_PI_cl(data)

    return data

def smoother(x, y):

    def func(x, a0, a1, a2, a3, a4, a5):
        return a0 + a1 * x + a2 * x**2 + a3 * x**3 + a4 * x**4 + a5 * x**5

    popt, pcov = curve_fit(func, x, y)
    return func(x, *popt)

def form_PI_cl(data):
    A = np.array([[1.0]])
    B = np.array([[1.0]])
    for i in range(N):
        C = np.array([[dt*data['Ki_fit'][i]]])
        D = np.array([[data['Kp_fit'][i]]])
        pi_block = ss(A, B, C, D)
        bike_block = ss(data['A_cl'][i], data['B_cl'][i], data['C_cl'][i], 0)
        pc = series(pi_block, bike_block)
        cl = feedback(pc, 1, sign=-1)

        data['yr_cl_evals'][i] = la.eigvals(cl.A)
        assert(np.all(abs(data['yr_cl_evals'][i]) < 1.0))
        data['A_yr_cl'][i] = cl.A
        data['B_yr_cl'][i] = cl.B
        data['C_yr_cl'][i] = cl.C
        assert(cl.D == 0)

        num, den = ss2tf(cl.A, cl.B, cl.C, cl.D)
        data['w_psi_r_to_psi_dot'][i], y = freqz(num[0], den)
        data['w_psi_r_to_psi_dot'][i] /= (dt * 2.0 * np.pi)
        data['mag_psi_r_to_psi_dot'][i] = 20.0 * np.log10(abs(y))
        data['phase_psi_r_to_psi_dot'][i] = np.unwrap(np.angle(y)) * 180.0 / np.pi

def main():
    data = design_controller()
    np.savez("controller_data.npz", data)

if __name__ == "__main__":
    main()

