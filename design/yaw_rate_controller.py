"""Full state feedback LQR yaw rate controller with integral action.

"""
from parameters import rear, w
import scipy.io as sio
from scipy.signal import cont2discrete, bode
import numpy as np
from numpy import pi
from numpy.matlib import zeros, empty, eye, asmatrix, diag
import numpy.linalg as la
np.set_printoptions(precision=4)
import matplotlib.pyplot as plt
from control_tools import dare


data = np.load('robotic_bicycle_linear_dynamics_vs_logspeed.npz')
radius = rear.R
#data = np.load('benchmark_bicycle_linear_dynamics_vs_logspeed.npz')
#radius = w.rR

skip = 10
theta_R_dot = data['theta_R_dot'][::skip]
A_w = data['A_w'][::skip]
B_w = data['B_w'][::skip]
C_w = data['C_w'][::skip]   # steer, roll rate, yaw rate measurements
N = len(theta_R_dot)
print(N)

def plot_evals():
    evals = np.zeros((N, 4), dtype=np.complex128)
    for i, A in enumerate(A_w):
        evals[i], _ = la.eig(A)

    plt.figure()
    plt.plot(-theta_R_dot*radius, evals.real, 'k.')
    plt.plot(-theta_R_dot*radius, evals.imag, 'b.')

def compute_gains(Q, R, W, V, dt):
    # Allocate arrays to store discrete state space matrices
    A_w_d = np.empty((N, 4, 4))
    B_w_d = np.empty((N, 4, 3))
    Aa_w_d = np.zeros((N, 5, 5))
    Ba_w_d = np.zeros((N, 5, 1))

    # Allocate space for feedback control gain
    F_w = np.empty((N, 1, 5))
    evals_controller_w = np.empty((N, 5), dtype=np.complex128)

    # Allocate space for filter gain and filter eigenvalues
    K_c_w = np.empty((N, 4, 3))
    evals_estimator_w = np.empty((N, 4), dtype=np.complex128)

    # Loop over all speeds for which we have system dynamics
    for i in range(N):
        # Convert the bike dynamics to discrete time using a zero order hold
        A_w_d[i], B_w_d[i], _, _, _ = cont2discrete((A_w[i],
                                        B_w[i, :, 2].reshape((4, 1)),
                                        eye(4), zeros((4, 1))), dt)
        # Augment the state matrices
        Aa_w_d[i, :4, :4] = A_w_d[i]
        Aa_w_d[i, 4, :4] = -C_w[i, 2, :] * dt
        Aa_w_d[i, 4, 4] = 1

        Ba_w_d[i, :4] = np.array([B_w_d[i, :, 2]]).T

        # Get the closed loop eigenvalues and optimal gain as determined by
        # solving the discrete algebraic Riccati equation
        Aa = asmatrix(Aa_w_d[i])
        Ba = asmatrix(Ba_w_d[i])
        P_c = dare(Aa, Ba, R, Q)
        F_w[i] = -la.inv(R + Ba.T*P_c*Ba)*Ba.T*P_c*Aa
        evals_controller_w[i] = la.eigvals(Aa + Ba*F_w[i])

        C = asmatrix(C_w[i])
        P_e = dare(A_w_d[i].T, C.T, V, W)
        K_c_w[i] = P_e*C.T*la.inv(C*P_e*C.T + V)
        evals_estimator_w[i] = la.eigvals((eye(4) - K_c_w[i]*C)*A_w_d[i])

    return evals_controller_w, F_w, evals_estimator_w, K_c_w, Aa_w_d, Ba_w_d

if __name__ == "__main__":
    #plot_evals()

    dt = 0.005
    # State weighting matrix Q
    Q = diag([(10*pi/180)**(-2), (10*pi/180)**(-2),
                 (100*pi/180)**(-2), (100*pi/180)**(-2),
                 (10*pi/180)**(-2)])

    # Control input weighting matrix R
    R = diag([.1])

    # State error covariance
    W = diag([0.0, 0.0, .001, .001]) * dt

    # Measurement error covariance
    V = diag([(10.0/20000*2.0*pi)**2.,
              (100/(2**16)*500*pi/180)**2,
              (100/(2**16)*500*pi/180)**2]) / dt

    # Calculate closed loop eigenvalues and gains
    evals_controller_w, F_w, evals_estimator_w, K_c_w, Aa_w_d, Ba_w_d = compute_gains(Q, R, W, V, dt)

    # State feedback gains versus speed
    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, F_w.reshape((N, 5)))
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    ax[4].set_label(r"$k_\i$")
    plt.legend(loc=0)
    plt.title('Feedback gains vs. speed')
    plt.xlabel('Speed [m / s]')
    plt.ylabel('Gain')

    plt.figure()
    ax = plt.plot(evals_controller_w.real, evals_controller_w.imag, 'k.')
    plt.title('Closed loop controller eigenvalues')
    plt.xlabel('Imaginary')
    plt.ylabel('Real')
    plt.axis((-1, 1, -1, 1))

    # Estimator gains versus speed (first column, associated with steer angle
    # measurement)
    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, K_c_w[:, :, 0])
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    plt.title("Observer steer angle gains")
    plt.legend(loc=0)

    # Estimator gains versus speed (second column, associated with roll rate
    # measurement)
    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, K_c_w[:, :, 1])
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    plt.legend(loc=0)
    plt.title("Observer roll rate gains")
    
    # Estimator gains versus speed (third column, associated with yaw rate
    # measurement)
    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, K_c_w[:, :, 2])
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    plt.legend(loc=0)
    plt.title("Observer yaw rate gains")
    
    plt.figure()
    ax = plt.plot(evals_estimator_w.real, evals_estimator_w.imag, 'k.')
    plt.title('Closed loop estimator eigenvalues')
    plt.xlabel('Imaginary')
    plt.ylabel('Real')
    plt.axis((-1, 1, -1, 1))

    plt.show()
