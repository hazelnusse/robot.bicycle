"""Full state feedback LQR yaw rate controller with integral action.

"""
from parameters import rear, w
import scipy.io as sio
from scipy.signal import cont2discrete
import numpy as np
from numpy import pi, dot
import numpy.linalg as la
np.set_printoptions(precision=4)
import matplotlib.pyplot as plt
from control_tools import dare

data = np.load('robotic_bicycle_linear_dynamics_vs_logspeed.npz')
radius = rear.R
#data = np.load('benchmark_bicycle_linear_dynamics_vs_logspeed.npz')
#radius = w.rR

theta_R_dot = data['theta_R_dot']
A_w = data['A_w']
B_w = data['B_w']
C_w = data['C_w']   # steer, roll rate, yaw rate measurements
N = len(theta_R_dot)

def plot_evals():
    evals = np.zeros((N, 4), dtype=np.complex128)
    for i, A in enumerate(A_w):
        evals[i], _ = la.eig(A)

    plt.figure()
    plt.plot(-theta_R_dot*radius, evals.real, 'k.')
    plt.plot(-theta_R_dot*radius, evals.imag, 'b.')

def compute_gains(Q, R, W, V, dt):
    # Allocate arrays to store discrete state space matrices
    A_w_d = np.zeros((N, 4, 4))
    B_w_d = np.zeros((N, 4, 3))
    eye4 = np.identity(4)          # Full state feedback
    eye5 = np.identity(5)          # Full state feedback
    z41 = np.zeros((4, 1))

    # Allocate space for feedback control gain
    F_w = np.zeros((N, 5))
    evals_controller_w = np.zeros((N, 5), dtype=np.complex128)

    # Allocate space for filter gain and filter eigenvalues
    K_w = np.zeros((N, 2, 4))
    evals_observer_w = np.zeros((N, 4), dtype=np.complex128)

    # Loop over all speeds for which we have system dynamics
    for i in range(N):
        # Convert the bike dynamics to discrete time using a zero order hold
        A_w_d[i], B_w_d[i], _, _, _ = cont2discrete((A_w[i],
                                        B_w[i, :, 2].reshape((4, 1)),
                                        eye4, z41), dt)
        # Augment the state matrices
        Aa = np.zeros((5, 5))
        Aa[:4, :4] = A_w_d[i]
        Aa[4, :4] = -C_w[i, 2, :] * dt
        Aa[4, 4] = 1

        Ba = np.zeros((5, 1))
        Ba[:4] = np.array([B_w_d[i, :, 2]]).T

        # Get the closed loop eigenvalues and optimal gain as determined by
        # solving the discrete time algebraic Riccati equation
        _, evals_controller_w[i], F_w[i] = dare(Aa, Ba, eye5, Q, R)


        _, evals_observer_w[i], K_w[i] = dare(A_w_d[i].T, C_w[i, :2, :].T,
                eye4, W, V)

    return evals_controller_w, F_w, evals_observer_w, K_w
    
if __name__ == "__main__":
    plot_evals()

    dt = 0.005
    # State weighting matrix Q
    Q = np.diag([(10*pi/180)**(-2), (10*pi/180)**(-2),
                 (100*pi/180)**(-2), (100*pi/180)**(-2),
                 (10*pi/180)**(-2)])

    # Control input weighting matrix R
    R = np.array([[.1]])

    # State error covariance
    W = np.diag([0.0, 0.0, .01, .01])

    # Measurement error covariance
    V = np.diag([(10.0/20000*2.0*pi)**2.,
                 (100*4000*pi/180/(2**16))**2])
    # Calculate closed loop eigenvalues and gains
    evals_controller_w, F_w, evals_observer_w, K_w = compute_gains(Q, R, W, V, dt)

    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, F_w)
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    ax[4].set_label(r"$k_\i$")
    plt.legend(loc=0)
    
    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, K_w[:, 0, :])
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    plt.title("Observer steer angle gains")
    plt.legend(loc=0)
    
    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, K_w[:, 1, :])
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    plt.legend(loc=0)
    plt.title("Observer roll rate gains")
    print(evals_observer_w)

    plt.show()
