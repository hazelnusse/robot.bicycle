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

def compute_gains(Q, R, dt):
    # Allocate arrays to store discrete state space matrices
    A_w_d = np.zeros((N, 4, 4))
    B_w_d = np.zeros((N, 4, 3))
    eye4 = np.identity(4)          # Full state feedback
    z41 = np.zeros((4, 1))

    # Allocate space for feedback control gain
    G_w = np.zeros((N, 5))

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
        
        # Hamiltonian matrix
        H = np.zeros((10, 10))      # From page 349 of Antsaklis
        Aa_inv_T = la.inv(Aa).T
        H[:5, :5] = Aa + dot(dot(Ba, Ba.T), dot(Aa_inv_T, Q)) / R
        H[:5, 5:] = -dot(dot(Ba, Ba.T), Aa_inv_T) / R
        H[5:, :5] = -dot(Aa_inv_T, Q)
        H[5:, 5:] = Aa_inv_T
        evals, evecs = la.eig(H)
        stable_indices = []
        for j in range(10):
            if abs(evals[j]) < 1:       # inside unit circle
                stable_indices.append(j)

        V1 = evecs[:5, stable_indices]
        V2 = evecs[5:, stable_indices]

        X = dot(V2, la.inv(V1))
        G_w[i] = dot(dot(la.inv((R + dot(dot(Ba.T, X), Ba))), Ba.T), dot(X, Aa)).real

    return G_w
    
if __name__ == "__main__":
    plot_evals()

    dt = 0.005
    # State weighting matrix Q
    Q = np.diag([(10*pi/180)**(-2), (10*pi/180)**(-2),
                 (100*pi/180)**(-2), (100*pi/180)**(-2),
                 (10*pi/180)**(-2)])

    # Control input weighting matrix R
    R = .1

    # Calculate gains
    G_w = compute_gains(Q, R, dt)

    plt.figure()
    ax = plt.plot(-theta_R_dot*radius, G_w)
    ax[0].set_label(r"$k_\phi$")
    ax[1].set_label(r"$k_\delta$")
    ax[2].set_label(r"$k_\dot{\phi}$")
    ax[3].set_label(r"$k_\dot{\delta}$")
    ax[4].set_label(r"$k_\i$")
    plt.legend(loc=0)

    plt.show()
