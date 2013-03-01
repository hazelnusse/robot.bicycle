"""Helper tools for control system design.

"""

from scipy.linalg import solve, inv, eig, eigvals
from numpy import dot, zeros

def dare(A, B, M, Q, R):
    """Solves the discrete-time algebraic Riccati equation

    Parameters
    ==========

    A : System dynamics matrix (n x n)
    B : Input matrix           (n x m)
    Q : State weighting matrix (n x n), symmetric, positive definite
    R : Input weighting matrix (m x m), symmetric, positive definite

    Returns
    =======

    P_c : solution to Riccati equation
      L : closed loop eigenvalues
      F : optimal state feedback gain u(k) = F * x(k), note sign!

    Notes
    =====

    For the discrete time optimal control problem, we have the system

      x(k + 1) = A * x(k) + B * u(k)
          z(k) = M * x(k)

    The Riccati equation is

      A ^ T * (P_c - P_c * B * (R + B ^ T * P_c * B) ^ (-1) * B ^ T * P_c) * A - P_c + M ^ T * Q * M = 0

    The solution to this equation, P_c, yields the linear state feedback law

      u(k) = F * x(k) = -(R + B ^ T * P_c * B) ^ (-1) B ^ T * P_c * A * x(k)

    which minimizes the cost function

      J(u) = \sum_{k = 0}^{\infty} (z ^ T (k) * Q * z(k) + u ^ T (k) * R * u(k))

    Note that the discrete time optimal control problem is dual to the discrete
    time optimal state estimation problem:

      x(k + 1) = A * x(k) + B * u(k) + G * w(k)
          y(k) = C * x(k) + v(k)

    where w and k are white, zero-mean Gaussian processes with covariances:

      E[w * w ^ T] = W

      E[v * v ^ T] = V

    With W = W ^ T > 0, V = V ^ T > 0.

    The optimal current estimator is

      \bar{x} (k) = \hat{x}(k) + K_c * (y(k) - C * \hat{x}(k))

    where

      \hat{x} = A * \bar{x}(k - 1) + B * u(k - 1)

    where \hat{x} denotes the prior estimate of the state at the time of a
    measurement.  The state error covariance is minimized when the filter gain
    is

      K_c = P_e * C ^ T * (C * P_e * C ^ T + V) ^ (-1)

    where P_e is the solution to the dual of the above Riccati equation
    obtained by substituting:

        A => A ^ T
        B => C ^ T
        M => G ^ T
        R => V
        Q => W

    """

    An, Am = A.shape
    Bn, Bm = B.shape
    Qn, Qm = Q.shape
    Rn, Rm = R.shape
    assert(An == Am == Bn == Qn == Qm)
    assert(Bm == Rn == Rm)

    n = An
    m = Bm

    # Allocate the Hamiltonian matrix
    H = zeros((2*n, 2*n))

    # Solve for B * R ^ -1
    B_R_inv = solve(R.T, B.T).T

    # Solve for  A ^ -T * M ^ T * Q * M
    A_nT_M_T_Q_M = solve(A.T, dot(M.T, dot(Q, M)))

    # Solve for B ^ T * A ^ -T
    B_T_A_nT = solve(A, B).T

    H[:n, :n] = A + dot(dot(B_R_inv, B.T), A_nT_M_T_Q_M)
    H[:n, n:] = -dot(B_R_inv, B_T_A_nT)
    H[n:, :n] = -A_nT_M_T_Q_M
    H[n:, n:] = inv(A).T

    evals, evecs = eig(H)
    stable_indices = []
    num_stable = 0
    for j in range(2*n):
        if abs(evals[j]) < 1:       # inside unit circle
            stable_indices.append(j)
            num_stable += 1
    assert(num_stable == n)         # Verify construction of Hamiltonian

    P_c = solve(evecs[:n, stable_indices].T, evecs[n:, stable_indices].T)
    F = -solve(R + dot(B.T, dot(P_c, B)), dot(B.T, dot(P_c, A))).real

    return P_c, eigvals(A + dot(B, F)), F

