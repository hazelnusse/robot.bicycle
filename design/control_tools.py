"""Helper tools for control system design.

"""

from scipy.linalg import inv, schur, solve
from numpy.matlib import empty
import numpy.linalg as la
import numpy as np
from control import obsv, ctrb

def dare(F, G1, G2, H):
    """Solves the discrete-time algebraic Riccati equation

    0 = F ^ T * X * F
        - X - F ^ T * X * G1 * (G2 + G1 ^ T * X * G1) ^ -1 * G1 ^ T * X * F + H

    Under the assumption that X ^ -1 exists, this equation is equivalent to

    0 = F ^ T * (X ^ -1 + G1 * G2 ^ -1 * G1 ^ T) ^ -1 * F - X + H

    Parameters
    ==========
    Inputs are real matrices:

    F : n x n
    G1 : n x m
    G2 : m x m, symmetric, positive definite
    H : n x n, symmetric, positive semi-definite

    Assumptions
    ===========
    (F, G1) is a stabilizable pair
    (C, F) is a detectable pair (where C is full rank factorization of H, i.e.,
        C ^ T * C = H and rank(C) = rank(H).
    F is invertible

    Returns
    =======

    Unique nonnegative definite solution of discrete Algrebraic Ricatti
    equation.

    Notes
    =====
    This is an implementation of the Schur method for solving algebraic Riccati
    eqautions as described in dx.doi.org/10.1109/TAC.1979.1102178

    """
    # Verify that F is non-singular
    u, s, v = la.svd(F)
    assert(np.all(s > 0.0))
    # Verify that (F, G1) controllable
    C = ctrb(F, G1)
    u, s, v = la.svd(C)
    assert(np.all(s > 0.0))
    # Verify that (H**.5, F) is observable
    O = obsv(H**.5, F)
    u, s, v = la.svd(O)
    assert(np.all(s > 0.0))
    
    n = F.shape[0]
    m = G2.shape[0]

    G = G1*inv(G2)*G1.T
    Finv = inv(F)
    Finvt = Finv.T

    # Form symplectic matrix
    Z = empty((2*n, 2*n))
    Z[:n, :n] = F + G*Finvt*H
    Z[:n, n:] = -G*Finvt
    Z[n:, :n] = -Finvt*H
    Z[n:, n:] = Finvt

    S, U, sdim = schur(Z, sort='iuc')

    # Verify that the n eigenvalues of the upper left block stable
    assert(sdim == n)

    U11 = U[:n, :n]
    U21 = U[n:, :n]
    return solve(U[:n, :n].T, U[n:, :n].T).T
