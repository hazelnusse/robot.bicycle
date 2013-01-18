import numpy as np

colors = ['r', 'g', 'b', 'y', 'c']

def compute_d_f(alpha_a):
    """Solve linear least squares problem for center of mass location.

    Set linear least squares problem up as:
    A*[d; f] = a
    where Ai = [cos(alphai), sin(alphai)]

    Parameters
    ----------
    alpha_a :   numpy.array of shape (N, 2), orientation angle in first column,
                offset in second column

    Returns
    -------
    Least squares estimate of mass center location

    """
    A = np.array([np.cos(alpha_a[:, 0]), np.sin(alpha_a[:, 0])])
    return np.linalg.lstsq(A.T, alpha_a[:, 1])

def compute_Ixx_Ixz_Izz(alpha, T, Ir, Tr):
    """Given a set of angles and oscillation periods, along with the inertia and
    oscillation period for a calibration rod on the same torsional spring,
    compute the least squares estimate of x-z inertia scalars.
    """

    A = np.array([np.sin(alpha)**2.0, -2.0*np.sin(alpha)*np.cos(alpha),
        np.cos(alpha)**2.0]).transpose()
    b = Ir*(np.array(T)/Tr)**2.0
    return np.linalg.lstsq(A, b)

def decaying_sinusoid(t, a, zeta, T, d, e):
    return a*np.exp(-2.0*np.pi/T*zeta*t)*np.sin(2.0*np.pi/T*np.sqrt(1-zeta**2.0)*t + d) + e

def GyrostatParameters(mA, mB, r_BO_AO, I_A_AO, I_B_BO):
    d_, e_, f_ = r_BO_AO
    IAxx, IAyy, IAzz, IAxy, IAyz, IAxz = I_A_AO
    IBxx, IByy, IBzz, IBxy, IByz, IBxz = I_B_BO

    # Generated using sympy.physics.mechanics
    mT = mA + mB
    beta = mA/mT
    d = beta*d_
    e = beta*e_
    f = beta*f_
    gamma = beta*mB
    IGxx = IAxx + IBxx + gamma*(e_**2 + f_**2)
    IGyy = IAyy + IByy + gamma*(d_**2 + f_**2)
    IGzz = IAzz + IBzz + gamma*(d_**2 + e_**2)
    IGxy = IAxy + IBxy - d_*e_*gamma
    IGyz = IAyz + IByz - e_*f_*gamma
    IGxz = IAxz + IBxz - d_*f_*gamma

    r_BO_GO = [d, e, f]
    I_G_GO = [IGxx, IGyy, IGzz, IGxy, IGyz, IGxz]

    return mT, r_BO_GO, I_G_GO

