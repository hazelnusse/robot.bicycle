from parameters import benchmark_bicycle, robotic_bicycle, rear, w
import numpy as np
import numpy.linalg as la

def calc_eigenvalues(theta_R_dot, bike, filename):
    N = len(theta_R_dot)

    # state derivative coefficient matrix. Does not vary with speed
    mm = np.zeros((bike.n + bike.o, bike.n + bike.o))
    #bike.mass_matrix_full(mm)

    # State coefficient matrix.  Does vary with speed
    aa = np.zeros((bike.n + bike.o, bike.n - bike.l + bike.o - bike.m))

    # Input coefficient matrix associated with dynamic equations
    B_u = np.zeros((bike.n + bike.m, bike.s))
    B_l = np.zeros((bike.o - bike.m, bike.s))

    # A, B, C matrices parameterized by speed
    A_w = np.zeros((N, 4, 4))
    B_w = np.zeros((N, 4, 3))
    C_w = np.zeros((N, 3, 4))

    # Dynamic equations of interest (roll and steer second time derivatives)
    rows = [9, 11]
    # Independent state columns of interest
    cols = [1, 2, 7, 8]
    for i, wi in enumerate(theta_R_dot):
        bike.set_state(12, wi)                          # set rear wheel rate
        bike.solve_velocity_constraints_and_set_state() # determine dependent rates
        bike.mass_matrix_full(mm)                       # full mass matrix
        bike.independent_state_matrix(aa)               # compute state coef matrix
        bike.input_matrix(B_l)                          # input coefficient matrix

        A = la.solve(mm[:14, :14], aa[:14, :10])      # solve assuming no slip

        # Upper right hand corner of A matrix
        A_w[i, 0, 2] = A_w[i, 1, 3] = 1
        # Populate bottom rows of A matrix with appropriate rows & cols of full
        # state matrix
        A_w[i, 2, :] = A[rows[0], cols]
        A_w[i, 3, :] = A[rows[1], cols]

        # Columns associated with lateral frame and fork disturbances, and steer torque
        bb = np.vstack((B_u, B_l))
        bb = la.solve(mm[:14, :14], bb[:14, :])
        bb = np.vstack((bb[:, 8], bb[:, 18], bb[:, 20])).T
        B_w[i] = np.zeros((4, 3))
        B_w[i, 2, :] = bb[9, :]
        B_w[i, 3, :] = bb[11, :]
        #print(B_w[i])
        
        # Yaw kinematic equation
        C_w[i] = np.array([[0.0, 1.0, 0.0, 0.0],      # steer measurement
                           [0.0, 0.0, 1.0, 0.0],      # roll rate measurement
                           A[0, cols]])               # yaw rate measurement
        print(C_w[i])

        #print(i)

    # Save the data
    np.savez(filename, theta_R_dot=theta_R_dot, A_w=A_w, B_w=B_w, C_w=C_w)

if __name__ == '__main__':
    N = 11                           # Number of speeds
    lowest_speed = 0.5               # Lowest speed for calculations
    highest_speed = 10.0             # Highest speed for calculations
    speeds = np.logspace(np.log10(lowest_speed), np.log10(highest_speed), N)
    theta_R_dot = sorted(-speeds / (rear.R + rear.r))
    calc_eigenvalues(theta_R_dot, benchmark_bicycle,
            'benchmark_bicycle_linear_dynamics_vs_logspeed.npz')
    calc_eigenvalues(theta_R_dot, robotic_bicycle,
            'robotic_bicycle_linear_dynamics_vs_logspeed.npz')
    theta_R_dot = sorted(-np.linspace(0, 10, N) / w.rR)
    calc_eigenvalues(theta_R_dot, benchmark_bicycle,
            'benchmark_bicycle_linear_dynamics_vs_speed.npz')
    calc_eigenvalues(theta_R_dot, robotic_bicycle,
            'robotic_bicycle_linear_dynamics_vs_speed.npz')

