"""Data type to manage controller gains matrices."""

from numpy import dtype

controller_t = dtype([
    ('theta_R_dot', 'f8'), # Speed for all matrices/gains
    ('dt', 'f8'),          # Discretization sample period
    # Discrete time bicycle state space matrices.
    # States: roll angle, steer angle, roll rate, steer rate
    # Inputs: lateral rear frame disturbance, lateral front frame distubance,
    #         steer torque
    # Outputs: Steer angle, roll rate, yaw rate
    ('A', '(4,4)f8', 1),
    ('B', '(4,3)f8'),
    ('C', '(3,4)f8'),
    ('plant_evals', '(4,)c16'),
    ('wn_p', '(4,)f8'),
    ('zeta_p', '(4,)f8'),
    ('tau_p', '(4,)f8'),
    # LQR controller closed loop dynamics assuming full state feedback
    # States:  First 4 states are bicycle states, fifth state is integral of
    #          yaw rate error
    # Inputs:  lateral disturbance forces, reference yaw rate
    # Outputs: Yaw rate
    ('A_c', '(5,5)f8'),
    ('B_c', '(5,3)f8'),
    ('C_c', '(1,5)f8'),
    ('F', '(1,5)f8'),      # Optimal state feedback gain (LQR controller output matrix)
    ('controller_evals', '(5,)c16'),
    ('wn_c', '(5,)f8'),    # Natural frequency of controller eigenvalues
    ('zeta_c', '(5,)f8'),  # Damping ratio of controller eigenvalues
    ('tau_c', '(5,)f8'),   # Time constant of controller eigenvalues
    # Kalman filter
    # States: Estimates of 4 bicycle states
    # Inputs: steer torque, steer angle measurement, roll rate measurement
    ('K_c', '(4,2)f8'),    # Optimal filter gain (Kalman filter input matrix)
    ('A_e', '(4,4)f8'),    # (I - K_c * C) * A, also error dynamics matrix
    ('B_e', '(4,3)f8'),    # [I - K_c * C) * B_delta, K_c]
    ('estimator_evals', '(4,)c16'),
    ('wn_e', '(4,)f8'),    # Natural frequency of estimator eigenvalues
    ('zeta_e', '(4,)f8'),  # Damping ratio of estimator eigenvalues
    ('tau_e', '(4,)f8'),   # Time constant of estimator eigenvalues
    # Combined controller and observer
    # States: Estimates of 4 bicycle states, integral of yaw rate error
    # Inputs: yaw rate reference, steer angle measurement, roll rate measurement
    # Outputs: steer torque
    ('A_ce', '(5,5)f8'),    # LQR/Kalman controller dynamics matrix
    ('B_ce', '(5,3)f8'),    # LQR/Kalman controller input matrix
    ('controller_estimator_evals', '(5,)c16'),
    ('wn_ce', '(5,)f8'),    # Natural frequency of controller eigenvalues
    ('zeta_ce', '(5,)f8'),  # Damping ratio of controller eigenvalues
    ('tau_ce', '(5,)f8'),   # Time constant of controller eigenvalues

    # Closed loop system
    # States: four bicycle states, four bicycle state estimates, integral of
    #         yaw rate error
    # Inputs: lateral disturbances (2), process noise (4), yaw rate reference (1)
    #         measurement noise (2)
    ('A_cl', '(9,9)f8'),   # Closed loop A matrix
    ('B_cl', '(9,9)f8'),   # Closed loop B matrix
    ('C_cl', '(1,9)f8'),   # Closed loop output (yaw rate)
    ('closed_loop_evals', '(9,)c16'),
    ('wn_cl', '(9,)f8'),   # Natural frequency of closed loop eigenvalues
    ('zeta_cl', '(9,)f8'), # Damping ratio of closed loop eigenvalues
    ('tau_cl', '(9,)f8'),  # Time constant of closed loop eigenvalues
    ('w', '(100,)f8'),     # Frequencies (Hz)
    ('mag_cl', '(100,)f8'),   # Magnitude (dB)
    ('phase_cl', '(100,)f8')  # Phase (deg)
    ])

