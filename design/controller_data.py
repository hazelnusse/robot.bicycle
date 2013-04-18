"""Data type to manage controller gains matrices."""

from numpy import dtype

controller_t = dtype([
    ('theta_R_dot', 'f8'), # Speed for all matrices/gains
    ('dt', 'f8'),          # Time discretization period
    # Discrete time bicycle state space matrices.
    # States: roll angle, steer angle, roll rate, steer rate
    # Inputs: lateral rear frame disturbance, lateral front frame distubance,
    #         steer torque
    # Outputs: Steer angle, roll rate, yaw rate
    ('A', '(4,4)f8'),
    ('B', '(4,3)f8'),
    ('C_m', '(2,4)f8'),
    ('C_z', '(1,4)f8'),
    ('plant_evals', '(4,)c16'),
    ('plant_evals_c', '(4,)c16'),
    # Plant controllability and observability matrices
    ('ctrb_plant', '(4,4)f8'),  # steer torque input
    ('obsv_plant', '(8,4)f8'),  # steer angle and roll rate measured
    # LQR controller closed loop dynamics assuming full bicycle state feedback
    # States:  4 states are bicycle states
    # Inputs:  additive steer torque (added to feedback law: u = K_c * x + r)
    # Outputs: Yaw rate
    ('A_c', '(4,4)f8'),
    ('B_c', '(4,1)f8'),
    # Optimal state feedback gain u = K_c*x
    ('K_c', '(1,4)f8'),
    ('controller_evals', '(4,)c16'),
    # Kalman filter
    # States: Estimates of 4 bicycle states
    # Inputs: steer torque, steer angle measurement, roll rate measurement
    ('K_e', '(4,2)f8'),    # Optimal filter gain (Kalman filter input matrix)
    ('A_e', '(4,4)f8'),    # (I - K_e * C) * A, also error dynamics matrix
    ('B_e', '(4,3)f8'),    # [I - K_e * C) * B_delta, K_e]
    ('estimator_evals', '(4,)c16'),
    # Closed loop system (for simulation)
    # States: four bicycle states, four estimates
    # Inputs: additive steer torque (added to feedback law: u = K_c * \hat{x} + r)
    ('A_cl', '(8,8)f8'),  # Closed loop A matrix
    ('B_cl', '(8,1)f8'),  # Closed loop B matrix
    ('C_cl', '(1,8)f8'),  # Closed loop output (yaw rate)
    ('closed_loop_evals', '(8,)c16'),
    ('w_r_to_psi_dot', '(512,)f8'),     # Frequencies (Hz)
    ('mag_r_to_psi_dot', '(512,)f8'),   # Magnitude (dB)
    ('phase_r_to_psi_dot', '(512,)f8'), # Phase (deg)
    ('Kp', 'f8'),
    ('Ki', 'f8'),
    ('Kp_fit', 'f8'),
    ('Ki_fit', 'f8'),
    ('A_yr_cl', '(9,9)f8'),
    ('B_yr_cl', '(9,1)f8'),
    ('C_yr_cl', '(1,9)f8'),
    ('yr_cl_evals', '(9,)c16')
    ])

