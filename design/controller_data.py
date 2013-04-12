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
    ('C', '(3,4)f8'),
    ('plant_evals', '(4,)c16'),
    ('plant_evals_c', '(4,)c16'),
    # Plant controllability and observability matrices
    ('ctrb_plant', '(4,4)f8'),  # steer torque input
    ('obsv_plant', '(8,4)f8'),  # steer angle and roll rate measured
    # LQI controller closed loop dynamics assuming full bicycle state feedback
    # States:  First 4 states are bicycle states, fifth state is integral of
    #          yaw rate error
    # Inputs:  lateral disturbance forces, reference yaw rate
    # Outputs: Yaw rate
    ('A_cp', '(5,5)f8'),
    ('B_cp', '(5,1)f8'),
    ('C_cp', '(1,5)f8'),
    # Optimal state feedback gain u = K_c*x
    ('K_c', '(1,5)f8'),
    ('controller_evals', '(5,)c16'),
    ('controller_evals_c', '(5,)c16'),
    # Kalman filter
    # States: Estimates of 4 bicycle states
    # Inputs: steer torque, steer angle measurement, roll rate measurement
    ('K_e', '(5,3)f8'),    # Optimal filter gain (Kalman filter input matrix)
    ('A_ep', '(5,5)f8'),    # (I - K_c * C) * A, also error dynamics matrix
    ('B_ep', '(5,5)f8'),    # [I - K_c * C) * B_delta, K_c]
    ('C_ep', '(3,5)f8'),    # steer, roll rate, integral of yaw rate error
    ('estimator_evals', '(5,)c16'),
    ('estimator_evals_c', '(5,)c16'),
    # Combined controller and observer
    # States: Estimates of 4 bicycle states, integral of yaw rate error
    # Inputs: yaw rate reference, steer angle measurement, roll rate measurement
    # Outputs: steer torque
    ('A_ce', '(5,5)f8'),    # LQR/Kalman controller dynamics matrix
    ('B_ce', '(5,3)f8'),    # LQR/Kalman controller input matrix
    # Closed loop system
    # States: four bicycle states, integral of yaw error, estimates 
    # Inputs: yaw rate reference (1)
    ('A_cl', '(10,10)f8'),   # Closed loop A matrix
    ('B_cl', '(10,1)f8'),   # Closed loop B matrix
    ('C_cl', '(1,10)f8'),   # Closed loop output (yaw rate)
    ('closed_loop_evals', '(10,)c16'),
    ('closed_loop_evals_c', '(10,)c16'),
    ('w_cl', '(512,)f8'),     # Frequencies (Hz)
    ('mag_cl', '(512,)f8'),   # Magnitude (dB)
    ('phase_cl', '(512,)f8'), # Phase (deg)
    ('w_ol', '(512,)f8'),     # Frequencies (Hz)
    ('mag_ol', '(512,)f8'),   # Magnitude (dB)
    ('phase_ol', '(512,)f8')  # Phase (deg)
    ])

