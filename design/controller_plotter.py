import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import dstep, dlsim
from parameters import rear, w
import yaw_rate_controller as yrc

class CDataPlotter(object):

    def __init__(self, datafile):
        self.d = np.load(datafile)['arr_0']

    def plant_evals(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['plant_evals'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['plant_evals'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Plant eigenvalues (discrete)")

    def plant_evals_c(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['plant_evals_c'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['plant_evals_c'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Plant eigenvalues (continuous)")

    def plant_damp(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['wn_p'], 'k')
        plt.plot(-self.d['theta_R_dot'], self.d['zeta_p'], 'b')
        plt.plot(-self.d['theta_R_dot'], self.d['tau_p'], 'g')
        plt.xlabel('$\\dot{\\theta}_R$')
    
    def controller_evals(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['controller_evals'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['controller_evals'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Controller eigenvalues (discrete)")

    def controller_evals_c(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['controller_evals_c'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['controller_evals_c'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Controller eigenvalues (continuous)")

    def controller_gains(self):
        plt.figure()
        N = len(self.d)
        ax = plt.plot(-self.d['theta_R_dot'], self.d['F'][:,:,:].reshape((N, 5)))
        ax[0].set_label(r"$k_\phi$")
        ax[1].set_label(r"$k_\delta$")
        ax[2].set_label(r"$k_\dot{\phi}$")
        ax[3].set_label(r"$k_\dot{\delta}$")
        ax[4].set_label(r"$k_\i$")
        plt.legend(loc=0)
        plt.title('Feedback gains vs. speed')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.ylabel('Gain')
        
    def estimator_evals(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['estimator_evals'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['estimator_evals'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Estimator eigenvalues (discrete)")

    def estimator_evals_c(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['estimator_evals_c'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['estimator_evals_c'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Estimator eigenvalues (continuous)")

    def estimator_gains(self):
        N = len(self.d)
        f, axarr = plt.subplots(2, 1, sharex=True)
        lines = axarr[0].plot(-self.d['theta_R_dot'], self.d['K_c'][:,:,0].reshape((N, 4)))
        lines[0].set_label(r"$k_\phi$")
        lines[1].set_label(r"$k_\delta$")
        lines[2].set_label(r"$k_\dot{\phi}$")
        lines[3].set_label(r"$k_\dot{\delta}$")
        axarr[0].legend(loc=0)
        axarr[0].set_title('Estimator gains vs. speed')
        axarr[0].set_ylabel('Steer measurement gain')
        
        lines = axarr[1].plot(-self.d['theta_R_dot'], self.d['K_c'][:,:,1].reshape((N, 4)))
        lines[0].set_label(r"$k_\phi$")
        lines[1].set_label(r"$k_\delta$")
        lines[2].set_label(r"$k_\dot{\phi}$")
        lines[3].set_label(r"$k_\dot{\delta}$")
        axarr[1].legend(loc=0)
        axarr[1].set_xlabel('$\\dot{\\theta}_R$')
        axarr[1].set_ylabel('Roll rate measurement gain')
    
    def controller_estimator_evals(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['controller_estimator_evals'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['controller_estimator_evals'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Controller Estimator eigenvalues (discrete)")

    def controller_estimator_evals_c(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['controller_estimator_evals_c'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['controller_estimator_evals_c'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Controller Estimator eigenvalues (continuous)")

    def closed_loop_evals(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['closed_loop_evals'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['closed_loop_evals'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Closed loop eigenvalues (discrete)")

    def closed_loop_evals_c(self):
        plt.figure()
        plt.plot(-self.d['theta_R_dot'], self.d['closed_loop_evals_c'].real, 'k.')
        plt.plot(-self.d['theta_R_dot'], self.d['closed_loop_evals_c'].imag, 'b.')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.title("Closed loop eigenvalues (continuous)")

    def closed_loop_bode(self, speed):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].semilogx(self.d['w_cl'][i], self.d['mag_cl'][i])
        axarr[0].set_title('Closed loop tf, $\\dot{\\theta}_R$'
                        + ' = {0}'.format(-self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_cl'][i], self.d['phase_cl'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
    
    def noise_to_torque_bode(self, speed):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].semilogx(self.d['w_n_to_u'][i], self.d['mag_n_to_u'][i])
        axarr[0].set_title('Noise to torque bode, $\\dot{\\theta}_R$'
                        + ' = {0}'.format(-self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_n_to_u'][i], self.d['phase_n_to_u'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")

    def closed_loop_step(self, speed):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        C_yr = self.d['C_cl'][i]
        C_u = np.zeros((1, 9))
        C_u[:, 4:] = self.d['F'][i]
        C_x = np.hstack((np.eye(4), np.zeros((4,5))))
        C_xe = np.hstack((np.zeros((4,4)), np.eye(4), np.zeros((4, 1))))

        C = np.vstack((C_yr, C_u, C_x, C_xe))
        D = np.zeros((C.shape[0], 1))
        x0 = np.zeros((9,))
        # Start the bicycle with non-zero initial conditions to see how
        # quickly estimator converges given the initial condition mismatch
        x0[0] = 10.0 * np.pi/180.0  # Initial roll
        x0[1] = 10.0 * np.pi/180.0  # Initial steer
        x0[2] = 10.0 * np.pi/180.0  # Initial roll rate
        x0[3] = 10.0 * np.pi/180.0  # Initial steer rate
        t, x = dstep((self.d['A_cl'][i], self.d['B_cl'][i, :, 6].reshape((9,1)),
                      C, D, self.d['dt'][i]),
                      x0=x0,
                      t=np.linspace(0, 20, 100))

        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].plot(t, x[0][:, 0])
        axarr[0].set_title('Closed loop yaw rate step response, $\\dot{\\theta}_R$'
                        + ' = {0}'.format(-self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Yaw rate")
        axarr[1].plot(t, x[0][:, 1])
        axarr[1].set_ylabel("Steer torque [N * m]")
        axarr[1].set_xlabel("Time [s]")

        f, axarr = plt.subplots(2, 2)
        f.suptitle("Estimator performance")
        axarr[0, 0].plot(t, x[0][:, [2, 6]])    # Roll
        axarr[0, 0].set_title("Roll angle")
        axarr[0, 1].plot(t, x[0][:, [3, 7]])    # Steer
        axarr[0, 1].set_title("Steer angle")
        axarr[1, 0].plot(t, x[0][:, [4, 8]])    # Roll rate
        axarr[1, 0].set_title("Roll rate")
        axarr[1, 1].plot(t, x[0][:, [5, 9]])    # Steer rate
        axarr[1, 1].set_title("Steer rate")


    def closed_loop_zero_input(self, speed):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        C_yr = self.d['C_cl'][i]
        C_u = np.zeros((1, 9))
        C_u[:, 4:] = self.d['F'][i]
        C_x = np.hstack((np.eye(4), np.zeros((4,5))))
        C_xe = np.hstack((np.zeros((4,4)), np.eye(4), np.zeros((4, 1))))

        C = np.vstack((C_yr, C_u, C_x, C_xe))
        D = np.zeros((C.shape[0], 1))
        x0 = np.zeros((9,))
        u = np.zeros((100,))
        t = np.linspace(0, 20, 100)
        # Start the bicycle with non-zero initial conditions to see how
        # quickly estimator converges given the initial condition mismatch
        x0[0] = 10.0 * np.pi/180.0  # Initial roll
        x0[1] = 10.0 * np.pi/180.0  # Initial steer
        x0[2] = 10.0 * np.pi/180.0  # Initial roll rate
        x0[3] = 10.0 * np.pi/180.0  # Initial steer rate
        t, y, x = dlsim((self.d['A_cl'][i],
                         self.d['B_cl'][i, :, 6].reshape((9,1)),
                         C,
                         D,
                         self.d['dt'][i]),
                        u=u,
                        t=t,
                        x0=x0)

        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].plot(t, y[:, 0])
        axarr[0].set_title('Closed loop zero input response, $\\dot{\\theta}_R$'
                        + ' = {0}'.format(-self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Yaw rate")
        axarr[1].plot(t, y[:, 1])
        axarr[1].set_ylabel("Steer torque [N * m]")
        axarr[1].set_xlabel("Time [s]")

        f, axarr = plt.subplots(2, 2)
        f.suptitle("Estimator performance")
        axarr[0, 0].plot(t, y[:, [2, 6]])    # Roll
        axarr[0, 0].set_title("Roll angle")
        axarr[0, 1].plot(t, y[:, [3, 7]])    # Steer
        axarr[0, 1].set_title("Steer angle")
        axarr[1, 0].plot(t, y[:, [4, 8]])    # Roll rate
        axarr[1, 0].set_title("Roll rate")
        axarr[1, 1].plot(t, y[:, [5, 9]])    # Steer rate
        axarr[1, 1].set_title("Steer rate")


def main():
    #yrc.main()
    d = CDataPlotter("controller_data.npz")
    v = 2.0
    #d.plant_evals()
    #d.plant_evals_c()
    #d.plant_damp()
    #d.controller_evals()
    #d.controller_evals_c()
    #d.estimator_evals_c()
    d.controller_gains()
    d.estimator_gains()
    #d.controller_estimator_evals_c()
    #d.closed_loop_evals_c()
    d.closed_loop_bode(-v / rear.R)
    d.noise_to_torque_bode(-v / rear.R)
    d.closed_loop_step(-v / rear.R)
    d.closed_loop_zero_input(-v / rear.R)
    plt.show()

if __name__ == "__main__":
    main()
