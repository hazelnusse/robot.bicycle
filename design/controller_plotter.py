import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import dstep, dlsim
from parameters import rear, w
import yaw_rate_controller as yrc

class CDataPlotter(object):

    def __init__(self, datafile=None, c_data=None):
        if datafile is not None:
            self.d = np.load(datafile)['arr_0']
        elif c_data is not None:
            self.d = c_data
        self.cm = plt.get_cmap('gist_rainbow')

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
    
    def plot_evals_vs_speed(self, data_label, plot_title):
        plt.figure()
        ax = plt.subplot(1, 1, 1)
        eigvals = self.d[data_label]
        speeds = np.shape(eigvals)[0]
        phs = []
        lbl = []
        for v in range(speeds):
            ph, = ax.plot(eigvals[v, :].real, eigvals[v, :].imag,
                          marker='.', linestyle='None',
                          color=self.cm(1.*v/speeds),
                          label='$v = {}$'.format(v))
            if v in range(0, speeds, 10):
                phs.append(ph)
                lbl.append(ph.get_label())
        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.9, box.height])
        l = ax.legend(phs, lbl, loc='center left', bbox_to_anchor=(1, 0.5),
                      numpoints=1)
        #ax.legend(phs, lbl, loc='upper left'
        plt.title(plot_title)

    def controller_evals(self):
        self.plot_evals_vs_speed('controller_evals',
                                 "Controller eigenvalues (discrete)")

    def controller_gains(self):
        plt.figure()
        N = len(self.d)
        ax = plt.plot(-self.d['theta_R_dot'], self.d['K_c'][:,:,:].reshape((N, 5)))
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
        self.plot_evals_vs_speed('estimator_evals',
                                 "Estimator eigenvalues (discrete)")

    def estimator_gains(self):
        N = len(self.d)
        f, axarr = plt.subplots(2, 1, sharex=True)
        lines = axarr[0].plot(-self.d['theta_R_dot'], self.d['K_e'][:, :, 0].reshape((N, 5)))
        lines[0].set_label(r"$k_\phi$")
        lines[1].set_label(r"$k_\delta$")
        lines[2].set_label(r"$k_\dot{\phi}$")
        lines[3].set_label(r"$k_\dot{\delta}$")
        axarr[0].legend(loc=0)
        axarr[0].set_title('Estimator gains vs. speed')
        axarr[0].set_ylabel('Steer measurement gain')
        
        lines = axarr[1].plot(-self.d['theta_R_dot'], self.d['K_e'][:, :, 1].reshape((N, 5)))
        lines[0].set_label(r"$k_\phi$")
        lines[1].set_label(r"$k_\delta$")
        lines[2].set_label(r"$k_\dot{\phi}$")
        lines[3].set_label(r"$k_\dot{\delta}$")
        axarr[1].legend(loc=0)
        axarr[1].set_xlabel('$\\dot{\\theta}_R$')
        axarr[1].set_ylabel('Roll rate measurement gain')
    
    def closed_loop_evals(self):
        plt.figure()
        plt.plot(self.d['estimator_evals'].real,
                 self.d['estimator_evals'].imag, 'r.')
        plt.plot(self.d['controller_evals'].real,
                 self.d['controller_evals'].imag, 'g.')
        plt.title("Closed loop eigenvalues, red: estimator, green: controller")

    def cl_eval_difference(self):
        plt.figure()
        cl_evals_cat = np.hstack((self.d['controller_evals'],
                                  self.d['estimator_evals']))
        cl_evals_cat.sort(axis=1)

        cl_evals_orig = np.sort(self.d['closed_loop_evals'], axis=1)

        diff = cl_evals_cat - cl_evals_orig
        plt.plot(diff.real, diff.imag, 'k.')
        plt.title("Difference in computed closed loop eigenvalues.")


    def open_loop_bode(self, speed, filename=None):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True, figsize=(8.5,11))
        axarr[0].semilogx(self.d['w_ol'][i], self.d['mag_ol'][i])
        axarr[0].set_title('Open loop e to y, LQR only, $(v, \\dot{\\theta}_R$)'
                        + ' = ({0}, {1})'.format(-self.d['theta_R_dot'][i] *
                            rear.R, self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_ol'][i], self.d['phase_ol'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
        if filename is not None:
            f.savefig(filename)

    def closed_loop_bode(self, speed, filename=None):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True, figsize=(8.5,11))
        axarr[0].semilogx(self.d['w_cl'][i], self.d['mag_cl'][i])
        axarr[0].set_title('Closed loop tf, $(v, \\dot{\\theta}_R$)'
                        + ' = ({0}, {1})'.format(-self.d['theta_R_dot'][i] *
                            rear.R, self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_cl'][i], self.d['phase_cl'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
        if filename is not None:
            f.savefig(filename)
    
    def noise_to_torque_bode(self, speed, filename=None):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True, figsize=(8.5,11))
        axarr[0].semilogx(self.d['w_n_to_u'][i], self.d['mag_n_to_u'][i])
        axarr[0].set_title('Noise to torque bode, $(v, \\dot{\\theta}_R$)'
                        + ' = ({0}, {1})'.format(-self.d['theta_R_dot'][i] *
                            rear.R, self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_n_to_u'][i], self.d['phase_n_to_u'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
        if filename is not None:
            f.savefig(filename)

    def closed_loop_step(self, speed, x0):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        C_yr = self.d['C_cl'][i]
        C_u = np.zeros((1, 10))
        C_u[:, 5:] = self.d['K_c'][i]
        C_x = np.hstack((np.eye(5), np.zeros((5, 5))))
        C_xe = np.hstack((np.zeros((5, 5)), np.eye(5)))

        C = np.vstack((C_yr, C_u, C_x, C_xe))
        D = np.zeros((C.shape[0], 1))
        t, y, x = dlsim((self.d['A_cl'][i],
                         self.d['B_cl'][i],
                         C,
                         D,
                         self.d['dt'][i]),
                        u=45.0*np.ones(100)*np.pi/180,
                        t=np.linspace(0, 20, 100),
                        x0=x0)

        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].plot(t, y[:, 0])
        axarr[0].set_title('Closed loop yaw rate step response, $\\dot{\\theta}_R$'
                        + ' = {0}'.format(-self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Yaw rate")
        axarr[1].plot(t, y[:, 1])
        axarr[1].set_ylabel("Steer torque [N * m]")
        axarr[1].set_xlabel("Time [s]")

        f, axarr = plt.subplots(2, 2)
        f.suptitle("Estimator performance")
        axarr[0, 0].plot(t, y[:, [2, 7]])    # Roll
        axarr[0, 0].set_title("Roll angle")
        axarr[0, 1].plot(t, y[:, [3, 8]])    # Steer
        axarr[0, 1].set_title("Steer angle")
        axarr[1, 0].plot(t, y[:, [4, 9]])    # Roll rate
        axarr[1, 0].set_title("Roll rate")
        axarr[1, 1].plot(t, y[:, [5, 10]])    # Steer rate
        axarr[1, 1].set_title("Steer rate")


    def closed_loop_zero_input(self, speed, x0):
        i = np.searchsorted(self.d['theta_R_dot'], speed)

        C_yr = self.d['C_cl'][i]
        C_u = np.zeros((1, 10))
        C_u[:, 5:] = self.d['K_c'][i]
        C_x = np.hstack((np.eye(5), np.zeros((5, 5))))
        C_xe = np.hstack((np.zeros((5, 5)), np.eye(5)))
        C = np.vstack((C_yr, C_u, C_x, C_xe))

        D = np.zeros((C.shape[0], 1))
        u = np.zeros((100,))
        t = np.linspace(0, 20, 100)
        t, y, x = dlsim((self.d['A_cl'][i],
                         self.d['B_cl'][i],
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
        axarr[0, 0].plot(t, y[:, [2, 7]])    # Roll
        axarr[0, 0].set_title("Roll angle")
        axarr[0, 1].plot(t, y[:, [3, 8]])    # Steer
        axarr[0, 1].set_title("Steer angle")
        axarr[1, 0].plot(t, y[:, [4, 9]])    # Roll rate
        axarr[1, 0].set_title("Roll rate")
        axarr[1, 1].plot(t, y[:, [5, 10]])    # Steer rate
        axarr[1, 1].set_title("Steer rate")


def main():
    d = CDataPlotter(c_data=yrc.design_controller())
    speeds = [1.0, 3.0, 5.0]

    x0 = np.zeros((10,))
    # Start the bicycle with non-zero initial conditions to see how
    # quickly estimator converges given the initial condition mismatch
    x0[0] = 1e1 * np.pi/180.0  # Initial roll
    x0[1] = 1e1 * np.pi/180.0  # Initial steer
    x0[2] = 1e1 * np.pi/180.0  # Initial roll rate
    x0[3] = 1e1 * np.pi/180.0  # Initial steer rate
    x0[4] = 0.0                # Initial integral of yaw rate error
    for v in speeds:
        #d.plant_evals()
        #d.plant_evals_c()
        #d.plant_damp()
        #d.closed_loop_bode(-v / rear.R)#, "cl_{0}.pdf".format(int(v)))
        #d.open_loop_bode(-v / rear.R)#, "cl_{0}.pdf".format(int(v)))
        #d.noise_to_torque_bode(-v / rear.R, "n_to_u_{0}.pdf".format(int(v)))
        #d.closed_loop_step(-v / rear.R, x0)
        #d.closed_loop_zero_input(-v / rear.R, x0)
        continue

    #d.controller_gains()
    d.estimator_evals()
    d.controller_evals()
    #d.estimator_gains()
    #d.closed_loop_evals()
    #d.cl_eval_difference()
    plt.show()

if __name__ == "__main__":
    main()
