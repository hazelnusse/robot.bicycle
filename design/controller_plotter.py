import matplotlib.pyplot as plt
from matplotlib.patches import Circle
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

    def plant_evals_c(self):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(-self.d['theta_R_dot'] * rear.R,
                self.d['plant_evals_c'].real, 'k.')
        ax.plot(-self.d['theta_R_dot'] * rear.R,
                self.d['plant_evals_c'].imag, 'b.')
        ax.axis('tight')
        ax.set_xlabel('v [m / s]')
        ax.set_ylabel('$\\lambda$')
        ax.set_title("Plant eigenvalues (continuous)")
    
    def plant_evals_d(self):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(self.d['plant_evals_d'].real,
                self.d['plant_evals_d'].imag, 'k.')
        ax.axis('equal')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.add_patch(Circle((0, 0), radius=1, fill=False))
        ax.set_xlabel('real($\\lambda$)')
        ax.set_xlabel('imag($\\lambda$)')
        ax.set_title("Plant eigenvalues (discrete)")
    
    def closed_loop_evals(self):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(self.d['estimator_evals'].real,
                 self.d['estimator_evals'].imag, 'r.')
        ax.plot(self.d['controller_evals'].real,
                 self.d['controller_evals'].imag, 'g.')
        ax.axis('equal')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.add_patch(Circle((0, 0), radius=1, fill=False))
        plt.title("Closed loop eigenvalues, red: estimator, green: controller")

    def controller_gains(self):
        plt.figure()
        N = len(self.d)
        ax = plt.plot(self.d['theta_R_dot'],
                      self.d['K_c'][:,:,:].reshape((N, 4)))
        ax[0].set_label(r"$k_\phi$")
        ax[1].set_label(r"$k_\delta$")
        ax[2].set_label(r"$k_\dot{\phi}$")
        ax[3].set_label(r"$k_\dot{\delta}$")
        plt.legend(loc=0)
        plt.title('Feedback gains vs. speed')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.ylabel('Gain')

    def estimator_gains(self):
        N = len(self.d)
        f, axarr = plt.subplots(2, 1, sharex=True)
        lines = axarr[0].plot(self.d['theta_R_dot'],
                              self.d['K_e'][:, :, 0].reshape((N, 4)))
        lines[0].set_label(r"$k_\phi$")
        lines[1].set_label(r"$k_\delta$")
        lines[2].set_label(r"$k_\dot{\phi}$")
        lines[3].set_label(r"$k_\dot{\delta}$")
        axarr[0].legend(loc=0)
        axarr[0].set_title('Estimator gains vs. speed')
        axarr[0].set_ylabel('Steer measurement gain')
        
        lines = axarr[1].plot(self.d['theta_R_dot'],
                              self.d['K_e'][:, :, 1].reshape((N, 4)))
        lines[0].set_label(r"$k_\phi$")
        lines[1].set_label(r"$k_\delta$")
        lines[2].set_label(r"$k_\dot{\phi}$")
        lines[3].set_label(r"$k_\dot{\delta}$")
        axarr[1].legend(loc=0)
        axarr[1].set_xlabel('$\\dot{\\theta}_R$')
        axarr[1].set_ylabel('Roll rate measurement gain')

    def pi_gains(self):
        plt.figure()
        N = len(self.d)
        ax = plt.plot(self.d['theta_R_dot'],
                      np.hstack((self.d['Kp'].reshape((N, 1)),
                                 self.d['Kp_fit'].reshape((N, 1)),
                                 self.d['Ki'].reshape((N, 1)),
                                 self.d['Ki_fit'].reshape((N, 1)))))
        ax[0].set_label(r"$K_p$")
        ax[1].set_label(r"$K_{pf}$")
        ax[2].set_label(r"$K_i$")
        ax[3].set_label(r"$K_{if}$")
        plt.legend(loc=0)
        plt.title('Yaw rate PI gains vs. speed')
        plt.xlabel('$\\dot{\\theta}_R$')
        plt.ylabel('Gain')

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
        self.plot_evals_vs_speed("controller_evals",
                                 "Controller eigenvalues (discrete)")

    def estimator_evals(self):
        self.plot_evals_vs_speed("estimator_evals",
                                 "Estimator eigenvalues (discrete)")
    
    def cl_eval_difference(self):
        plt.figure()
        cl_evals_cat = np.hstack((self.d['controller_evals'],
                                  self.d['estimator_evals']))
        cl_evals_cat.sort(axis=1)

        cl_evals_orig = np.sort(self.d['closed_loop_evals'], axis=1)

        diff = cl_evals_cat - cl_evals_orig
        plt.plot(diff.real, diff.imag, 'k.')
        plt.title("Difference in computed closed loop eigenvalues.")

    def bode_r_to_psi_dot(self, speed, filename=None):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True, figsize=(8.5,11))
        axarr[0].semilogx(self.d['w_r_to_psi_dot'][i],
                          self.d['mag_r_to_psi_dot'][i])
        axarr[0].set_title('Closed loop tf, $(v, \\dot{\\theta}_R$)'
                        + ' = ({0}, {1})'.format(-self.d['theta_R_dot'][i] *
                            rear.R, self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_r_to_psi_dot'][i],
                          self.d['phase_r_to_psi_dot'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
        if filename is not None:
            f.savefig(filename)

    def bode_open_loop_e_to_psi_dot(self, speed, filename=None):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True, figsize=(8.5,11))
        axarr[0].semilogx(self.d['w_e_to_psi_dot'][i],
                          self.d['mag_e_to_psi_dot'][i])
        axarr[0].set_title('Inner LQR/LQG loop closed, PI outer loop open:\n' 
                        +  '$e_{\dot{\psi}}$ to $\\dot{\\psi}}$\n'
                        +  '$(v, \\dot{\\theta}_R$)'
                        + ' = ({0}, {1})'.format(-self.d['theta_R_dot'][i] *
                            rear.R, self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_e_to_psi_dot'][i],
                          self.d['phase_e_to_psi_dot'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
        if filename is not None:
            f.savefig(filename)
    
    def bode_psi_r_to_psi_dot(self, speed, filename=None):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        f, axarr = plt.subplots(2, 1, sharex=True, figsize=(8.5,11))
        axarr[0].semilogx(self.d['w_psi_r_to_psi_dot'][i],
                          self.d['mag_psi_r_to_psi_dot'][i])
        axarr[0].set_title('Closed loop\n: $\\dot{\\psi}_r$ to $\\dot{\\psi}$, $(v, \\dot{\\theta}_R$)'
                        + ' = ({0}, {1})'.format(-self.d['theta_R_dot'][i] *
                            rear.R, self.d['theta_R_dot'][i]))
        axarr[0].set_ylabel("Magnitude [dB]")
        axarr[1].semilogx(self.d['w_psi_r_to_psi_dot'][i],
                          self.d['phase_psi_r_to_psi_dot'][i])
        axarr[1].set_xlabel("Frequency [Hz]")
        axarr[1].set_ylabel("Phase [deg]")
        if filename is not None:
            f.savefig(filename)
    
    def step_r_to_psi_dot(self, speed, x0):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        C_yr = self.d['C_cl'][i]
        C_u = np.zeros((1, 8))
        C_u[:, 4:] = self.d['K_c'][i]
        C_x = np.hstack((np.eye(4), np.zeros((4, 4))))
        C_xe = np.hstack((np.zeros((4, 4)), np.eye(4)))

        C = np.vstack((C_yr, C_u, C_x, C_xe))
        D = np.zeros((C.shape[0], 1))
        t, y, x = dlsim((self.d['A_cl'][i],
                         self.d['B_cl'][i],
                         C,
                         D,
                         self.d['dt'][i]),
                        u=np.ones(100),
                        t=np.linspace(0, 20, 100),
                        x0=x0)

        speed_string = '$\\dot{\\theta}_R$'
        speed_string += ' = {0}'.format(self.d['theta_R_dot'][i])

        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].plot(t, y[:, 0])
        axarr[0].set_title("Closed loop yaw rate step response, " + speed_string)
        axarr[0].set_ylabel("Yaw rate")
        axarr[1].plot(t, y[:, 1])
        axarr[1].set_ylabel("Steer torque [N * m]")
        axarr[1].set_xlabel("Time [s]")

        f, axarr = plt.subplots(2, 2)
        f.suptitle("Estimator performance, " + speed_string)
        axarr[0, 0].plot(t, y[:, [2, 6]])    # Roll
        axarr[0, 0].set_title("Roll angle")
        axarr[0, 1].plot(t, y[:, [3, 7]])    # Steer
        axarr[0, 1].set_title("Steer angle")
        axarr[1, 0].plot(t, y[:, [4, 8]])    # Roll rate
        axarr[1, 0].set_title("Roll rate")
        axarr[1, 1].plot(t, y[:, [5, 9]])    # Steer rate
        axarr[1, 1].set_title("Steer rate")


    def lqrlqg_zero_input(self, speed, x0):
        i = np.searchsorted(self.d['theta_R_dot'], speed)

        C_yr = self.d['C_cl'][i]
        C_u = np.zeros((1, 8))
        C_u[:, 4:] = self.d['K_c'][i]
        C_x = np.hstack((np.eye(4), np.zeros((4, 4))))
        C_xe = np.hstack((np.zeros((4, 4)), np.eye(4)))
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

        speed_string = '$\\dot{\\theta}_R$'
        speed_string += ' = {0}'.format(self.d['theta_R_dot'][i])

        f, axarr = plt.subplots(2, 1, sharex=True)
        axarr[0].plot(t, y[:, 0])
        axarr[0].set_title("Closed loop zero input response, " + speed_string)
        axarr[0].set_ylabel("Yaw rate")
        axarr[1].plot(t, y[:, 1])
        axarr[1].set_ylabel("Steer torque [N * m]")
        axarr[1].set_xlabel("Time [s]")

        f, axarr = plt.subplots(2, 2)
        f.suptitle("Estimator performance, " + speed_string)
        axarr[0, 0].plot(t, y[:, [2, 6]])    # Roll
        axarr[0, 0].set_title("Roll angle")
        axarr[0, 1].plot(t, y[:, [3, 7]])    # Steer
        axarr[0, 1].set_title("Steer angle")
        axarr[1, 0].plot(t, y[:, [4, 8]])    # Roll rate
        axarr[1, 0].set_title("Roll rate")
        axarr[1, 1].plot(t, y[:, [5, 9]])    # Steer rate
        axarr[1, 1].set_title("Steer rate")

    def step_yr_cl(self, speed, x0):
        i = np.searchsorted(self.d['theta_R_dot'], speed)
        print(i)
        C_yr = self.d['C_yr_cl'][i]
        C_u_lqr = np.zeros((1, 9))
        C_u_lqr[0, 5:] = self.d['K_c'][i]         # portion of control from LQR/LQG
        C_u_pi = np.zeros((1, 9))
        C_u_pi[0, 0] = self.d['Ki'][i] * 0.005
        C_u_pi[0, 1:5] = -self.d['Kp'][i] * self.d['C_z'][i]
        C_x = np.hstack((np.zeros((4, 1)), np.eye(4), np.zeros((4, 4))))
        C_xe = np.hstack((np.zeros((4, 5)), np.eye(4)))

        C = np.vstack((C_yr, C_u_lqr, C_u_pi, C_x, C_xe))
        D = np.zeros((C.shape[0], 1))
        D[2, 0] = self.d['Kp'][i]
        t, y, x = dlsim((self.d['A_yr_cl'][i],
                         self.d['B_yr_cl'][i],
                         C,
                         D,
                         self.d['dt'][i]),
                        u=45*np.pi/180.*np.ones(100),
                        t=np.linspace(0, 20, 100),
                        x0=x0)
        speed_string = '$\\dot{\\theta}_R$'
        speed_string += ' = {0}'.format(self.d['theta_R_dot'][i])

        f, ax = plt.subplots(2, 1, sharex=True)
        ax[0].plot(t, y[:, 0])
        ax[0].set_title("Closed loop yaw rate step response, " + speed_string)
        ax[0].set_ylabel("Yaw rate")
        ax[1].plot(t, y[:, 1] + y[:, 2])
        ax[1].set_ylabel("Steer torque [N * m]")
        ax[1].set_xlabel("Time [s]")

        f, ax = plt.subplots(2, 2)
        f.suptitle("Estimator performance, " + speed_string)
        ax[0, 0].plot(t, y[:, [3, 7]])    # Roll
        ax[0, 0].set_title("Roll angle")
        ax[0, 1].plot(t, y[:, [4, 8]])    # Steer
        ax[0, 1].set_title("Steer angle")
        ax[1, 0].plot(t, y[:, [5, 9]])    # Roll rate
        ax[1, 0].set_title("Roll rate")
        ax[1, 1].plot(t, y[:, [6, 10]])    # Steer rate
        ax[1, 1].set_title("Steer rate")

        #fig = plt.figure()
        #l = plt.plot(t, x[:, 0])
        

def main():
    d = CDataPlotter(datafile="controller_data.npz") #c_data=yrc.design_controller())
    speeds = [1.0, 3.0, 5.0, 7.0, 9.0]

    x0 = np.zeros((8,))
    # Start the bicycle with non-zero initial conditions to see how
    # quickly estimator converges given the initial condition mismatch
    x0[0] = 1e1 * np.pi/180.0  # Initial roll
    x0[1] = 1e1 * np.pi/180.0  # Initial steer
    x0[2] = 1e1 * np.pi/180.0  # Initial roll rate
    x0[3] = 1e1 * np.pi/180.0  # Initial steer rate
    for v in speeds:
        d.plant_evals_c()
        d.plant_evals_d()
        #d.bode_r_to_psi_dot(-v / rear.R)#, "cl_{0}.pdf".format(int(v)))
        d.bode_open_loop_e_to_psi_dot(-v / rear.R, "e_to_psi_dot_{0}.pdf".format(int(v)))
        #d.bode_psi_r_to_psi_dot(-v / rear.R)
        #d.step_r_to_psi_dot(-v / rear.R, x0)
        #d.lqrlqg_zero_input(-v / rear.R, x0)
        #d.step_yr_cl(-v / rear.R, np.hstack((0, x0)))

    # Speed parameterized plots
    #d.controller_gains()
    #d.estimator_gains()
    #d.pi_gains()
    #d.estimator_evals()
    #d.closed_loop_evals()
    #d.controller_evals()
    #d.cl_eval_difference()
    plt.show()

if __name__ == "__main__":
    main()

