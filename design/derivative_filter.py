from control import tf, bode_plot
from matplotlib.pyplot import show
from scipy import pi
from scipy.signal import cont2discrete, tf2ss
from numpy import logspace

def astrom_derivative_filter(Td, N):
    num = [Td, 0]
    den = [Td/N, 1]
    return tf(num, den), num, den

omega = logspace(-1, 3, num=100) * 2 * pi

num = [1]
den = [0.01, 1]

lpf = tf(num, den)
mag, phase, f = bode_plot(lpf, omega=omega, dB=True, Hz=True)
print(mag)
print(phase)
print(f)

delay = phase / (360.0 * f)

lpf_ss = tf2ss(num, den)
print(lpf_ss)
lpf_ss_d = cont2discrete(lpf_ss, dt=0.05)
print(lpf_ss_d)


show()

