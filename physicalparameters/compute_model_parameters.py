import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt
import psycopg2
from dataprocessing import *

# Error tolerance used for curve_fit
ftol = 1e-12
xtol = 1e-12

# Establish database connection
conn = psycopg2.connect(database="robot_bicycle_parameters", user="hazelnusse")
cur = conn.cursor()

# Fork mass center measurements
cur.execute('select alpha, a from parametermeasurements.forkcenterofmass;')
results = np.array(cur.fetchall())
alpha_fork = results[:, 0]
fork_d, fork_f = compute_d_f(results)[0]

# Frame mass center measurements
cur.execute('select alpha, a from parametermeasurements.framecenterofmass;')
results = np.array(cur.fetchall())
alpha_frame = results[:, 0]
frame_d, frame_f = compute_d_f(results)[0]

fig_fork_torsional = plt.figure()
fig_frame_torsional = plt.figure()
fig_calibrationrod = plt.figure()
fig_rearwheel_torsional = plt.figure()
fig_frontwheel_torsional = plt.figure()

axes_fork = []
axes_frame = []
axes_rod = fig_calibrationrod.add_subplot(111)
axes_rearwheel = []
axes_frontwheel = []

# Calculation of calibration rod period
rod_periods = []
for i in [1, 2, 3, 4, 5]:
    cur.execute('select samplefrequency, duration, voltage from ' +
                'parametermeasurements.rodtorsionalpendulumtimeseries'
                ' where id = {0};'.format(i))
    f, d, v = cur.fetchall()[0]
    v = np.array(v)
    t, Ts = np.linspace(0, d, len(v), endpoint=False, retstep=True)
    assert (Ts == 1.0/f)
    p0 = [max(v)-v.mean(), 0.01, 1.0, np.pi/2, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    rod_periods.append(T)
    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_rod.plot(t, v, colors[i-1]+',')
    axes_rod.plot(t, v_fit, colors[i-1]+'-')

axes_rod.set_title("Calibration rod torsional pendulum")

# Compute the mean of all periods
rod_period = np.array(rod_periods).mean()
#print("Rod periods = " + str(rod_periods))
#print("Mean rod period = " + str(rod_period))

# Calculation of calibration rod inertia
cur.execute('select mass, outsidediameter, insidediameter, length from parametermeasurements.calibrationrods;')
mass, Od, Id, length = cur.fetchall()[0]
rod_transverse_inertia = 1.0/12.0*mass*(3.0*(Od**2 + Id**2)/4.0 + length**2)
#print("Rod transverse inertia = " + str(rod_transverse_inertia))

# Frame and Fork Torsional Pendulum measurements
fork_periods = []
frame_periods = []
for i in [1, 2, 3, 4]:
    axes_frame.append(fig_frame_torsional.add_subplot(320 + i))
    axes_fork.append(fig_fork_torsional.add_subplot(320 + i))

    # Frame period calculations
    cur.execute('select * from' +
                ' parametermeasurements.frametorsionalpendulumtimeseries '+
                ' where configuration = {0};'.format(i))
    results = np.array(cur.fetchall())
    T_sum = 0
    for j, trial in enumerate(results):
        v = np.array(trial[4])
        t, Ts = np.linspace(0, trial[3], len(trial[4]), endpoint=False, retstep=True)
        assert (Ts == 1.0/trial[2])
        p0 = [max(v)-v.mean(), 0.001, 2.0, np.pi/2, v.mean()]
        popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
                ftol=ftol, xtol=xtol)
        a, zeta, T, d, e = popt
        T_sum += T
        v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
        axes_frame[i-1].plot(t, v, colors[j] + ',')
        axes_frame[i-1].plot(t, v_fit, colors[j] + '-')
    axes_frame[i-1].set_title("Frame torsional pendulum configuration {0}".format(i))

    # Compute the mean of all periods for all trials at a given configuration
    frame_periods.append(T_sum / (j + 1))

    # Fork period calculations
    cur.execute('select * from' +
                ' parametermeasurements.forktorsionalpendulumtimeseries '+
                ' where configuration = {0};'.format(i))
    results = np.array(cur.fetchall())
    T_sum = 0
    for j, trial in enumerate(results):
        v = np.array(trial[4])
        t, Ts = np.linspace(0, trial[3], len(trial[4]), endpoint=False, retstep=True)
        assert (Ts == 1.0/trial[2])
        p0 = [max(v)-v.mean(), 0.01, 0.3, np.pi/2, v.mean()]
        popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
                ftol=ftol, xtol=xtol)
        a, zeta, T, d, e = popt
        T_sum += T
        v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
        axes_fork[i-1].plot(t, v, colors[j]+',')
        axes_fork[i-1].plot(t, v_fit, colors[j]+'-')
    axes_fork[i-1].set_title("Fork torsional pendulum configuration {0}".format(i))

    # Compute the mean of all periods for all trials at a given configuration
    fork_periods.append(T_sum / (j+1))

#print("Mean frame periods:")
#print(frame_periods)
#print("Mean fork periods:")
#print(fork_periods)

# Compute frame inertia scalars
frame_inertias = compute_Ixx_Ixz_Izz(alpha_frame[:-1], frame_periods,
                               rod_transverse_inertia, rod_period)
IRxx, IRxz, IRzz = frame_inertias[0]

# Compute fork inertia scalars
fork_inertias = compute_Ixx_Ixz_Izz(alpha_fork[:-1], fork_periods,
        rod_transverse_inertia, rod_period)
IFxx, IFxz, IFzz = fork_inertias[0]

# Frame and Fork compound pendulum
axes_frame.append(fig_frame_torsional.add_subplot(325))
axes_fork.append(fig_fork_torsional.add_subplot(325))

# Frame period calculations
cur.execute('select * from parametermeasurements.framecompoundpendulumtimeseries;')
results = np.array(cur.fetchall())
T_sum = 0
for j, trial in enumerate(results):
    v = np.array(trial[4])
    t, Ts = np.linspace(0, trial[3], len(trial[4]), endpoint=False, retstep=True)
    assert (Ts == 1.0/trial[2])
    p0 = [max(v)-v.mean(), 0.01, 1.75, 0.0, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    T_sum += T
    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_frame[4].plot(t, v, colors[j] + ',')
    axes_frame[4].plot(t, v_fit, colors[j] + '-')
axes_frame[4].set_title("Frame compound pendulum configuration")

# Compute the mean of all periods for all trials
T_frame_compound = T_sum / (j + 1)

# Fork period calculations
cur.execute('select * from parametermeasurements.forkcompoundpendulumtimeseries;')
results = np.array(cur.fetchall())
T_sum = 0
for j, trial in enumerate(results):
    v = np.array(trial[4])
    t, Ts = np.linspace(0, trial[3], len(trial[4]), endpoint=False, retstep=True)
    assert (Ts == 1.0/trial[2])
    p0 = [max(v)-v.mean(), 0.01, 1.25, 0.0, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    T_sum += T
    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_fork[4].plot(t, v, colors[j]+',')
    axes_fork[4].plot(t, v_fit, colors[j]+'-')
axes_fork[4].set_title("Fork compound pendulum configuration")

# Compute the mean of all periods for all trials
T_fork_compound = T_sum / (j + 1)

# Compute rear wheel inertias
cur.execute('select samplefrequency, duration, voltage from parametermeasurements.rearwheeltorsionalpendulumtimeseries;')
results = cur.fetchall()
rearwheel_periods = []
axes_rearwheel.append(fig_rearwheel_torsional.add_subplot(211))

for i, trial in enumerate(results):
    f, d, v = trial
    v = np.array(v)
    t, Ts = np.linspace(0, d, len(v), endpoint=False, retstep=True)
    assert (Ts == 1.0/f)

    # Initial guess on parameters
    p0 = [max(v)-v.mean(), 0.001, .5, np.pi/2, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    rearwheel_periods.append(T)

    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_rearwheel[0].plot(t, v, colors[i] + ',')
    axes_rearwheel[0].plot(t, v_fit, colors[i] + '-')

axes_rearwheel[0].set_title('Rear wheel torsional pendulum')

#print("Rear wheel periods: {0}".format(rearwheel_periods))
rearwheel_period = np.array(rearwheel_periods).mean()
#print("Mean rear wheel period = {0}".format(rearwheel_period))
IRWxx = rod_transverse_inertia*(rearwheel_period/rod_period)**2

# Compute front wheel inertias
cur.execute('select samplefrequency, duration, voltage from parametermeasurements.frontwheeltorsionalpendulumtimeseries;')
results = cur.fetchall()
frontwheel_periods = []
axes_frontwheel.append(fig_frontwheel_torsional.add_subplot(211))

for i, trial in enumerate(results):
    f, d, v = trial
    v = np.array(v)
    t, Ts = np.linspace(0, d, len(v), endpoint=False, retstep=True)
    assert (Ts == 1.0/f)

    # Initial guess on parameters
    p0 = [max(v)-v.mean(), 0.001, .4, np.pi/2, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    frontwheel_periods.append(T)

    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_frontwheel[0].plot(t, v, colors[i] + ',')
    axes_frontwheel[0].plot(t, v_fit, colors[i] + '-')

axes_frontwheel[0].set_title('Front wheel torsional pendulum')
#print("Front wheel periods: {0}".format(frontwheel_periods))
frontwheel_period = np.array(frontwheel_periods).mean()
#print("Mean front wheel period = {0}".format(frontwheel_period))
IFWxx = rod_transverse_inertia*(frontwheel_period/rod_period)**2

# Compute rear wheel spin inertias
cur.execute('select samplefrequency, duration, voltage from parametermeasurements.rearwheelcompoundpendulumtimeseries;')
results = cur.fetchall()
rearwheel_periods = []
axes_rearwheel.append(fig_rearwheel_torsional.add_subplot(212))

for i, trial in enumerate(results):
    f, d, v = trial
    v = np.array(v)
    t, Ts = np.linspace(0, d, len(v), endpoint=False, retstep=True)
    assert (Ts == 1.0/f)

    # Initial guess on parameters
    p0 = [max(v)-v.mean(), 0.001, 1.4, np.pi/2, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    rearwheel_periods.append(T)

    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_rearwheel[1].plot(t, v, colors[i] + ',')
    axes_rearwheel[1].plot(t, v_fit, colors[i] + '-')

axes_rearwheel[1].set_title('Rear wheel compound pendulum')
T_rw_compound = np.array(rearwheel_periods).mean()

# Compute front wheel spin inertias
cur.execute('select samplefrequency, duration, voltage from parametermeasurements.frontwheelcompoundpendulumtimeseries;')
results = cur.fetchall()
frontwheel_periods = []
axes_frontwheel.append(fig_frontwheel_torsional.add_subplot(212))

for i, trial in enumerate(results):
    f, d, v = trial
    v = np.array(v)
    t, Ts = np.linspace(0, d, len(v), endpoint=False, retstep=True)
    assert (Ts == 1.0/f)

    # Initial guess on parameters
    p0 = [max(v)-v.mean(), 0.001, 1.4, np.pi/2, v.mean()]
    popt, pcov = scipy.optimize.curve_fit(decaying_sinusoid, t, v, p0=p0,
            ftol=ftol, xtol=xtol)
    a, zeta, T, d, e = popt
    frontwheel_periods.append(T)

    v_fit = decaying_sinusoid(t, a, zeta, T, d, e)
    axes_frontwheel[1].plot(t, v, colors[i] + ',')
    axes_frontwheel[1].plot(t, v_fit, colors[i] + '-')

axes_frontwheel[1].set_title('Front wheel compound pendulum')
T_fw_compound = np.array(frontwheel_periods).mean()

# Get masses, compound pendulum lengths
cur.execute("select mass from parametermeasurements.massmeasurements;")
results = cur.fetchall()
(m_rw,), (m_fw,), (m_fork,), (m_frame,) = results
cur.execute("select length from parametermeasurements.compoundpendulumlengths;")
(l_rw,), (l_fw,) = cur.fetchall()

# Rear wheel radius
cur.execute("select revolutions, distance from parametermeasurements.wheelrolloutmeasurements where body = 'Rear Wheel';")
results = cur.fetchall()
circumference_rear = np.array([d / N for N, d in results])
radius_rear = circumference_rear.mean() / (2.0*np.pi)

# Front wheel radius
cur.execute("select revolutions, distance from parametermeasurements.wheelrolloutmeasurements where body = 'Front Wheel';")
results = cur.fetchall()
circumference_front = np.array([d / N for N, d in results])
radius_front = circumference_front.mean() / (2.0*np.pi)

g = 9.81
IRWyy = (T_rw_compound / 2. / np.pi)**2. * m_rw * g * l_rw - m_rw * l_rw**2.
IFWyy = (T_fw_compound / 2. / np.pi)**2. * m_fw * g * l_fw - m_fw * l_fw**2.
l_frame = np.sqrt(frame_d**2.0 + frame_f**2.0)
l_fork = np.sqrt(fork_d**2.0 + fork_f**2.0)
IRyy = (T_frame_compound / 2. / np.pi)**2.0 * m_frame * g * l_frame - m_frame * l_frame**2.
IFyy = (T_fork_compound / 2. / np.pi)**2.0 * m_fork * g * l_fork - m_fork * l_fork**2.

# Axle measurements
cur.execute("select * from parametermeasurements.axleoffsetmeasurements;")
results = cur.fetchall()[0]
id, lr, lf, ls = results


frame_com = [frame_d, 0.0, frame_f]
I_Frame = [IRxx, IRyy, IRzz, 0.0, 0.0, IRxz]
I_RW = [IRWxx, IRWyy, IRWxx, 0.0, 0.0, 0.0]
mr, r_RWO_RO, I_R_RO = GyrostatParameters(m_frame, m_rw, frame_com, I_Frame, I_RW)
print("Rear Gyrostat:")
print("Inertia = {0}".format(I_R_RO))
print("J = {0}".format(IRWyy))
print("r_RWO_RO = {0}".format(r_RWO_RO))
print("m_total = {0}".format(mr))
print("a = {0}".format(radius_rear))
print("g = {0}".format(lr))

fork_com = [fork_d, 0.0, fork_f]
I_Fork = [IFxx, IFyy, IFzz, 0.0, 0.0, IFxz]
I_FW = [IFWxx, IFWyy, IFWxx, 0.0, 0.0, 0.0]
mf, r_FWO_FO, I_F_FO = GyrostatParameters(m_fork, m_fw, fork_com, I_Fork, I_FW)
print("Front Gyrostat:")
print("Inertia = {0}".format(I_F_FO))
print("J = {0}".format(IFWyy))
print("r_FWO_FO = {0}".format(r_FWO_FO))
print("m_total = {0}".format(mf))
print("a = {0}".format(radius_front))
print("g = {0}".format(-lf))

print("ls = {0}".format(ls))


#plt.show()
cur.close()
conn.close()
