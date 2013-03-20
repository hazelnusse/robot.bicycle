import os
import sys
sys.path.append(os.path.join(os.getcwd(), "..", "..", "common"))
import numpy as np
import sampletypes as st
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

data = np.fromfile("./rw1_5A00_converted.dat", dtype=st.sample_t)

t = data['T'][10:101]
theta = np.unwrap(data['RearWheelAngle'][10:101])

I = -1.5
fitfunc = lambda t, *p: p[3]*t**3 + p[2]*t**2 + p[1]*t + p[0]
p0 = [0, 0, 4, 0]
popt, pcov = curve_fit(fitfunc, t, theta, p0, ftol=1e-12, xtol=1e-12)
print(popt)
print(pcov)
fit = np.empty((len(t),))
for i, ti in enumerate(t):
    fit[i] = fitfunc(ti, *popt)

kt = 2.0*popt[2]/I
print("kt = {0}".format(kt))
plt.plot(t, theta)
plt.plot(t, fit)
plt.show()
