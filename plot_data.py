import matplotlib.pyplot as plt
import sampleplotter as sp

datafile = "collected_data_converted/samples_2227_23OCT2012_converted.dat"
samples = sp.Samples(datafile)
samples.plotEncoders(v_est='LPP')
samples.plotAccelerometer()
samples.plotGyroscope()
samples.plotTemperature()
plt.show()
