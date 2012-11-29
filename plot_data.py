import matplotlib.pyplot as plt
import sampleplotter as sp

datafile = "collected_data_converted/samples_1501_29NOV2012_converted.dat"
samples = sp.Samples(datafile)
samples.plotRearWheel(N=10)
#samples.plotTime()
#samples.plotAccelerometer()
#samples.plotGyroscope()
#samples.plotTemperature()
plt.show()
