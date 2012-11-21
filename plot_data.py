import matplotlib.pyplot as plt
import sampleplotter as sp

datafile = "collected_data_converted/samples_0152_21NOV2012_converted.dat"
samples = sp.Samples(datafile)
samples.plotEncoders()
samples.plotTime()
#samples.plotAccelerometer()
#samples.plotGyroscope()
#samples.plotTemperature()
plt.show()
