import matplotlib.pyplot as plt
import sampleplotter as sp

datafile = "collected_data_converted/samples_1612_27NOV2012_converted.dat"
samples = sp.Samples(datafile)
samples.plotEncoders(N=10)
#samples.plotTime()
#samples.plotAccelerometer()
#samples.plotGyroscope()
#samples.plotTemperature()
plt.show()
