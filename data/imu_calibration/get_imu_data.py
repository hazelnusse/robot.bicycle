import numpy as np
import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "..", "plots"))
from plot_sample import PlotSample

runs = [PlotSample(x) for x in ["phi_000_theta_000.dat",
                                "phi_000_theta_090.dat",
                                "phi_000_theta_180.dat",
                                "phi_000_theta_270.dat",
                                "phi_090_theta_000.dat",
                                "phi_270_theta_000.dat",
                                "reference.dat"]]
for r in runs:
    print("Processing file {0}".format(r.filename))
    data = r.get_field_data("mpu6050").data
    filename = r.filename.split(".")[0] + "_mpu6050"
    np.savez(filename, 
             accelerometer_x=data['accelerometer_x'],
             accelerometer_y=data['accelerometer_y'],
             accelerometer_z=data['accelerometer_z'],
             gyroscope_x=data['gyroscope_x'],
             gyroscope_y=data['gyroscope_y'],
             gyroscope_z=data['gyroscope_z'])
    print("File {0} processed; MPU6050 data saved to {1}.npz".format(
            r.filename, filename))

