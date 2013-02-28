from bicycle import WheelAssemblyGyrostat, Bicycle, Whipple

rear = WheelAssemblyGyrostat()
front = WheelAssemblyGyrostat()

# BEGIN Copy paste from output of data/physicalparameters/compute_model_parameters.py
rear.Ixx =         1.542023074848535
rear.Iyy =         3.557313581104186
rear.Izz =         3.014542484492762
rear.Ixz =        0.8392298939950623
rear.J   =        0.1138146697426332
rear.m   =                     34.05
rear.R   =        0.3359074174667941
rear.r   =                         0
rear.a   =        0.5139283534655644
rear.b   =       -0.2185938844305652
rear.c   =        0.9637521999999999
front.Ixx =        0.1834745058089018
front.Iyy =        0.2257063173763909
front.Izz =       0.06923407270022158
front.Ixz =     -0.009663296585475524
front.J   =       0.09227403707515749
front.m   =                      2.95
front.R   =        0.3358632190780645
front.r   =                         0
front.a   =      -0.02086466545111477
front.b   =       -0.1524559736777648
front.c   =                 0.0478155
ls = 0.3434334
# END Copy paste from output of data/physicalparameters/compute_model_parameters.py

# Assumed
g = 9.81

robotic_bicycle = Bicycle()
robotic_bicycle.set_parameters(rear, front, ls, g)
robotic_bicycle.solve_configuration_constraint_and_set_state()
robotic_bicycle.set_steady_constraint_forces()

benchmark_bicycle = Bicycle()
w = Whipple()
benchmark_bicycle.set_parameters(w)
benchmark_bicycle.solve_configuration_constraint_and_set_state()
benchmark_bicycle.set_steady_constraint_forces()
