from sympy import symbols
from sympy.physics.mechanics import *
q = symbols('q:3')
wx, wy, wz = symbols('wx wy wz')
N = ReferenceFrame('N')                    # Inertial frame
Y = N.orientnew('Y', 'Axis', [q[0], N.z])  # Rear yaw frame (heading)
L = Y.orientnew('L', 'Axis', [q[1], Y.x])  # Rear lean frame (roll)
R = L.orientnew('R', 'Axis', [q[2], L.y])  # Fixed to rear frame
w = wx*R.x + wy*R.y + wz*R.z

eq = {q[1]:0.0, q[2]:symbols('q2_eq')}
yaw_rate = w & Y.z
print("Yaw rate:", yaw_rate)
yaw_rate_0 = yaw_rate.subs(eq)
yaw_rate_dq = [yaw_rate.diff(qi).subs(eq) for qi in q]

print("Yaw rate d0:", yaw_rate_0)
print("Yaw rate dq:", yaw_rate_dq)
