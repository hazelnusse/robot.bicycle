"""Observer and feedback controller design for rear wheel speed control.

Design notes:
    - US Digital optical encoder has 50 CPR of encoder shaft.
    - 100:25 wheel to encoder gearing implies 200 CPR of wheel.
    - Detecting rising and falling edges of A or B channels results in 400 CPR
      of wheel (half quadrature).
    - Detecting rising and falling edges of A and B channels results in 800 CPR
      (full quadrature).
    - Wheel angle measurement uses full quadrature.  One timer count represents
      2*pi/800 radians
    - Wheel speed measurement uses half quadrature.  One cycle between a rising
      and falling edge represents 2*pi/400 radians.
    - Encoder rising and falling edges occur asynchronously from main control
      loop.
    - Absolute position of wheel is generally not of interest.
    - Speed of wheel is of interest.

"""

from sympy import symbols, Poly
import numpy as np

a, b, T, s, w, x, z, I = symbols('a b T s w x z I')
bilinear_transform = {s : 2/T*(z-1)/(z+1)}

# Continuous time transfer function from I(s) to w(s) of the following plant
# model:
#   dw/dt = a*w + b*I
G_s = b/(s-a)

G_z_num, G_z_den = G_s.subs(bilinear_transform).as_numer_denom()
G_z_den = Poly(G_z_den, z)

G_z_num = Poly(G_z_num / G_z_den.LC(), z) # divide by leading coefficient of den
G_z_den = G_z_den.monic()                 # make denominator monic
assert(G_z_den.coeffs()[0] == 1)

kp = Poly(G_z_num.coeffs()[0], z)

G_z_N_p = G_z_num - kp * G_z_den

print(kp)
print(G_z_N_p)
print(G_z_den)
from sympy import symbols, Poly
import numpy as np

a, b, T, s, w, x, z, I = symbols('a b T s w x z I')
bilinear_transform = {s : 2/T*(z-1)/(z+1)}

# Continuous time transfer function from I(s) to w(s) of the following plant
# model:
#   dw/dt = a*w + b*I
G_s = b/(s-a)

G_z_num, G_z_den = G_s.subs(bilinear_transform).as_numer_denom()
G_z_den = Poly(G_z_den, z)

G_z_num = Poly(G_z_num / G_z_den.LC(), z) # divide by leading coefficient of den
G_z_den = G_z_den.monic()                 # make denominator monic
assert(G_z_den.coeffs()[0] == 1)
A = - G_z_den.coeffs()[1]
B = 1
kp = Poly(G_z_num.coeffs()[0], z)

# Numerator of strictly proper portion of transfer function
G_z_N_p = G_z_num - kp * G_z_den

# This is observable canonical form.
# Input: rear wheel torque
# Output: rear wheel rate
A = - G_z_den.coeffs()[1]
B = G_z_N_p.coeffs()[0]
C = 1
D = kp.coeffs()[0]


print(kp)
print(G_z_N_p)
print(G_z_den)

for n, m in [('A', A), ('B', B), ('C', C), ('D', D)]: print(n + "=" + str(m))

