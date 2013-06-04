"""
Given a continuous time first order transfer function of the form:

    n1 * s + n0
    -----------
      s + d0

Compute the Tustin approximation and return a state space realization of this
discrete time transfer function.
"""
from sympy import symbols, Poly, ccode, S, sqrt

def discrete_realization(n0, n1, d0, T):
    z = symbols('z')
    s = 2/T*(z-1)/(z+1)
    num = ((n1*s + n0)*T*(z + 1)).simplify()
    den = ((s + d0)*T*(z + 1)).simplify()
    num_poly = Poly(num, z)
    den_poly = Poly(den, z)

    n1_z, n0_z = num_poly.coeffs()
    d1_z, d0_z = den_poly.coeffs()

    # Make denominator monic and divide numerator appropriately
    n1_z /= d1_z
    n0_z /= d1_z
    d0_z /= d1_z

    a = -d0_z
    b_times_c = (n0_z - n1_z * d0_z).simplify()
    d = n1_z

    return a, b_times_c, d


n0, n1, d0, T = symbols('n0 n1 d0 T')
#T = 0.0013
#n0 = 1.23
#n1 = 4.56
#d0 = 7.89
a, b_times_c, d = discrete_realization(n0, n1, d0, T)

a_str = ccode(a)
b_times_c_str = ccode(b_times_c)
d_str = ccode(d)

print(a_str)
print(b_times_c_str)
print(d_str)

