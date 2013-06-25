from sympy import *

k0, k1, k2, k3 = symbols('k:4')
a20, a21, a22, a23 = symbols('a20 a21 a22 a23')
a30, a31, a32, a33 = symbols('a30 a31 a32 a33')
b20, b30 = symbols('b20 b30')
s = Symbol('s')

def gain_and_zero(tf):
    gain = tf.subs({s:0})
    num, den = tf.as_numer_denom()
    n0 = num.subs({s:0}).expand()
    n1 = num.diff(s).expand()
    zero = (-n0 / n1).expand()
    if (n1 != 0):
        assert(tf.subs({s:zero}).expand() == 0)
    return gain, zero

M = Matrix([[k0, k1, k2, k3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
A = Matrix([[0, 0, 1, 0], [0, 0, 0, 1], [a20, a21, a22, a23], [a30, a31, a32, a33]])
B = Matrix([0, 0, b20, b30])

Minv = M.inv()
MAMinv = M*A*Minv
MB = M*B

A_obs = MAMinv[0, 0]
B_obs = MAMinv[0, 1:4].row_join(MB[0, :])
C_obs = Minv[0, 0:1]
D_obs = Minv[0, 1:4].row_join(zeros((1, 1)))


delta_to_phi = ((C_obs * 1/(s - A_obs) * B_obs[0, 0])[0, 0] + D_obs[0,
    0]).simplify()
phi_dot_to_phi = ((C_obs * 1/(s - A_obs) * B_obs[0, 1])[0, 0] + D_obs[0,
    1]).simplify()
delta_dot_to_phi = ((C_obs * 1/(s - A_obs) * B_obs[0, 2])[0, 0] + D_obs[0,
    2]).simplify()
T_delta_to_phi = ((C_obs * 1/(s - A_obs) * B_obs[0, 3])[0, 0] + D_obs[0,
    3]).simplify()

print(delta_to_phi)
print(phi_dot_to_phi)
print(delta_dot_to_phi)
print(T_delta_to_phi)

delta_to_phi_k, delta_to_phi_zero = gain_and_zero(delta_to_phi)
phi_dot_to_phi_k, phi_dot_to_phi_zero = gain_and_zero(phi_dot_to_phi)
delta_dot_to_phi_k, delta_dot_to_phi_zero = gain_and_zero(delta_dot_to_phi)
T_delta_to_phi_k, T_delta_to_phi_zero = gain_and_zero(T_delta_to_phi)

print(delta_to_phi_k)
print(delta_to_phi_zero)
print(phi_dot_to_phi_k)
print(phi_dot_to_phi_zero)
print(delta_dot_to_phi_k)
print(delta_dot_to_phi_zero)
print(T_delta_to_phi_k)
print(T_delta_to_phi_zero)

stop
print(M)
#print(latex(M.inv()))
print((M.inv()))

#print(latex(A))
#print(latex(B))

print(A_)
print(B_)

N = symbols('N')
eqn = A_[0, 1] / A_[0, 3] - N
print(eqn)
k1_soln = solve(eqn, k1)[0]
print("k1 = ")
print(k1_soln)
print(ccode(k1_soln))

A0_latex = latex(A_[0, :])
B0_latex = latex(B_[0, :])

#print(A0_latex)
#print(B0_latex)

# Observer transfer functions
# 4 inputs:
# - steer angle
# - roll rate
# - steer rate
# - steer torque


