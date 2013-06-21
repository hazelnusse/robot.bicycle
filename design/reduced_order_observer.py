from sympy import *


k0, k1, k2, k3 = symbols('k:4')
a20, a21, a22, a23 = symbols('a20 a21 a22 a23')
a30, a31, a32, a33 = symbols('a30 a31 a32 a33')
b20, b30 = symbols('b20 b30')


M = Matrix([[k0, k1, k2, k3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

print(M)
#print(latex(M.inv()))
print((M.inv()))

A = Matrix([[0, 0, 1, 0], [0, 0, 0, 1], [a20, a21, a22, a23], [a30, a31, a32, a33]])
#print(latex(A))
B = Matrix([0, 0, b20, b30])
#print(latex(B))


A_ = M*A*M.inv()
B_ = M*B
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



