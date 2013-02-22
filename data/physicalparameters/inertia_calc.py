from sympy.physics.mechanics import ReferenceFrame, inertia, dot
from sympy import symbols, trigsimp, solve, simplify, collect

alpha = symbols('alpha')

# Measured/estimated quantities
IAxx, IAyy, IAzz, IAxy, IAyz, IAxz = symbols('IAxx IAyy IAzz IAxy IAyz IAxz')
IBxx, IByy, IBzz, IBxy, IByz, IBxz = symbols('IBxx IByy IBzz IBxy IByz IBxz')
d_, e_, f_ = symbols('d_ e_ f_')
mA, mB = symbols('mA mB')

# Calculated quantites based on inertial considerations
IGxx, IGyy, IGzz, IGxy, IGyz, IGxz = symbols('IGxx IGyy IGzz IGxy IGyz IGxz')
d, e, f = symbols('d e f')
beta, gamma = symbols('beta gamma')
mT = symbols('mT')

# A symbol dictionary to hold the relationships between meausured quantities and
# calculated quantities

symbol_dict = {}

N = ReferenceFrame('N')
A = N.orientnew('A', 'Axis', [alpha, N.y])

I_A_AO = inertia(A, IAxx, IAyy, IAzz, IAxy, IAyz, IAxz)
#I_B_BO = inertia(A, IBxx, IByy, IBxx, 0, 0, 0)
I_B_BO = inertia(A, IBxx, IByy, IBzz, IBxy, IByz, IBxz)

# d_, e_, f_ are what was measured for both fork and frame, it is the mass
# center location of the fork and frame relative to their respective wheel
# centers
r_BO_AO = d_*A.x + e_*A.y + f_*A.z

print("N.z * I^{A/AO} * N.z = " + str(dot(N.z, dot(I_A_AO, N.z)).expand()))
print("N.z * I^{B/BO} * N.z = " + str(trigsimp(dot(N.z, dot(I_B_BO, N.z)).expand())))

# Given I^{A/AO}, I^{B/BO}, mA, mB, r^{AO/BO}, compute:
# - mT
# - r^{GO/BO}
# - I^{G/GO}
# where G is the gyrostat comprised of body A & B, and GO is the mass center
# Accumulate all these symbols in a symbol dictionary

symbol_dict[mT] = mA + mB
symbol_dict[beta] = mA / mT
# position from BO to GO (a.k.a. GO relative to BO)
r_BO_GO = beta*r_BO_AO
symbol_dict[d] = dot(r_BO_GO, A.x)
symbol_dict[e] = dot(r_BO_GO, A.y)
symbol_dict[f] = dot(r_BO_GO, A.z)

#equations = [lhs - symbol_dict[lhs] for lhs in [d, e, f]]
#soln = solve(equations, [d_, e_, f_])
#print "soln = ", soln

# Inertia of fictious particle of mass mA located at point AO about point GO
#I_AO_GO = mA*(mB/mT)**2*(inertia(A, 1, 1, 1)*dot(r_BO_AO, r_BO_AO) - (r_BO_AO|r_BO_AO))
# Inertia of fictious particle of mass mB located at point BO about point GO
#I_AO_GO = mB*(mA/mT)**2*(inertia(A, 1, 1, 1)*dot(r_BO_AO, r_BO_AO) - (r_BO_AO|r_BO_AO))

I_G_GO = I_A_AO + I_B_BO + gamma*(inertia(A, 1, 1, 1)*dot(r_BO_AO, r_BO_AO) - (r_BO_AO|r_BO_AO))

symbol_dict[gamma] = mB*beta
symbol_dict[IGxx] = collect(dot(A.x, dot(I_G_GO, A.x)).expand(), gamma)
symbol_dict[IGyy] = collect(dot(A.y, dot(I_G_GO, A.y)).expand(), gamma)
symbol_dict[IGzz] = collect(dot(A.z, dot(I_G_GO, A.z)).expand(), gamma)
symbol_dict[IGxy] = collect(dot(A.x, dot(I_G_GO, A.y)).expand(), gamma)
symbol_dict[IGyz] = collect(dot(A.y, dot(I_G_GO, A.z)).expand(), gamma)
symbol_dict[IGxz] = collect(dot(A.x, dot(I_G_GO, A.z)).expand(), gamma)

#print symbol_dict
compute_order = [mT, beta, d, e, f, gamma, IGxx, IGyy, IGzz, IGxy, IGyz, IGxz]
for lhs in compute_order:
    print("{0} = {1};".format(lhs, symbol_dict[lhs]))

