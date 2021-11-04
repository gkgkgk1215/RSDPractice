import sympy as sym
from sympy import sin, cos, simplify, pprint


def DH_transform(alpha, a, d, theta):
    T = sym.Matrix([[cos(theta), -sin(theta), 0, a],
                    [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                    [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d],
                    [0, 0, 0, 1]])
    return T


L1, L2, L3, L4, L5, offset, Lg, Lt = sym.symbols('L1, L2, L3, L4, L5, offset, Lg, Lt')
q1, q2, q3, q4, q5, q6, q7 = sym.symbols('q1, q2, q3, q4, q5, q6, q7')
T01 = DH_transform(0,               0,    L1,        q1)
T12 = DH_transform(-sym.pi/2,       0,     0,        q2)
T23 = DH_transform(sym.pi/2,        0,    L2,        q3)
T34 = DH_transform(sym.pi/2,   offset,     0,        q4)
T45 = DH_transform(-sym.pi/2, -offset,    L3,        q5)
T56 = DH_transform(sym.pi/2,        0,     0,        q6)
T67 = DH_transform(sym.pi/2,       L4,     0,        q7)
T7f = DH_transform(0,               0,    L5,         0)
Tfe = DH_transform(0,               0, Lg+Lt, -sym.pi/4)

T02 = T01*T12
T03 = T02*T23
T04 = T03*T34
T05 = T04*T45
T06 = T05*T56
T07 = T06*T67
T0f = T07*T7f
T0e = T0f*Tfe

# Position
px = T0e[0, 3]
py = T0e[1, 3]
pz = T0e[2, 3]
# px = T0f[0, 3]
# py = T0f[1, 3]
# pz = T0f[2, 3]

J = sym.zeros(6, 7)
J[0, 0] = sym.diff(px, q1)
J[0, 1] = sym.diff(px, q2)
J[0, 2] = sym.diff(px, q3)
J[0, 3] = sym.diff(px, q4)
J[0, 4] = sym.diff(px, q5)
J[0, 5] = sym.diff(px, q6)
J[0, 6] = sym.diff(px, q7)

J[1, 0] = sym.diff(py, q1)
J[1, 1] = sym.diff(py, q2)
J[1, 2] = sym.diff(py, q3)
J[1, 3] = sym.diff(py, q4)
J[1, 4] = sym.diff(py, q5)
J[1, 5] = sym.diff(py, q6)
J[1, 6] = sym.diff(py, q7)

J[2, 0] = sym.diff(pz, q1)
J[2, 1] = sym.diff(pz, q2)
J[2, 2] = sym.diff(pz, q3)
J[2, 3] = sym.diff(pz, q4)
J[2, 4] = sym.diff(pz, q5)
J[2, 5] = sym.diff(pz, q6)
J[2, 6] = sym.diff(pz, q7)

J[3:, 0] = T01[:3, 2]
J[3:, 1] = T02[:3, 2]
J[3:, 2] = T03[:3, 2]
J[3:, 3] = T04[:3, 2]
J[3:, 4] = T05[:3, 2]
J[3:, 5] = T06[:3, 2]
J[3:, 6] = T07[:3, 2]

print (J.subs(([(q1, 0), (q2, 0), (q3, 0), (q4, 0), (q5, 0), (q6, 0), (q7, 0),
                (L1, 0.333), (L2, 0.316), (L3, 0.384), (L4, 0.088), (L5, 0.107), (Lg, 0.0584), (Lt, 0.0), (offset, 0.0825)])))

for i in range(6):
    for j in range(7):
        print ("J[",i,", ",j,"] = ", J[i, j])