#!/usr/bin/env python3

import sympy as sp

t1, t2, t3, t4, t5 = sp.symbols(
    "thetas[0] thetas[1] thetas[2] thetas[3] thetas[4]")
l1, l2, l3 = sp.symbols("206.16 200 -174.15")


def fk_transform(alpha, a, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a],
        [sp.sin(theta)*sp.cos(alpha), sp.cos(theta) *
         sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha)*d],
        [sp.sin(theta)*sp.sin(alpha), sp.cos(theta) *
         sp.sin(alpha), sp.cos(alpha), sp.cos(alpha)*d],
        [0, 0, 0, 1]
    ])


dh = [
    [0, 0, 0, t1 - sp.pi],
    [sp.pi/2, 0, 0, t2 - sp.pi/2 - sp.sin(50/206.16)],
    [0, l1, 0, t3 - 3*sp.pi/2 + sp.sin(50/206.16)],
    [0, l2, 0, t4 - 3*sp.pi/2],
    [sp.pi/2, 0, 0, t5 - sp.pi],
    [0, 0, l3, 0]
]

# Build tool to base transform
T = fk_transform(dh[0][0], dh[0][1], dh[0][2], dh[0][3])
for i in range(1, len(dh)):
    T = T @ fk_transform(dh[i][0], dh[i][1], dh[i][2], dh[i][3])

T = sp.simplify(T)
print("T from 0 to 6")
print(T)


# Build pose (x, y, z, roll, pitch, yaw)
xyz = T[0:3, 3]
roll_pitch = sp.Matrix([[t5], [t2 + t3 + t4]])
pose = sp.Matrix.vstack(xyz, roll_pitch)

J = sp.simplify(pose.jacobian(sp.Matrix([t1, t2, t3, t4, t5])))
print("J")
print(J)
