import sympy as sp

t1, t2, t3, t4, t5 = sp.symbols("t1 t2 t3 t4 t5")
l1, l2, l3 = sp.symbols("l1 l2 l3")

T_01 = sp.Matrix([
    [sp.cos(t1), -sp.sin(t1), 0, 0],
    [sp.sin(t1), sp.cos(t1), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

T_12 = sp.Matrix([
    [sp.cos(t2), -sp.sin(t2), 0, 0],
    [0, 0, -1, 0],
    [sp.sin(t2), sp.cos(t2), 0, 0],
    [0, 0, 0, 1]
])

T_23 = sp.Matrix([
    [sp.cos(t3), -sp.sin(t3), 0, l1],
    [sp.sin(t3), sp.cos(t3), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_34 = sp.Matrix([
    [sp.cos(t4), -sp.sin(t4), 0, l2],
    [sp.sin(t4), sp.cos(t4), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_45 = sp.Matrix([
    [sp.cos(t5), -sp.sin(t5), 0, 0],
    [0, 0, -1, 0],
    [sp.sin(t5), sp.cos(t5), 0, 0],
    [0, 0, 0, 1]
])

T_56 = sp.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, l3],
    [0, 0, 0, 1]
])

T_06 = T_01*T_12*T_23*T_34*T_45*T_56

# sp.pretty_print(sp.simplify(T_06))

x = l1*sp.cos(t1) + l2*sp.cos(t1)*sp.cos(t2) + \
    l3*sp.cos(t1)*sp.cos(t2)*sp.cos(t3)

sp.pretty_print(sp.diff(x, t1))
