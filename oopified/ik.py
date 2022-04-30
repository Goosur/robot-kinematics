import sympy as sp

x = sp.symbols('x')
a = sp.Integral(sp.cos(x)*sp.exp(x), x)
print(sp.Eq(a, a.doit()))
