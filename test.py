import sympy as sp

sp.var('c2 c4 c3 s2 s4 l2 l4')

exp1 = -c2*c4 - c3*s2*s4
exp2 = -c2*c4*l4 - c2*l2 - c3*l4*s2*s4

exp2.collect(exp1)

# even though exp2 contains exp1 (times a factor l4)
# the results shows false
result = exp2.has(exp1)
print (result)