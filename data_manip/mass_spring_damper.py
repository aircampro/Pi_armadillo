# !/usr/bin/python
# coding:utf-8
# mass spring damper
#
import sympy as sym
import sympy.physics.mechanics as me

x, v = me.dynamicsymbols('x v')
m, c, k, g, t, f = sym.symbols('m c k g t f')

# Creating a coordinate system
ceiling = me.ReferenceFrame('C')

o = me.Point('o') # Ceiling point
p = me.Point('p') # point mass
o.set_vel(ceiling, 0)
p.set_pos(o, x * ceiling.x)
p.set_vel(ceiling, v * ceiling.x)

# Calculate the external force acting on a mass point
damping = -c * p.vel(ceiling)          
stiffness = -k * p.pos_from(o)        
gravity = m * g * ceiling.x           
exf = f * ceiling.x                   
forces = damping + stiffness + gravity + exf
print(forces)
mass = me.Particle('mass', p, m)
MM = [mass]
kane = me.KanesMethod(ceiling, q_ind=[x], u_ind=[v], kd_eqs=[v - x.diff(t)])
kane.kanes_equations(MM, [(p, forces)])
M = kane.mass_matrix_full
f = kane.forcing_full
print(M)
print(f)
print(M.inv() * f)