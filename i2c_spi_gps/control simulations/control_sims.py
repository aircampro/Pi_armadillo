#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Angle tracking control of vertical drive arm
#
from control.matlab import tf, feedback, step, bode, logspace, mag2db

g = 9.81                                                                # Gravitational acceleration [m/s^2]
l = 0.2                                                                 # Arm length [m]
M = 0.5                                                                 # Arm mass [kg]
mu = 1.5e-2                                                             # Viscous friction coefficient [kg*m^2/s]
J = 1.0e-2                                                              # Moment of inertia [kg*m^2]

P = tf( [0,1], [J, mu, M*g*l] )

ref = 30                                                                # target angle [deg]

from control.matlab import stepinfo
info = stepinfo(P, SettlingTimeThreshold=0.05)
info

from control.matlab import bode, logspace, mag2db
from scipy.signal import argrelmax

# Peak gain
mag, _, w = bode(P, logspace(-1,2,1000), plot=False)
print(f'Peak gain Mp = {mag2db(mag.max())} [dB]')

[maxId] = argrelmax(mag)
print('wp=', w[maxId])
print('Peak gain Mp=', mag2db(mag[maxId]))

# Bandwidth
print(f'Bandwidth wbw = {P.bandwidth()} [rad/s]')

kp = (0.5, 1, 2)                                                        # proportional gain

LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))
for i in range(len(kp)):
    K = tf([0, kp[i]], [0, 1])
    Gyr = feedback(P*K, 1)
    y,t = step(Gyr, np.arange(0, 2, 0.01))

    pltargs = {'ls': next(LS), 'label': f'$k_P$={kp[i]}'}
    ax.plot(t, y*ref, **pltargs)

ax.axhline(ref, color="k", linewidth=0.5)
plot_set(ax, 't', 'y', 'best')

ax.set_xlim(0, 2)
ax.set_ylim(0, 50)

fig.savefig("pcont.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

LS = linestyle_generator()
fig, ax = plt.subplots(2, 1, figsize=(4, 3.5))
for i in range(len(kp)):
    K = tf([0, kp[i]], [0, 1])
    Gyr = feedback(P*K, 1)
    mag, phase, w = bode(Gyr, logspace(-1,2,1000), plot=False)

    pltargs = {'ls': next(LS), 'label': f'$k_P$={kp[i]}'}
    ax[0].semilogx(w, mag2db(mag), **pltargs)
    ax[1].semilogx(w, np.rad2deg(phase), **pltargs)

bodeplot_set(ax, 'lower left')

ax[1].set_ylim(-190,10)
ax[1].set_yticks([-180,-90,0])

fig.tight_layout()
# fig.savefig("pcont_bode.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# show the response with various values of kd PD controller
#
kp = 2                                                               # prop band
kd = (0, 0.1, 0.2)                                                   # differntial gain

LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))
for i in range(len(kd)):
    K = tf([kd[i], kp], [0, 1])
    Gyr = feedback(P*K, 1)
    y,t = step(Gyr, np.arange(0, 2, 0.01))

    pltargs = {'ls': next(LS), 'label': f'$k_D$={kd[i]}'}
    ax.plot(t, y*ref, **pltargs)

ax.axhline(ref, color="k", linewidth=0.5)
plot_set(ax, 't', 'y', 'best')

ax.set_xlim(0, 2)
ax.set_ylim(0, 50)
fig.savefig("pdcont.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# bode diagram
#
LS = linestyle_generator()
fig, ax = plt.subplots(2, 1, figsize=(4, 3.5))

for i in range(len(kd)):
    K = tf([kd[i], kp], [0,1])
    Gyr = feedback(P*K, 1)
    mag, phase, w = bode(Gyr, logspace(-1,2,1000), plot=False)

    pltargs = {'ls': next(LS), 'label': f'$k_D$={kd[i]}'}
    ax[0].semilogx(w, mag2db(mag), **pltargs)
    ax[1].semilogx(w, np.rad2deg(phase), **pltargs)

bodeplot_set(ax, 'lower left')

ax[1].set_ylim(-190,10)
ax[1].set_yticks([-180,-90,0])

fig.tight_layout()
fig.savefig("pdcont_bode.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# PID
kp = 2                                                            # prop band
kd = 0.1                                                          # derivative
ki = (0, 5, 10)                                                   # integral term

LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))

for i in range(len(ki)):
    K = tf([kd, kp, ki[i]], [1, 0])
    Gyr = feedback(P*K, 1)
    y, t = step(Gyr, np.arange(0, 2, 0.01))

    pltargs = {'ls': next(LS), 'label': f'$k_I$={ki[i]}'}
    ax.plot(t, y*ref, **pltargs)

ax.axhline(ref, color="k", linewidth=0.5)
plot_set(ax, 't', 'y', 'upper left')

ax.set_xlim(0, 2)
ax.set_ylim(0,50)

fig.savefig("pidcont.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# bode plot
LS = linestyle_generator()
fig, ax = plt.subplots(2, 1, figsize=(4, 3.5))

for i in range(len(ki)):
    K = tf([kd, kp, ki[i]], [1, 0])
    Gyr = feedback(P*K,1)
    mag, phase, w = bode(Gyr, logspace(-1,2,1000), plot=False, wrap_phase = True)

    pltargs = {'ls': next(LS), 'label': f'$k_I$={ki[i]}'}
    ax[0].semilogx(w, mag2db(mag), **pltargs)
    ax[1].semilogx(w, np.rad2deg(phase), **pltargs)

bodeplot_set(ax, 'best')

ax[1].set_ylim(-190,10)
ax[1].set_yticks([-180,-90,0])

fig.tight_layout()
fig.savefig("pidcont_bode.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# Exercise (disturbance suppression)
LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))

for i in range(len(ki)):
    K = tf([kd, kp, ki[i]], [1, 0])
    Gyd = feedback(P, K)
    y, t = step(Gyd, np.arange(0, 2, 0.01))

    pltargs = {'ls': next(LS), 'label': f'$k_I$={ki[i]}'}
    ax.plot(t, y, **pltargs)

plot_set(ax, 't', 'y', 'center right')
ax.set_xlim(0, 2)
ax.set_ylim(-0.05, 0.5)

fig.savefig("pidcont_dis.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# bode plot
LS = linestyle_generator()
fig, ax = plt.subplots(2, 1, figsize=(4, 3.5))

for i in range(len(ki)):
    K = tf([kd, kp, ki[i]], [1,0])
    Gyd = feedback(P, K)
    mag, phase, w = bode(Gyd, logspace(-1,2,1000), plot=False, wrap_phase=True)

    pltargs = {'ls': next(LS), 'label': f'$k_I$={ki[i]}'}
    ax[0].semilogx(w, mag2db(mag), **pltargs)
    ax[1].semilogx(w, np.rad2deg(phase), **pltargs)

bodeplot_set(ax, 'best')

ax[1].set_ylim(-190,100)
ax[1].set_yticks([-180,-90, 0, 90])

fig.tight_layout()
fig.savefig("pidcont_dis_bode.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# Two-degree-of-freedom control
kp, ki, kd = 2, 10, 0.1
K1 = tf([kd, kp, ki], [1, 0])
K2 = tf([kp, ki], [kd, kp, ki]) # PI-D
K3 = tf([0, ki], [kd, kp, ki]) # I-PD

Gyz = feedback(P*K1, 1)

Td = np.arange(0, 2, 0.01)
r = 1*(Td>0)

z, t, _ = lsim(K2, r, Td, 0)

fig, ax = plt.subplots(1, 2, figsize=(6, 2.3))

y, _, _ = lsim(Gyz, r, Td, 0)
ax[0].plot(t, r*ref)
ax[1].plot(t, y*ref, ls='--', label='PID')

y, _, _ = lsim(Gyz, z, Td, 0)
ax[0].plot(t, z*ref)
ax[1].plot(t, y*ref, label='PI-D')

plot_set(ax[0], 't', 'r')
ax[0].set_xlim(0, 2)
ax[0].set_ylim(0,50)

ax[1].axhline(ref, color="k", linewidth=0.5)
plot_set(ax[1], 't', 'y', 'best')

ax[1].set_xlim(0, 2)
ax[1].set_ylim(0,50)

fig.tight_layout()
fig.savefig("2deg1.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

tau = 0.0000001 # ローパスフィルタ
Klp = tf([kd, 0], [tau, 1]) # 擬似微分器
Ktau = tf([kp, ki], [1, 0]) + Klp

Gyz = feedback(P*Ktau, 1)
Guz = Ktau/(1+P*Ktau)

Td = np.arange(0, 2, 0.01)
r = 1*(Td>0)

z, t, _ = lsim(K2, r, Td, 0)

fig, ax = plt.subplots(1, 2, figsize=(6, 2.3))

u, _, _ = lsim(Guz, r, Td, 0)
ax[0].plot(t, u, ls='--', label='PID')

u, _, _ = lsim(Guz, z, Td, 0)
ax[0].plot(t, u, label='PI-D')

y, _, _ = lsim(Gyz, r, Td, 0)
ax[1].plot(t, y*ref, ls='--', label='PID')

y, _, _ = lsim(Gyz, z, Td, 0)
ax[1].plot(t, y*ref, label='PI-D')

ax[0].set_xlim(0, 0.5)
ax[1].axhline(ref, color="k", linewidth=0.5)
plot_set(ax[0], 't', 'u', 'best')
plot_set(ax[1], 't', 'y', 'best')
ax[1].set_xlim(0, 2)
ax[1].set_ylim(0,50)

fig.tight_layout()
fig.savefig("2deg1_u.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

Gyz = feedback(P*K1, 1)

Td = np.arange(0, 2, 0.01)
r = 1*(Td>0)

z, t, _ = lsim(K3, r, Td, 0)

fig, ax = plt.subplots(1, 2, figsize=(6, 2.3))

y, _, _ = lsim(Gyz, r, Td, 0)
ax[0].plot(t, r*ref)
ax[1].plot(t, y*ref, ls='--', label='PID')

y, _, _ = lsim(Gyz, z, Td, 0)
ax[0].plot(t, z*ref)
ax[1].plot(t, y*ref, label='I-PD')

plot_set(ax[0], 't', 'r')
ax[0].set_xlim(0, 2)
ax[0].set_ylim(0,50)

ax[1].axhline(ref, linewidth=0.5)
plot_set(ax[1], 't', 'y', 'best')

ax[1].set_xlim(0, 2)
ax[1].set_ylim(0,50)

fig.tight_layout()
fig.savefig("2deg2.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

tau = 0.0000001                                                     # low pass filter
Klp = tf([kd, 0], [tau, 1])                                         # Pseudo Differentiator
Ktau = tf([kp, ki], [1, 0]) + Klp

Gyz = feedback(P*Ktau, 1)
Guz = Ktau/(1+P*Ktau)

Td = np.arange(0, 2, 0.01)
r = 1*(Td>0)

z, t, _ = lsim(K3, r, Td, 0)

fig, ax = plt.subplots(1, 2, figsize=(6, 2.3))

u, _, _ = lsim(Guz, r, Td, 0)
ax[0].plot(t, u, ls='--', label='PID')

u, _, _ = lsim(Guz, z, Td, 0)
ax[0].plot(t, u, label='I-PD')

y, _, _ = lsim(Gyz, r, Td, 0)
ax[1].plot(t, y*ref, ls='--', label='PID')

y, _, _ = lsim(Gyz, z, Td, 0)
ax[1].plot(t, y*ref, label='I-PD')

ax[0].set_xlim(0, 0.5)

ax[1].axhline(ref, color="k", linewidth=0.5)
plot_set(ax[0], 't', 'u', 'best')
plot_set(ax[1], 't', 'y', 'best')
ax[1].set_xlim(0, 2)
ax[1].set_ylim(0,50)


fig.tight_layout()
fig.savefig("2deg2_u.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

from control.matlab import tf, pade, rlocus, poles, feedback, step
g = 9.81 # Gravitational acceleration [m/s^2]
l = 0.2 # Arm length [m]
M = 0.5 # Arm mass [kg]
mu = 1.5e-2 # Viscous friction coefficient [kg*m^2/s]
J = 1.0e-2 # Moment of inertia [kg*m^2]

P = tf( [0,1], [J, mu, M*g*l] )

ref = 30 # traget angle [deg]

# num_delay, den_delay = pade( 0.005, 5)
num_delay, den_delay = pade( 0.005, 1) # First-order Pade approximation
Pdelay = P * tf(num_delay, den_delay)
Pdelay
print(poles(Pdelay))

kvect = np.arange(0, 5, 0.001)
rlist, klist = rlocus(Pdelay, kvect, plot=False)
fig, ax = plt.subplots(figsize=(3,3))

ax.plot(rlist.real, rlist.imag)
ax.set_xlim(-3, 1)
ax.grid(ls=':')

rlist, klist = rlocus(P, kvect, plot=False)
fig, ax = plt.subplots(figsize=(3,3))

ax.plot(rlist.real, rlist.imag)
ax.set_xlim(-3, 1)
ax.grid(ls=':')

kp0 = 2.9                                     # Proportional gain
K = tf([0, kp0], [0, 1])                      # P control
Gyr = feedback(Pdelay*K, 1)                   # Closed loop system
y,t = step(Gyr, np.arange(0, 2, 0.01))        # Step response

fig, ax = plt.subplots(figsize=(3, 2.3))
ax.plot(t, y*ref)

ax.axhline(ref, color='k', linewidth=0.5)
ax.set_xlim(0, 2)
ax.set_ylim(0, 50)
plot_set(ax, 't', 'y')

fig.savefig("tune_zn.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

kp = [0, 0]
ki = [0, 0]
kd = [0, 0]
Rule = ['', '']

T0 = 0.3                                             # period
# Classic ZN
Rule[0] = 'Classic'
kp[0] = 0.6 * kp0
ki[0] = kp[0] / (0.5 * T0)
kd[0] = kp[0] * (0.125 * T0)

# No overshoot
Rule[1] = 'No Overshoot'
kp[1] = 0.2 * kp0
ki[1] = kp[1] / (0.5 * T0)
kd[1] = kp[1] * (0.33 * T0)

LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))

for i in range(2):
    K = tf([kd[i], kp[i], ki[i]], [1, 0])
    Gyr = feedback(Pdelay*K, 1)
    y, t = step(Gyr, np.arange(0, 2, 0.01))

    ax.plot(t, y*ref, ls=next(LS), label=Rule[i])

    print(Rule[i])
    print('kP=', kp[i])
    print('kI=', ki[i])
    print('kD=', kd[i])
    print('------------------')

ax.axhline(ref, color="k", linewidth=0.5)
ax.set_xlim(0, 2)
ax.set_ylim(0, 50)
plot_set(ax, 't', 'y', 'best')

fig.savefig("tune_zn_result.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# model matching
import sympy as sp
s = sp.Symbol('s')
kp, kd, ki = sp.symbols('k_p k_d k_i')
Mgl, mu, J = sp.symbols('Mgl mu J')
sp.init_printing()

G = (kp*s+ki)/(J*s**3 +(mu+kd)*s**2 + (Mgl + kp)*s + ki)
sp.series(1/G, s, 0, 4)

import sympy as sp
z, wn = sp.symbols('zeta omega_n')
kp, kd, ki = sp.symbols('k_p k_d k_i')
Mgl,mu,J = sp.symbols('Mgl mu J')
sp.init_printing()

f1 = Mgl/ki-2*z/wn
f2 = (mu+kd)/ki-Mgl*kp/(ki**2)-1/(wn**2)
f3 = J/ki-kp*(mu+kd)/(ki**2)+Mgl*kp**2/(ki**3)
sp.solve([f1, f2, f3],[kp, kd, ki])

g = 9.81                  # Gravitational acceleration [m/s^2]
l = 0.2                   # Arm length [m]
M = 0.5                   # Arm mass [kg]
mu = 1.5e-2               # Viscous friction coefficient
J = 1.0e-2                # Moment of inertia

P = tf( [0,1], [J, mu, M*g*l] )

ref = 30 # traget angle [deg]

omega_n = 15
zeta = (1, 1/np.sqrt(2))
Label = ('Binomial coeff.', 'Butterworth')

LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))

for i in range(2):
    Msys = tf([0,omega_n**2], [1,2*zeta[i]*omega_n,omega_n**2])
    y, t = step(Msys, np.arange(0, 2, 0.01))

    ax.plot(t, y*ref, ls=next(LS), label=Label[i])

ax.set_xlim(0, 2)
ax.set_ylim(0, 50)
plot_set(ax, 't', 'y', 'best')

fig.savefig("ref_model_2nd.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

omega_n = 15
zeta = 0.707

Msys = tf([0,omega_n**2], [1,2*zeta*omega_n,omega_n**2])

kp = omega_n**2*J
ki = omega_n*M*g*l/(2*zeta)
kd = 2*zeta*omega_n*J + M*g*l/(2*zeta*omega_n) - mu

print('kP=', kp)
print('kI=', ki)
print('kD=', kd)

Gyr = tf([kp,ki], [J, mu+kd, M*g*l+kp, ki])

yM, tM = step(Msys, np.arange(0, 2, 0.01))
y, t = step(Gyr, np.arange(0, 2, 0.01))

fig, ax = plt.subplots(figsize=(3, 2.3))

ax.plot(tM, yM*ref, label='M', ls = '-.')
ax.plot(t, y*ref, label='Gyr')

ax.set_xlim(0, 2)
ax.set_ylim(0, 50)
plot_set(ax, 't', 'y', 'best')

fig.savefig("model_match.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

alpha1 = (3, 2, 2.15)
alpha2 = (3, 2, 1.75)
omega_n = 15
Label = ('Binomial coeff.', 'Butterworth', 'ITAE')

LS = linestyle_generator()
fig, ax = plt.subplots(figsize=(3, 2.3))

for i in range(3):
    Msys = tf([0, omega_n**3], [1, alpha2[i]*omega_n, alpha1[i]*omega_n**2, omega_n**3])
    y,t = step(Msys, np.arange(0, 2, 0.01))

    ax.plot(t, y*ref, ls=next(LS), label=Label[i])

ax.set_xlim(0, 2)
ax.set_ylim(0, 50)
plot_set(ax, 't', 'y', 'best')

fig.savefig("ref_model_3rd.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# optimal regulator
Q = np.diag( [100, 1] )
R = 1

F, X, E = lqr(P.A, P.B, Q, R)
F = -F
FF = - (1/R)*(P.B.T)@X
X, E, F = care(P.A, P.B, Q, R)
F = -F
print('--- Feedback gain ---')
print(F)
print('--- Closed loop poles ---')
print(E)

Acl = P.A + P.B@F
Pfb = ss(Acl, P.B, P.C, P.D)

tdata = np.arange(0, 5, 0.01)
xini, tini = initial(Pfb, tdata, [-0.3, 0.4]) #ゼロ入力応答

fig, ax = plt.subplots(figsize=(3, 2.3))

ax.plot(tini, xini[:,0], label = '$x_1$')
ax.plot(tini, xini[:,1], ls = '-.', label = '$x_2$')

ax.set_xlabel('t')
ax.set_ylabel('x')
ax.grid(ls=':')
ax.legend()

fig.savefig("sf_lqr.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# Circle condition (robustness of optimal regulator)

from control.matlab import ss, nyquist, logspace

# A = '0 1; -4 5'
# B = '0; 1'
# C = '1 0 ; 0 1'
# D = '0; 0'

A = [[0, 1], [-4, 5]]
B = [[0], [1]]
C = [[1, 0], [0, 1]]
D = [[0], [0]]
P = ss(A, B, C, D)
L = ss(P.A, P.B, -F, 0)

print(L)

import matplotlib.patches as patches

fig, ax = plt.subplots(figsize=(3, 3))
x, y, w = nyquist(L, logspace(-2,3,1000), plot=False)
ax.plot(x, y)
ax.plot(x, -y, ls='--')
ax.scatter(-1, 0, color='k')

ax.grid(ls=':')
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)

c = patches.Circle(xy=(-1, 0), radius=1, fill=False, ec='k')
ax.add_patch(c)

fig.tight_layout()

#The Nyquist locus of the open loop system does not fall within the unit circle centered at (-1, 0j)

#This ensures that the phase margin is at least 60 [deg]

#Hamiltonian matrix
#The closed loop poles when using the optimal regulator are equal to the stable eigenvalues ​​of the Hamiltonian matrix

# H1 = np.c_[P.A, -P.B*(1/R)*P.B.T]
# H2 = np.c_[ Q, P.A.T]
# H = np.r_[H1, -H2]

H = np.block( [ [P.A, -P.B*(1/R)@P.B.T], [-Q, -P.A.T]])
eigH = np.linalg.eigvals(H)
print(eigH)

print('--- Stable eigenvalues ​​of Hamiltonian matrix ---')
eigH_stable = [ i for i in eigH if i < 0]
print(eigH_stable)
F = -acker(P.A, P.B, eigH_stable)
print('--- Feedback gain ---')
print(F)

from control.matlab import ss, acker, lsim
# A = '0 1; -4 5'
# B = '0; 1'
# C = '1 0 ; 0 1'
# D = '0; 0'

A = [[0, 1], [-4, 5]]
B = [[0], [1]]
C = [[1, 0], [0, 1]]
D = [[0], [0]]
P = ss(A, B, C, D)
print(P)

Pole = [-1, -1]
F = -acker(P.A, P.B, Pole)

Acl = P.A + P.B@F
Pfb = ss(Acl, P.B, P.C, P.D)

Td = np.arange(0, 8, 0.01)
Ud = 0.2 * (Td>=0)
x, t, _ = lsim(Pfb, Ud, Td, [-0.3, 0.4])

fig, ax = plt.subplots(figsize=(3, 2.3))

ax.plot(t, x[:,0], label = '$x_1$')
ax.plot(t, x[:,1], ls = '-.', label = '$x_2$')
plot_set(ax, 't', 'x', 'best')

fig.savefig("sf_dis.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# A = '0 1; -4 5'
# B = '0; 1'
# C = '1 0'
# D = '0'

A = [[0, 1], [-4, 5]]
B = [[0], [1]]
C = [1, 0]
D = [0]
P = ss(A, B, C, D)
print(P)

# Expansion system
Abar = np.block([ [P.A, np.zeros((2,1))], [-P.C, 0] ])
Bbar = np.block([ [P.B], [0] ])
Cbar = np.block([ P.C, 0 ])

# Abar = np.r_[ np.c_[P.A, np.zeros((2,1))], -np.c_[ P.C, 0 ] ]
# Bbar = np.c_[ P.B.T, 0 ].T
# Cbar = np.c_[ P.C, 0 ]

Pole = [-1, -1, -5]
F = -acker(Abar, Bbar, Pole)

# Acl = P.A + P.B*F[0,0:2]
Acl = Abar + Bbar@F
Pfb = ss(Acl, Bbar, np.eye(3), np.zeros((3,1)))

Td = np.arange(0, 8, 0.01)
Ud = 0.2 * (Td>=0)
x, t, _ = lsim(Pfb, Ud, Td, [-0.3, 0.4, 0])

fig, ax = plt.subplots(figsize=(3, 2.3))
ax.plot(t, x[:,0], label = '$x_1$')
ax.plot(t, x[:,1], ls = '-.',label = '$x_2$')
# ax.plot(t, Ud, c='k')
plot_set(ax, 't', 'x', 'best')

fig.savefig("servo.pdf", transparent=True, bbox_inches="tight", pad_inches=0.0)

# Controllability and observability

from control.matlab import ss, ctrb, obsv

# A = '0 1; -4 5'
# B = '0; 1'
# C = '1 0'

A = [[0, 1], [-4, 5]]
B = [[0], [1]]
C = [1, 0]
P = ss(A, B, C, 0)
print(P)

# If the controllability matrix is ​​regular, the system is controllable.
Uc = ctrb(P.A, P.B)
print('Uc=\n',Uc)
print('det(Uc)=', np.linalg.det(Uc))
print('rank(Uc)=', np.linalg.matrix_rank(Uc))

# Observability matrix is ​​calculated, and if it is regular, the system is observable.
Uo = obsv(P.A, P.C)
print('Uo=\n', Uo)
print('det(Uo)=', np.linalg.det(Uo))
print('rank(Uo)=', np.linalg.matrix_rank(Uo))




















