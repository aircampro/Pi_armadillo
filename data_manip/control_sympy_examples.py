# !/usr/bin/env python
#
# examples to use the control library
#
# conda install -c conda-forge control slycot
#
# sudo apt-get install python-pip
# sudo apt-get install gfortran
# sudo apt-get install libblas-dev libatlas-dev liblapack-dev
# pip install control
# pip install slycot
#
from control.matlab import tf, tfdata, step, bode, evalfr
import sympy as sp
import numpy as np                           
from matplotlib import pyplot as plt         
import control
import math
import cmath

#       1
# -------------
# s^2 + 2 s + 3
#
Np = [0, 1]      # (0*s + 1)
Dp = [1, 2, 3]   # (1*s^2 + 2*s + 3)
P = tf(Np, Dp)
print('P(s)=', P)

#     s + 2
# -------------
# s^3 + 5s^2 + 3s + 4
#
P = tf([1, 2], [1, 5, 3, 4])
print('P(s)=', P)

#     s + 3
# -------------
# s^3 + 5s^2 + 8s + 4
#
P1 = tf([1, 3], [0, 1]) # (s+3)/1
P2 = tf([0, 1], [1, 1]) # 1/(s+1)
P3 = tf([0, 1], [1, 2]) # 1/(s+2)
P = P1 * P2 * P3**2
print('P(s)=', P)
print(P.num)            # [1,3]
print(P.den)            # [1,5,8,4]

s = sp.Symbol('s')
t = sp.Symbol('t', positive=True)
sp.init_printing()
sp.laplace_transform(1, t, s)
sp.inverse_laplace_transform(1/s, s, t)

sp.laplace_transform(t, t, s)
sp.inverse_laplace_transform(1/s**2, s, t)

a = sp.Symbol('a', real=True)
sp.laplace_transform(sp.exp(-a*t), t, s)

w = sp.Symbol('w', real=True)
sp.laplace_transform(sp.sin(w*t), t, s)

sp.laplace_transform(sp.cos(w*t), t, s)

sp.laplace_transform(sp.exp(-a*t)*sp.sin(w*t), t, s)

sp.laplace_transform(sp.exp(-a*t)*sp.cos(w*t), t, s)

# plot bode diagram
sys1 = tf([0, 0, 1], [0.1, 0.1, 1.0]) 
y1, t1 = step(sys1, T = np.arange(0, 10, 0.01))  # （t=0～10）
plt.plot(t1, y1)                                       
bd = bode(sys1)

T_samp = 0.01
w_max = 50 * 2*np.pi
w_min = 0.01
tf = tf([1],[1 , 0])
fig = plt.figure()
bode(tf) 
tf_d = control.c2d(tf,T_samp,method='zoh')     
print(tf_d)
fig2 = plt.figure()
bode(tf_d)
w = np.linspace(w_min,w_max,10000000)
fr = evalfr(tf_d,np.exp(1j*w*T_samp))                  #s = exp(jwT)　
gain = abs(fr)
phase = []
for i in fr:
    phase.append(cmath.phase(i))   
phase = np.array(phase)  *180 / np.pi                    
fig = plt.figure()

# top
ax1 = fig.add_subplot(2, 1, 1)
ax1.plot(w, gain)
ax1.set_xlabel("w[rad/s]")
ax1.set_ylabel("gain")
ax1.set_yscale('log')
ax1.set_xscale('log')
ax1.set_xlim(w_min,w_max)

# bottom
ax2 = fig.add_subplot(2, 1, 2)
ax2.plot(w, phase)
ax2.set_xlabel("w[rad/s]")
ax2.set_ylabel("phase")
ax2.set_xlim(w_min,w_max)
ax2.set_xscale('log')
ax2.set_ylim(-190,190)

# show plots
fig.tight_layout()

# low pass filter
cut_of_freqency = 10                       #Hz   : cut off freq
T_samp = 0.01                              #s    : period
tf = tf([1],[1/(cut_of_freqency*2*np.pi),1])
fig = plt.figure()
bode(tf)  
tf_d = control.c2d(tf,T_samp,method='zoh')     
print(tf_d)
fig2 = plt.figure()
bode(tf_d)
print(tf_d.den)
print(tf_d.num

# quadratic
cut_of_freqency = 1/2/np.pi                #Hz   : cut off freq
T_samp = 0.01                              #s    : period
rad = cut_of_freqency*2*np.pi
tf = tf([1],[1/(rad**2),np.sqrt(2)/rad,1])
fig = plt.figure()
bode(tf)           
tf_d = control.c2d(tf,T_samp,method='zoh')     
print(tf_d)
fig2 = plt.figure()
bode(tf_d)
print(tf_d.den)
print(tf_d.num)

