#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Example of Bode and Nyquist plots using control library
#
import control
from control import matlab
import matplotlib.pyplot as plt

tf = matlab.tf([1],[1 , 0])

A=[[1 , 0],
   [1 , 2]]
B=[[1],
   [0]]
C=[[1 , 0]]
D=[[0]]

c_sys = matlab.ss(A,B,C,D)
tf2 = control.ss2tf(c_sys)
c_sys2 = control.tf2ss(tf2)

A=c_sys.A
B=c_sys.B
C=c_sys.C
D=c_sys.D

np.linalg.eig(A)

matlab.bode(c_sys)

fig = plt.figure()
matlab.bode(c_sys)

fig2 = plt.figure()
matlab.nyquist(c_sys)

tf_d = control.c2d(tf,0.01,method='zoh')