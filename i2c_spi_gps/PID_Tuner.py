#!/usr/bin/env python
#
# Data-driven control with PythonPID controller auto-tuning with VRFT
#
# ref:- https://qiita.com/KariControl/items/bfbfb5b9ac7492df938f
#
# Data-driven control program: PID adjustment
# 
#
# ref work
# Kajiwara, Shiro Masuda, & Yoshihiro Matsui. (2018). Optimal prefilter design in FRIT using closed-loop step response data. 
# Journal of the Society of Instrument and Control Engineers, 54(2), 238-246.
#
# Yoshihiro Matsui, Hideki Ayano, Shiro Masuda, & Kazushi Nakano. (2017). Implementation of pre-filter for VRFT by FIR filter. 
# Transactions of the Institute of Electrical Engineers of Japan, 137(7), 884-890.
#
import numpy as np
from control.matlab import *
from matplotlib import pyplot as plt
import pandas as pd
import sys
from scipy.optimize.minpack import transpose

if len(sys.argv[0]) > 0:
    try:
        valb=int(sys.argv[1])
    except:
        valb=1
else:
    valb=1

deffile='tune_test_data.csv'
try:
    deffile=str(sys.argv[2])
except:
    print("no file specified using default tune_test_data.csv")
    
df = pd.read_csv(deffile)                             # Read input/output response

u=df['u'].values                                      ​​# Read input data string
t=df['t'].values                                      ​​# Read time data string e.g. 0, 0.1, 0.2 etc..
y=df['y'].values                                      ​​# Read output data string
ref=df['ref'].values ​​                                 # Read output data string ? not used

# PID controller updated below
Ts=0.5                                                # Time constant of the reference model
Td=tf([1],[Ts,1])
L=Td                                                  # Prefilter (replace this with any prefilter if you want to customize)
intg=tf([1],[1,0])                                    # Integration
dif=tf([1,0],[0.1,1])                                 # Differential
(y1p, T1p, x1p )=lsim(L*(1-Td)/Td,y,t)                # P control pseudo error
(y1i, T1i, x1i )=lsim(L*intg*(1-Td)/Td,y,t)           # I control pseudo error
(y1d, T1d, x1d )=lsim(L*dif*(1-Td)/Td,y,t)            # D control pseudo error
(y2a, T2a, x2a )=lsim(L,u,t) 
    
if valb != '1' and valb !='2':
    sys.exit('arg=1 PI control 2=PID control')
else:
    if valb=='1':
        A=[y1p,y1i]                                                                             # PI control
    elif valb=='2':  
        A=[y1p,y1i,y1d]                                                                         # PID control   

invA=np.linalg.pinv(A)                                                                          # Pseudo inverse
rho=y2a@invA                                                                                    # Solve optimal parameters

if valb=='1':
    print('PI gain update result:','kp=',rho[0],':ki=',rho[1])                                # PI update
if valb=='2':
    print('PID gain update result:：','kp=',rho[0],':ki=',rho[1],':kd=',rho[2])                # PID update


