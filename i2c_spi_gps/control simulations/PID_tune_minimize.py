#!/usr/bin/env python
#
# PID Controller tuning using various optimization methods e.g. Nelder-Mead
#
from control import matlab
import numpy as np
import sympy
from scipy.optimize import minimize

def T(x):
    """
    Loss Function(evaluation function)
    parameters
    ---------------------------------
    x[0](=K);proprotional gain
    x[1](=Ti);integral time
    x[2](=Td);Derivarive time
    
    return
    ---------------------------------
    valuation of Loss Function
    """
    
    
    """process model"""
    Kt = 10                             # Steady-State gain
    J = 25                              # Time Constant
    Cm = 4                              # Dead time 
    num = [Kt]                          # Numerator coefficient
    dem = [J,1]                         # Denominator coefficient
    P = matlab.tf(num,dem)
    num2, dem2 = matlab.pade(Cm, 6)
    Pade = matlab.tf(num2,dem2)
    Process = P * Pade
    """PID controller"""
    K = x[0]                            # Proprotional Gain
    Ti = x[1]                           # Integral Time
    Td = x[2]                           # Derivarive Time
    # Convert to coefficients for transfer function
    kp = K
    ki = kp/Ti
    kd = Td * kp
    num = [kd, kp, ki]
    den = [1, 0]
    C = matlab.tf(num, den)
    """Feedback Loop"""
    sys = matlab.feedback(Process*C,1,-1)
    """Step Response"""
    time= 100                           # Time to calculate step response [sec]
    t = np.linspace(0,time,1000)        # Specify the time.
    y1,T = matlab.step(sys,t)
    return sum(T*(y1*1000 - 1000)**2) 

def print_opt_res(mtd, x):
    print("\033[32m ---- ",mtd," ---- \033[9m")
    print(f"Kp {x.x[0]} Ki {x.x[1]} Kd {x.x[2]}")

if __name__ == "__main__": 	

    # Initial value
    init = [1,1,1]
    # optimization using various methods
    nelder_mead = minimize(T,init, method='Nelder-Mead')
    print_opt_res('Nelder-Mead', nelder_mead)
    powell = minimize(T,init, method='Powell')
    print_opt_res('Powell', powell)
    cg = minimize(T,init, method='CG')
    print_opt_res('CG', cg)
    bfgs = minimize(T, init, method='BFGS')
    print_opt_res('BFGS', bfgs)
    newton_cg = minimize(T,init, method='Newton-CG')
    print_opt_res('Newton-CG', newton_cg)
    l_bfgs_b = minimize(T, init, method='L-BFGS-B')
    print_opt_res('L-BFGS-B', l_bfgs_b)
    tnc = minimize(T, init, method='TNC')
    print_opt_res('TNC', tnc)
    cobyla = minimize(T, init, method='COBYLA')
    print_opt_res('COBYLA', cobyla)
    slsqp = minimize(T, init, method='SLSQP')
    print_opt_res('SLSQP', slsqp)
    dogleg = minimize(T, init, method='dogleg')
    print_opt_res('dogleg', dogleg)
    trust_ncg = minimize(T, init, method='trust-ncg')
    print_opt_res('trust-ncg', trust_ncg)




