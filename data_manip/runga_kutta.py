#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# Examples of using the Runga-Kutta method
#	
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict

# The interval with time t is divided by 100 and the following differential equation (output voltage v of the RC circuit) is calculated in order.
# (1) \begin{eqnarray*} \frac{dv(t)}{dt}=\frac{e-v(t)}{rc} \end{eqnarray*}
# e: Input voltage [V], v: Output voltage [V], r: Resistance [Ω], c: Capacitor capacitance [F]
#
class Runge:
    def __init__(self, cc=0.001, rr=100, ee=10):
        self.c = cc
        self.r = rr
        self.e = ee
    
    def dxdt(self, x):
        return (self.e-x)/self.r/self.c                        # this is the equation to solve

    # x0: Initial condition
    # t0, tn: interval [t0, tn]
    # n: Number of divisions 
    def runge(self, x0, t0, tn, n):
        x = x0
        t = t0
        h = (tn - t0) /n
        X = []

        for i in range(n):
            d1 = self.dxdt(x);
            d2 = self.dxdt(x + d1*h*0.5);
            d3 = self.dxdt(x + d2*h*0.5);
            d4 = self.dxdt(x + d3*h);
            x += (d1 + 2 * d2 + 2 * d3 + d4)*(h/6.0); 
            X.append(x)
        return X
 
    def run(self, x=0.0, y=0.0, z=1.0, w=100):
        X = self.runge(x, y, z, w)
        print(X)

# Solving Ordinary Differential Equations using runge-kutta method
#       
def func(x):
    return x**2                                # f(x)=x^2

def diff_func(x):
    return 2*x                                 # f'(x)=2x

dx = 0.5                                       # minute change
xs = np.array([i*dx for i in range(50)])       # data
   
#Runge Kutta method 
class RungeKutta4thOrder:
    def __init__(self, dx=0.5, diff_func, log_verbose=False):
        self.dx = dx
        self.diff_func = diff_func
        self.log_verbose = log_verbose 
        self.output = {'k1':[], 'k2':[], 'k3':[], 'k4':[], 'k':[]}

    # 4th order Rungekutta
    def step_k(self, x):
        k1 = self.diff_func(x)*self.dx
        k2 = self.diff_func(x + k1/2*self.dx)*self.dx
        k3 = self.diff_func(x + k2/2*self.dx)*self.dx
        k4 = self.diff_func(x + k3*self.dx)*self.dx
        
        #k：Runge-Kutta
        k = (k1 + 2*k2 + 2*k3 + k4)/6 
        
        if self.log_verbose:
            self.logging_data(self.output, {'k1':k1, 'k2':k2, 'k3':k3, 'k4':k4, 'k':k})
        
        return k
    
    def logging_data(self, datas:List, params:Dict):
        for key, value in params.items():
            datas[key].append(value)

# The differential equation being solved isy¨=−y
# that is ddt(y(t),y˙(t))=(y˙(t),−y(t))
# The initial conditions are(y(0),y˙(0))=(cos0,−sin0)=(1,0)
# the solution is(y(t),y˙(t))=(cost,−sint)
#
def rk4( y, t, h, f ):
    # Gill's low strage method is implemented.
    p= np.zeros_like(y); q= np.zeros_like(y)
    p=f(t,y); p=p*h*(1./2.); rt2=2**-.5; y+=p; q=p*2
    p=f(t,y); p=p*h*(1-rt2) - q*(1-rt2); y+=p; q=p*2 + q*rt2
    p=f(t,y); p=p*h*(1+rt2) - q*(1+rt2); y+=p; q=p*2 - q*rt2
    p=f(t,y); p=p*h*(1./6.) - q*(2./6.); y+=p;
    return y, t+h

def RHS(t,y):
    return np.array([y[1],-y[0]])            

# Parameters of the mass spring system
# Mass m = 10 kg
# Stiffness k = 50 N/m
# Viscous damping c = 5 Ns/m
#
class RungeKuttaVib:
    def __init__(self, n = 1000, m = 10, k = 50, c = 5):
        self.n = n
        self.t = np.linspace(0, 10, self.n)
        self.force = np.zeros(self.n)	
        for i in range(100, 150):
            self.a = np.pi / 50 * (i - 100)
            self.force[i] = np.sin(self.a)	
        self.m = m
        self.k = k
        self.c = c
        self.u = None
        self.v = None
		
    def runga_kutta_vibrations(t, u0, v0, m, c, k, force):
        """
        :param t: (list/ array)
        :param u0: (flt)u at t[0]
        :param v0: (flt) v at t[0].
        :param m:(flt) Mass.
        :param c: (flt) Damping.
        :param k: (flt) Spring stiffness.
        :param force: (list/ array) Force acting on the system.
        :return: (tpl) (displacement u, velocity v)
        """
        u = np.zeros(t.shape)
        v = np.zeros(t.shape)
        u[0] = u0
        v[0] = v0
        dt = t[1] - t[0]
    
        # Returns the acceleration a
        def func(u, V, force):
            return (force - c * V - k * u) / m

        for i in range(t.size - 1):
            # F at time step t / 2
            f_t_05 = (force[i + 1] - force[i]) / 2 + force[i]

            u1 = u[i]
            v1 = v[i]
            a1 = func(u1, v1, force[i])
            u2 = u[i] + v1 * dt / 2
            v2 = v[i] + a1 * dt / 2
            a2 = func(u2, v2, f_t_05)
            u3 = u[i] + v2 * dt / 2
            v3 = v[i] + a2 * dt / 2
            a3 = func(u3, v3, f_t_05)
            u4 = u[i] + v3 * dt
            v4 = v[i] + a3 * dt
            a4 = func(u4, v4, force[i + 1])
            u[i + 1] = u[i] + dt / 6 * (v1 + 2 * v2 + 2 * v3 + v4)
            v[i + 1] = v[i] + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)

        return u, v

    def do_rkv(self):
        self.u, self.v = self.runga_kutta_vibrations(self.t, 0, 0, self.m, self.c, self.k, self.force)	

    # Plot & save the result
    def plot(self):
        if not self.v == None:
            fig, ax1 = plt.subplots()
            l1 = ax1.plot(self.t, self.v, color='b', label="displacement")
            ax2 = ax1.twinx()
            l2 = ax2.plot(self.t, self.force, color='r', label="force")
            plt.show()
            plt.savefig('vibration.png')	
            
if __name__ == '__main__':

    # output voltage
    rr = Runge()
    rr.run()

    # Differential Equations
    y0 = func(xs[0]) 
    y = y0
    y_cal = []

    #Euler method
    for x in xs:
        y_cal.append(y)
        y += diff_func(x)*dx                        # y
    
    rungekutta = RungeKutta4thOrder(dx=dx, diff_func=diff_func)
    y = y0 
    y_cal_runge = []

    for x in xs:
        y_cal_runge.append(y)
        dy = rungekutta.step_k(x) 
        y += dy
    
    # show the data
    fig = plt.figure(figsize=(10, 5), facecolor='white')
    plt.plot(xs, func(xs), label='y=x^2')
    plt.plot(xs, y_cal, label='Euler method', linestyle='dashed')
    plt.plot(xs, y_cal_runge, label='Runge-Kutta method', linestyle='dashed')
    # plt.xticks(xs)
    # plt.xlim(0, 5); plt.ylim(0, 40)
    plt.grid(); plt.legend(); plt.text(0.1, max(func(xs))/2, f'Step Size h:{dx:.1f}', fontsize=14)
    plt.savefig(f'savedata_Runge_h={dx:.1f}.png')
    plt.show()
    plt.savefig("runge_kutta.png")
    
    # Gills method
    y=np.array([1.0,0.0]); t=0; h=0.001
    for i in range(100):
        y, t =rk4( y, t, h, RHS )
        print(y[0]-np.cos(t))
        
    # vibration problem
    rkv = RungeKuttaVib(m=40)                                   # mass 40 kg
    rkv.do_rkv()
    rkv.plot()    