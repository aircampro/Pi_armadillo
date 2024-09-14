#!/usr/bin/env python
#
# nelder mead optimisation example(s)
#
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib.animation import PillowWriter, FuncAnimation
import math

# ======================== cost functions ==================================
# ref :- https://qiita.com/tomitomi3/items/d4318bf7afbc1c835dda
#
# Rosenbrock 
#
def rosenbrock(x):
    return (1 - x[0])**2 + 100*(x[1] - x[0]**2)**2

def mccormick(x):
    return math.sin(x[0] + x[1])+(x[0]-x[1])**2-(1.5*x[0])+(2.5*x[1])+1

def easom(x):
    x = x[0]
    y = x[1]
    return math.cos(x)*math.cos(y)*math.exp(-((x-math.pi)**2)+((y-math.pi)**2))

def goldstein_price(x):
    x = x[0]
    y = x[1]
    return (1.0 + (x+y+1)**2*(19.0 - 14*x + 3*x**2 - 14*y + 6*x*y + 3*y**2))*(30+((2*x-3*y)**2)*(18-32*x-12*x**2+48*y-36*x*y+27*(y**2)))

def beale(x):
    x = x[0]
    y = x[1]
    return (1.5 - x + x*y)**2 + (2.25 - x + x*y**2)**2 + (2.625 - x + x*y**3)**2
    
def five_well_positional(x):
    x1 = x[0]
    x2 = x[1]
    topexp =  (1.0 - (1/(1+0.05*(x1**2 + (x2 - 10)**2))) - (1.0/(1.0 + 0.05((x1 - 10)**2+x2**2))) - (1.5/(1+0.03*((x1+10)**2+x2**2))) + (2.0/(1+0.05*((x1-5)**2+(x2+10)**2))) - (1.0/(1+0.1((x1+5)**2 + (x2+10)**2))))   
    botexp = 1+(0.0001*math.pow((x1**2+x2**2),1.2))
    return topexp / botexp

def egg_holder(x):
    x1 = x[0]
    x2 = x[1]
    return  -(x2+47.0)*math.sin(math.sqrt(x2+(x1/2)+47.0)) - x1*math.sin(math.sqrt(x1-(x2+47.0)))  
    
def six_hump_camel(x):
    x1 = x[0]
    x2 = x[1]
    return (4 - 2.1*x1**2 + (1/3)*math.pow(x1,4))*x1**2 + x1*x2 + 4*(x2**2 - 1)*x2**2   
    
def rastrigin(x):
    n = len(x)
    sum = 0
    for num in range(0,len(x)):
        sum += x[num]**2 - (10.0*math.cos(2.0*math.pi*x[num]))
    return (10.0*n) - sum

def schwefel(x):
    n = len(x)
    sum = 0
    for num in range(0,len(x)):
        sum += x[num]*math.sin(math.sqrt(x[num]))
    return - sum

def xin_she_yang(x):
    n = len(x)
    sum1 = 0
    sum2 = 0
    for num in range(0,len(x)):
        sum1 += x[num]
        sum2 += math.sin(x[num]**2)
    return sum1 * math.exp(-sum2)

def zakharov(x):
    n = len(x)
    sum1 = 0
    sum2 = 0
    for num in range(0,len(x)):
        sum1 += x[num]
        sum2 += num*x[num]
    return sum1 + ((1.0/2.0)*sum2)**2 + math.pow(((1.0/2.0)*sum2),4)

def shubert(x):
    n = len(x)
    sum1 = 0
    sum2 = 0
    for num in range(0, len(x)):
        for num >= 1:
            sum1 += num*math.cos(num + ((num+1)*x[num-1]))
            sum2 += num*math.cos(num + ((num+1)*x[num]))
    return sum1 * sum2

def styblinski_tang(x):
    n = len(x)
    sum1 = 0
    sum2 = 0
    for num in range(0, len(x)):
        sum1 += math.pow(x[num],4.0) - (16.0*(x[num]**2)) + (5.0*x[num])
    return sum1 / 2
    
# Rosenbrock cost function with gradient
#
def rosenbrock_grad(x):
    cost = (1 - x[0])**2 + 100*(x[1] - x[0]**2)**2
    grad = []
    g = -2.0 * (1.0 - x[0]) - 200.0 * (x[1] - x[0]**2) * 2.0 * x[0]
    grad.append(g)
    g = 200.0 * (x[1] - x[0]**2)
    grad.append(g)
    return cost, np.array(grad)  

# define some residual functions
#
def Rosenbrock_Residual(x):
    res = []
    r = 10.0 * (x[1] - x[0]**2)
    res.append(r)
    r = 1.0 - x[1]
    res.append(r)
    return np.array(res)

def FreudensteinRoth_Residual(x):
    x1 = x[0]
    x2 = x[1]
    res = []
    r = -13.0 + x1 + ((5.0 - x2) * x2 - 2.0) * x2
    res.append(r)
    r = -29.0 + x1 + ((x2 + 1.0) * x2 - 14.0) * x2
    res.append(r)
    return np.array(res)

def Powell_BS_Residual(x):
    x1 = x[0]
    x2 = x[1]
    res = []
    r = 10000.0 * x1 * x2 - 1.0
    res.append(r)
    r = math.exp(-x1) + math.exp(-x2) - 1.0001
    res.append(r)
    return np.array(res)

def Brown_BS_Residual(x):
    x1 = x[0]
    x2 = x[1]
    res = []
    r = x1 - 1000000.0
    res.append(r)
    r = x2 - 0.000002
    res.append(r)
    r = x1 * x2 - 2.0
    res.append(r)
    return np.array(res)

def Beale_Residual(x):
    x1 = x[0]
    x2 = x[1]
    res = []
    r = 1.5 - x1 * (1.0 - x2)
    res.append(r)
    r = 2.25 - x1 * (1.0 - x2 * x2)
    res.append(r)
    r = 2.625 - x1 * (1.0 - x2 * x2 * x2)
    res.append(r)
    return np.array(res)

def BrownDennis_Residual(x):
    x1 = x[0]
    x2 = x[1]
    x3 = x[2]
    x4 = x[3]

    res = []
    for i in range(0, 20):
        ti = (i + 1.0) / 5.0
        r = (x1 + ti * x2 - math.exp(ti)) * (x1 + ti * x2 - math.exp(ti)) + (x3 + x4 * math.sin(ti) - math.cos(ti)) * (x3 + x4 * math.sin(ti) - math.cos(ti))
        res.append(r)
    return np.array(res)
    
from scipy.optimize import minimize

#   ======================================        main (compare optimization methods shown below)       =================================================

# test nelder mead with beale  
x0 = [1.0, 1.0]
res = minimize(beale, x0, method="Nelder-Mead")
print("beale",res)
# now do all the cost functions with nelder mead 
for costfunc in [ rosenbrock, mccormick, easom, beale, five_well_positional, egg_holder, goldstein_price, six_hump_camel, rastrigin, schwefel, xin_she_yang, zakharov, shubert, styblinski_tang]:
    res = minimize(costfunc, x0, method="Nelder-Mead")
    print(str(costfunc).split(" ")[1],"  ",res)

# another method loading cost functions and optimization methods direct from scipy libs
#
from scipy. optimize import minimize, rosen, rosen_der, rosen_hess

# make simplex array
x0 = [2.0 for i in range(5)]
# mimimize with rosen
res = minimize(rosen, x0, method="Nelder-Mead")
print("rosen nelder mead",res.x)

# other methods
# res = minimize(rosen, x0, method="UNDX")
# res = minimize(rosen, x0, method="SPX")
# res = minimize(rosen, x0, method="PCX")
# res = minimize(rosen, x0, method="PSO")
# res = minimize(rosen, x0, method="LDIW")
# res = minimize(rosen, x0, method="AIW")
# res = minimize(rosen, x0, method="ChaoticIW")

# BFGS with rosen cost function
#
res = minimize(rosen, x0, jac=rosen_der, method="BFGS")
print("rosen BFGS",res.x)

# trust-ncg with rosen
#
res = minimize(rosen, x0, jac=rosen_der, hess=rosen_hess, method="trust-ncg")
print("rosen trust-ncg",res.x)

# Nelder-Mead with rosebrock example
#
# 
X = np.linspace(-2, 2, 400)
Y = np.linspace(-1, 3, 400)
X, Y = np.meshgrid(X, Y)
Z = (1 - X)**2 + 100*(Y - X**2)**2

# initilaise array
points = []

def callback(xk):
    points.append(xk)

# mimimze using rosenbrock cost function and nelder mead method
#
initial_simplex = np.array([[-1.0, 1.0], [-0.5, 1.6], [1.3, 1.5]])
minimizer = minimize(rosenbrock, initial_simplex[0], method='Nelder-Mead', callback=callback, options={'initial_simplex': initial_simplex, 'return_all': True})

# print results
print("Check fitting results")
print(minimizer)

# minimizer['allvecs']
simplexes = minimizer['allvecs']

# visualize the data
fig, ax = plt.subplots(figsize=(8,6))
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Nelder-Mead algorism using Rosenbrock function")

Z_min, Z_max = np.min(Z), np.max(Z)
levels = np.logspace(np.log10(Z_min), np.log10(Z_max), 35)
#ax.contour(X, Y, Z, levels=np.logspace(0, 5, 35), cmap='jet')
ax.contour(X, Y, Z, levels=levels, cmap='cool', linewidths=1, alpha=0.9)
line, = ax.plot([], [], lw=1, color="red")

def init():
    line.set_data([], [])
    return line,

text = ax.text(0.005, 1.06, '', transform=ax.transAxes)

def animate(i):
    if i < 3:  
        simplex = simplexes[:i+1]
    else:
        simplex = simplexes[i-2:i+1]
    
    x_data, y_data = zip(*simplex)
    
    last_x, last_y = x_data[-1], y_data[-1]
    
    x_data += x_data[:1]
    y_data += y_data[:1]
    print(str(i) + "-th Iteratrion x_data = ", x_data)
    
    line.set_data(x_data, y_data)
    
    text.set_text(f"Frame: {i} (x,y)=({last_x:.3f},{last_y:.3f})")
    
    return line, text

frame_per_sec = 5 
interval = int(1e3/5)                               # ms for FuncAnimation

ani = FuncAnimation(fig, animate, frames=len(simplexes), init_func=init, blit=True, interval=interval)
ani.save('nelder_mead_rosenbrock.gif', writer=PillowWriter(fps=frame_per_sec))

# ============== another example with another optimizer added to the chain ========================

from typing import Callable, Tuple, Union

import numpy as np

def _order(x: np.ndarray, ordering: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    indices = np.argsort(ordering)
    return x[indices], ordering[indices]

def optimize( fun: Callable, x0: np.ndarray, maxiter: Union[int, None] = None,   initial_simplex: Union[np.ndarray, None] = None ):
    if x0.ndim != 1:
        raise ValueError(f'Expected 1D array, got {x0.ndim}D array instead')

    # initialize simplex
    if initial_simplex is not None:
        if initial_simplex.ndim != 2:
            raise ValueError(f'Expected 2D array, got {x0.ndim}D array instead')
        x = initial_simplex.copy()
        n = x[0].size
    else:
        h = lambda x: (x[0][x[1]] != 0) * (0.05 - 0.00025) + 0.00025
        n = x0.size
        x = np.array([x0 + h([x0, i]) * e for i, e in enumerate(np.identity(n))] + [x0])

    if maxiter is None:
        maxiter = 200 * n

    # parameters
    alpha = 1.0
    gamma = 2.0
    rho = 0.5
    sigma = 0.5

    # order
    fx = np.array(list(map(fun, x)))
    x, fx = _order(x, fx)

    # centroid
    xo = np.mean(x[:-1], axis=0)
    n_inv = 1 / n

    for _ in range(maxiter):
        fx1 = fx[0]
        fxn = fx[-2]
        fxmax = fx[-1]
        xmax = x[-1]

        xr = xo + alpha * (xo - xmax)
        fxr = fun(xr)

        if fx1 <= fxr and fxr < fxn:
            # reflect
            x[-1] = xr
            fx[-1] = fun(xr)
            x, fx = _order(x, fx)
            xo = xo + n_inv * (xr - x[-1])

        elif fxr < fx1:
            xe = xo + gamma * (xo - xmax)
            fxe = fun(xe)
            if fxe < fxr:
                # expand
                x = np.append(xe.reshape(1, -1), x[:-1], axis=0)
                fx = np.append(fxe, fx[:-1])
                xo = xo + n_inv * (xe - x[-1])
            else:
                # reflect
                x = np.append(xr.reshape(1, -1), x[:-1], axis=0)
                fx = np.append(fxr, fx[:-1])
                xo = xo + n_inv * (xr - x[-1])

        else:
            if fxr > fxmax:
                xc = xo + rho * (xmax - xo)
            else: 
                xc = xo + rho * (xr - xo)
                fxmax = fxr
            if fun(xc) < fxmax:
                # contract
                x[-1] = xc
                fx[-1] = fun(xc)
                x, fx = _order(x, fx)
                xo = xo + n_inv * (xc - x[-1])
            else:
                # shrink
                x[1:] = (1 - sigma) * x[0] + sigma * x[1:]
                fx[1:] = np.array(list(map(fun, x[1:])))
                x, fx = _order(x, fx)
                xo = np.mean(x[:-1], axis=0)

    return x, fx

# maximum iterations for optimization  
maxiter = 25

# cost function used 
fun = lambda x: x @ x
# simplex used here
x0 = np.array([0.08, 0.08])

# scipy
res = minimize(fun=fun, x0=x0, options={'maxiter': maxiter}, method='Nelder-Mead')
xopt_scipy = res.x

# implemented
xopt, _ = optimize(fun=fun, x0=x0, maxiter=maxiter)

print('\n')
print(f'Scipy: {xopt_scipy}')
print(f'Implemented: {xopt[0]}')
 
