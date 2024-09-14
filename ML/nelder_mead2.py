#!/usr/bin/env python
# This class is from below I have used it with above cost functions   
# https://github.com/fchollet/nelder-mead/blob/master/nelder_mead.py
import copy
import math
import numpy as np

# ======================== cost functions ==================================
# Rosenbrock 
#
def rosenbrock(x):
    return (1 - x[0])**2 + 100*(x[1] - x[0]**2)**2

def mccormick(x):
    return np.sin(x[0] + x[1])+(x[0]-x[1])**2-(1.5*x[0])+(2.5*x[1])+1

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

def booth(x):
    x = x[0]
    y = x[1]
    return ((x+2*y-7)**2)+((2*x+y-5)**2)

def bunkinNo6(x):
    x = x[0]
    y = x[1]
    return 100*math.sqrt(y-(0.01*x**2))+0.01(x+10)
    
def five_well_positional(x):
    x1 = x[0]
    x2 = x[1]
    topexp =  (1.0 - (1/(1+0.05*(x1**2 + (x2 - 10)**2))) - (1.0/(1.0 + 0.05((x1 - 10)**2+x2**2))) - (1.5/(1+0.03*((x1+10)**2+x2**2))) + (2.0/(1+0.05*((x1-5)**2+(x2+10)**2))) - (1.0/(1+0.1((x1+5)**2 + (x2+10)**2))))   
    botexp = 1.0+(0.0001*math.pow((x1**2+x2**2),1.2))
    return (topexp / botexp)

def egg_holder(x):
    x1 = x[0]
    x2 = x[1]
    return  -(x2+47.0)*math.sin(math.sqrt(x2+(x1/2)+47.0)) - x1*math.sin(math.sqrt(x1-(x2+47.0)))  
    
def six_hump_camel(x):
    x1 = x[0]
    x2 = x[1]
    return (4 - 2.1*x1**2 + (1/3)*math.pow(x1,4))*x1**2 + x1*x2 + 4*(x2**2 - 1)*x2**2   

def three_hump_camel(x):
    x1 = x[0]
    x2 = x[1]
    return (2*x1**2) - (1.05*math.pow(x1,4)) + (math.pow(x1,6.0)/6.0) + (x1*x2) + (x2**2) 
    
def matyas(x):
    x1 = x[0]
    x2 = x[1]
    return 0.26*(x1**2+x2**2) + (0.48*x1*x2)

def levi(x):
    x1 = x[0]
    x2 = x[1]
    return np.sin(3*np.pi*x1)**2 + ((x1-1)**2*np.sin(3*np.pi*x2)**2) + ((x2-1)**2*(1+np.sin(2*np.pi*x2)**2))
    
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

def epsiloid(x):
    n = len(x)
    sum = 0
    for num in range(0,len(x)):
        sum += (math.pow(1000.0,((num-1)/(len(x)-1)))*x[num])**2
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

def k_tablet(x):
    n = len(x)
    K = n/4
    sum1 = 0
    sum2 = 0
    for num in range(0,len(x)):
        sum1 += x[num]**2
        sum2 += ((100.0*x[num])**2)*K
    return sum1 + sum2
    
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
    for num in range(0, len(x)):
        sum1 += math.pow(x[num],4.0) - (16.0*(x[num]**2)) + (5.0*x[num])
    return sum1 / 2

def sum_power(x):
    n = len(x)
    sum1 = 0
    for num in range(1, len(x)):
        sum1 += math.pow(x[num],num+1)
    return sum1 

def griewank(x):
    n = len(x)
    sum1 = 0
    sum2 = []
    for num in range(0, len(x)):
        sum1 += x[num]**2
        sum2.append(np.coa(x[num]/np.sqrt(num+1)))
    return 1 + ((1.0/4000.0)*sum1) - np.prod(sum2)  
    
'''
    Pure Python/Numpy implementation of the Nelder-Mead algorithm.
    Reference: https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method
'''


def nelder_mead(f, x_start, step=0.1, no_improve_thr=10e-6, no_improv_break=10, max_iter=0, alpha=1., gamma=2., rho=-0.5, sigma=0.5):
    '''
        @param f (function): function to optimize, must return a scalar score
            and operate over a numpy array of the same dimensions as x_start
        @param x_start (numpy array): initial position
        @param step (float): look-around radius in initial step
        @no_improv_thr,  no_improv_break (float, int): break after no_improv_break iterations with
            an improvement lower than no_improv_thr
        @max_iter (int): always break after this number of iterations.
            Set it to 0 to loop indefinitely.
        @alpha, gamma, rho, sigma (floats): parameters of the algorithm
            (see Wikipedia page for reference)

        return: tuple (best parameter array, best score)
    '''

    # init
    dim = len(x_start)
    prev_best = f(x_start)
    no_improv = 0
    res = [[x_start, prev_best]]

    for i in range(dim):
        x = copy.copy(x_start)
        x[i] = x[i] + step
        score = f(x)
        res.append([x, score])

    # simplex iter
    iters = 0
    while 1:
        # order
        res.sort(key=lambda x: x[1])
        best = res[0][1]

        # break after max_iter
        if max_iter and iters >= max_iter:
            return res[0]
        iters += 1

        # break after no_improv_break iterations with no improvement
        print('...best so far:', best)

        if best < prev_best - no_improve_thr:
            no_improv = 0
            prev_best = best
        else:
            no_improv += 1

        if no_improv >= no_improv_break:
            return res[0]

        # centroid
        x0 = [0.] * dim
        for tup in res[:-1]:
            for i, c in enumerate(tup[0]):
                x0[i] += c / (len(res)-1)

        # reflection
        xr = x0 + alpha*(x0 - res[-1][0])
        rscore = f(xr)
        if res[0][1] <= rscore < res[-2][1]:
            del res[-1]
            res.append([xr, rscore])
            continue

        # expansion
        if rscore < res[0][1]:
            xe = x0 + gamma*(x0 - res[-1][0])
            escore = f(xe)
            if escore < rscore:
                del res[-1]
                res.append([xe, escore])
                continue
            else:
                del res[-1]
                res.append([xr, rscore])
                continue

        # contraction
        xc = x0 + rho*(x0 - res[-1][0])
        cscore = f(xc)
        if cscore < res[-1][1]:
            del res[-1]
            res.append([xc, cscore])
            continue

        # reduction
        x1 = res[0][0]
        nres = []
        for tup in res:
            redx = x1 + sigma*(tup[0] - x1)
            score = f(redx)
            nres.append([redx, score])
        res = nres

LIST_OF_COST_FUNCTIONS = [ rosenbrock, mccormick, goldstein_price, levi, booth, matyas, bunkinNo6, easom, beale, five_well_positional, griewank, sum_power, egg_holder, epsiloid, three_hump_camel, six_hump_camel, rastrigin, schwefel, xin_she_yang, zakharov, shubert, styblinski_tang. k_tablet]

if __name__ == "__main__":
    # test
    import math
    import numpy as np

    def f(x):
        return math.sin(x[0]) * math.cos(x[1]) * (1. / (abs(x[2]) + 1))

    print(nelder_mead(f, np.array([0., 0., 0.])))
    
    for costfunc in LIST_OF_COST_FUNCTIONS:
        print(nelder_mead(costfunc, np.array([0., 0., 0.])))
    
  
