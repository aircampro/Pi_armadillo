#!/usr/bin/env python
# levenburg marquardt method fitting data to curve
#
import scipy
##from matplotlib import *
import pylab
from scipy.optimize import leastsq
"""
Example of curve fitting for
a1*exp(-k1*t) + a2*exp(-k2*t)
"""
def dbexpl(t,p):
    return(p[0]*pylab.exp(-p[1]*t) + p[2]*pylab.exp(-p[3]*t))

def residuals(p,data,t):
    err = data - dbexpl(t,p)
    return err

# do levenbug marquardt curve fir for setimate, data, sample time 
#
def l_m(p0, data, sample_interval):
    t=scipy.arange(0,len(data)/sample_interval,sample_interval)
    guessfit = dbexpl(t,p0)                                          # plot for the guess data
    pbest = leastsq(residuals,p0,args=(data,t),full_output=1)        # do best fit curve
    bestparams = pbest[0]
    cov_x = pbest[1]
    return bestparams, cov_x

# example
#
def main():
    a1,a2 = 1.0, 1.0
    k1,k2 = 0.05, 0.2
    t=scipy.arange(0,100,0.1)                                        # time span
    data = dbexpl(t,[a1,k1,a2,k2]) + 0.02*pylab.randn(len(t))        # create a data list
    p0 = [0.5,1,0.5,1]                                               # initial guesses
    guessfit = dbexpl(t,p0)                                          # plot for the guess data
    pbest = leastsq(residuals,p0,args=(data,t),full_output=1)        # do best fit curve
    bestparams = pbest[0]
    cov_x = pbest[1]
    print('best fit parameters ', bestparams)
    print('covarience', cov_x)
    datafit = dbexpl(t ,bestparams)                                  # plot for result
    pylab.plot(t,data,'x',t,datafit,'r',t,guessfit)                  # plot the data against the curves
    pylab.xlabel('Time')
    pylab.title('Curve-fitting example')
    pylab.grid(True)
    pylab.draw()
    pylab.show()
 
if __name__ == '__main__':
    main() 