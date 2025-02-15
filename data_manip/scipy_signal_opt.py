#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# This example shows signal filters butterworht, IIR and also SG as shown and described here
# ref:- https://qiita.com/lcmtk/items/06bdd965d8a79bbfd0a9
# How to clean spectral data with smoothing and baseline estimation
# example of using the functions in scipy
#
import csv
import numpy as np

# overview of the scipy library
#
# scipy.linalg	    Functions of linear algebra (matrix calculations, inverse of matrices, eigenvalues, etc.)
# scipy.optimize	Optimization and equation solving
# scipy.stats	    Statistical distributions, hypothesis testing, probability calculations
# scipy.integrate	Numerical integration and solving ordinary differential equations
# scipy.signal	    Signal processing (filtering, conversion, spectral analysis, etc.)
# scipy.fftpack	    Fast Fourier transform and inverse transformation
# scipy.spatial	    Calculation of spatial data (distance calculations, Voronoi diagrams, etc.)
# scipy.interpolate	Interpolation of data (spline interpolation, multidimensional interpolation)
# scipy.ndimage	    TN-dimensional image processing his
#
from scipy import signal
import matplotlib.pyplot as plt
from scipy.sparse import csc_matrix
from scipy.sparse import spdiags
import scipy.sparse.linalg as spla

from scipy import linalg

# example of matrix maths
class matrix_math2x2():
    def __init__(self,a,b,c,d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.A = None
        
    def make2x2(self):
        self.A = np.array([[self.a, self.b], [self.c, self.d]])         # 2x2 matrix
        
    def inver(self):
       if not self.A == None:
           # inverse
           self.inv_A = linalg.inv(self.A)
           print("inverse matrix ：\n", inv_A)
           
    def eigen(self):
        if not self.A == None:
            eigenvalues, eigenvectors = linalg.eig(self.A)
            print("eigen values：\n", eigenvalues)
            print("eigen vector：\n", eigenvectors)

# optimize.minimize()to find the minimum value of a function.
#
from scipy import optimize

# minimize
class minimize_func():
    def __init__(self):
    
    # cost function
    def f(self, x):
        return x**2 + 5 * np.sin(x)

    def res(self, x=0):
        # minimise x0=0
        result = optimize.minimize(self.f, x0=x)
        return result
        
    def print_res(self, result):
        if not result.message.find("successfully") == -1:
            print("Value of x that takes the minimum value ： ", result.x)
            print("Minimum value of objective function ： ", result.fu

# non-linear
class opt_nonlin():
    def __init__(self):
    
    def equation(self, x):
        return np.cos(x) - x
        
    def solve(self,x=0):
        solution = optimize.root(self.equation, x0=x)        # To find the solution to a nonlinear equation, use .optimize.root()
        if not solution.message.find("converged") == -1:
            print("solution of equation ： ", solution.x)
        return solution

# Solving ordinary differential equations
# Here is an example of a simple ordinary differential equation (dy/dt = -y) solved numerically with SciPy.
#
from scipy.integrate import solve_ivp
class solv_ivp():
    def __init__(self):
        self.solution = None
        
    def model(self, t, y):
        return -y

    def soln(self):
        self.solution = solve_ivp(self.model, [0, 5], [1], t_eval=np.linspace(0, 5, 100))

    def plot_ivp(self):
        plt.plot(self.solution.t, self.solution.y[0])
        plt.xlabel('Time')
        plt.ylabel('y')
        plt.show()
        plt.savefig("ivp.png")
        
# covolve
def convolve(mode=2, meth=0):
    modelist = [ "same", "valid", "full" ]
    methlist = [ "auto", "direct", "fft" ]
    me = methlist[meth]
    m = modelist[mode]    
    # データ作成
    in1 = [ 1.0 if i % 100 == 50 else 0.0 for i in range(500) ]
    in2 = signal.windows.hann(50)
    out = signal.convolve(in1, in2, mode=m, method=me)                                                
    xmax = len(out)

    fig = plt.figure(figsize = (10,6))
    ax1 = fig.add_subplot(3, 1, 1)
    ax2 = fig.add_subplot(3, 1, 2)
    ax3 = fig.add_subplot(3, 1, 3)
    ax1.set_xlim(0, xmax)
    ax2.set_xlim(0, xmax)
    ax3.set_xlim(0, xmax)
    ax1.plot(in1)
    ax2.plot(in2)
    ax3.plot(out)
    plt.show()
    plt.savefig("convolve.png") 
    
# butterworth filter on data
def butter_filter():
    # create timespec and data
    t = np.linspace(0, 1.0, 200)
    x = np.sin(2 * np.pi * 5 * t) + np.sin(2 * np.pi * 50 * t)

    # butterworth signal filter
    b, a = signal.butter(4, 0.1)
    y = signal.filtfilt(b, a, x)

    # show the data
    plt.plot(t, x, label='Noisy signal')
    plt.plot(t, y, label='Filtered signal')
    plt.legend()
    plt.show()
    plt.savefig("butter.png")
    
# Enter the parameters, if you are having trouble estimating them, please play around here.
# Baseline estimation in AsLS is ( W(p) + lam*D'D ) when z = Wy, weight p and penalty term coefficient lam are parameters
# Savitzky-Goly sets how many parts the measurement value is divided into with dn (the number of windows is len(Y)/dn) and sets the polynomial degree with poly
#
# paramAsLS = [ lam , p ]
# paramSG   = [ dn , poly ]
paramAsLS = [10**3.5, 0.00005]
paramSG = [80, 5]

# infared spectra data e.g. to find composition of a plastic
#
filename = 'testIR.csv'

# Open measurement data (csv)
#
X = []
Y = []
with open(filename, 'r') as f:
    reader = csv.reader(f)
    data = [row for row in reader]
    for i in range(len(data)):
        X.append(float(data[i][0]))
        Y.append(float(data[i][1]))

# AsLS Asymmetric least squares
# method is called Asymmetric least squares smoother (or Whittaker smoother).
#
def baseline_als(y, lam, p, niter=10):
    #https://stackoverflow.com/questions/29156532/python-baseline-correction-library
    #p: 0.001 - 0.1, lam: 10^2 - 10^9
    # Baseline correction with asymmetric least squares smoothing, P. Eilers, 2005
    L = len(y)
    D = csc_matrix(np.diff(np.eye(L), 2))
    w = np.ones(L)
    for i in range(niter):
        W = spdiags(w, 0, L, L)
        Z = W + lam * D.dot(D.transpose())
        z = spla.spsolve(Z, w*y)
        w = p * (y > z) + (1-p) * (y < z)
    return z

# smoothing
# We use the general polynomial law (Savitzky-Golay method).
# The reason for its use is that it is a proven method, easy to differentiate, and easy to develop into peak position identification and fitting.
# Savitzky-Golay
def SGs(y,dn,poly):
    # y as np.array, dn as int, poly as int
    n = len(y) // dn
    if n % 2 == 0:
        N = n+1
    elif n % 2 == 1:
        N = n
    else:
        print("window length can't set as odd")
    SGsmoothed = signal.savgol_filter(y, window_length=N, polyorder=poly)
    return SGsmoothed

# Output csv file and diagram
#
def outFigCSV(X,Y,paramAsLS, paramSG):

    # baseline estimation and smoothing
    Y_np = np.array(Y)
    bkg = baseline_als(Y_np,paramAsLS[0], paramAsLS[1])
    fix = Y_np - bkg
    smth= SGs(fix, paramSG[0], paramSG[1])

    # output
    dataOutput = np.c_[X, Y, bkg, smth]
    np.savetxt('output.csv', dataOutput, delimiter=',')

    #figures
    plt.figure(figsize=(12,9))
    ax1 = plt.subplot2grid((2,2), (0,0), colspan=2)
    ax2 = plt.subplot2grid((2,2), (1,0), colspan=2)

    ax1.plot(X, Y, linewidth=2)
    ax1.plot(X, bkg, "b", linewidth=1, linestyle = "dashed", label="baseline")

    ax2.plot(X, fix, "g", linewidth=1, linestyle = "dashed", label="remove baseline")
    ax2.plot(X, smth, "b", linewidth=2, label="smoothed")

    plt.axis("tight")
    plt.show()
    plt.savefig("sg.png")
    
def plotiirba(b, a, fs, title='', worN=512, shape=100):

    fn = fs / 2.0
    rad_to_freq = 1 / np.pi * fn

    # Frequency characteristics (amplitude characteristics)
    w, h = signal.freqz(b, a, worN)
    x_freq_rspns = w * rad_to_freq
    y_freq_rspns = 20 * np.log10(abs(h) / 1.0 + 1.0e-06)

    # group delay
    w, gd = signal.group_delay((b, a), worN)
    x_gd = w * rad_to_freq
    y_gd = gd

    # unit circle, zero point, pole
    theta = np.linspace(0, 2*np.pi, 360)
    x_unit_circle = np.cos(theta)
    y_unit_circle = np.sin(theta)
    zeros, poles, k = signal.tf2zpk(b, a)

    # --- Impulse response
    x_impluse = signal.unit_impulse(shape)
    y_impluse = signal.lfilter(b, a, x_impluse)

    # ------- Graph display
    fig = plt.figure(figsize=(8, 10), tight_layout=True)
    fig.suptitle(title)

    # --- Frequency characteristics (amplitude characteristics)
    ax = plt.subplot2grid((3, 2), (0, 0), colspan=2)
    ax.plot(x_freq_rspns, y_freq_rspns)
    ax.set_ylabel('Amplitude [dB]')
    ax.set_xlabel('Frequency [Hz]')
    ax.grid()

    # --- group delay
    ax = plt.subplot2grid((3, 2), (1, 0), colspan=2)
    ax.plot(x_gd, y_gd)
    ax.set_ylabel('Group delay [samples]')
    ax.set_xlabel('Frequency [Hz]')
    ax.grid()

    # --- Poles-zeros plot
    ax = plt.subplot2grid((3, 2), (2, 0))
    ax.plot(x_unit_circle, y_unit_circle, c='k', ls=':')
    for z in zeros:
        x, y = np.real(z), np.imag(z)
        if np.sqrt(x**2+y**2) > 1:
            ax.plot(x, y, marker='o', c='r', fillstyle='none')
        else:
            ax.plot(x, y, marker='o', c='k', fillstyle='none')
    # Extreme
    for p in poles:
        x, y = np.real(p), np.imag(p)
        if np.sqrt(x**2+y**2) > 1:
            ax.plot(x, y, marker='x', c='r')
        else:
            ax.plot(x, y, marker='x', c='k')
    ax.axis('equal')  # Make it square
    ax.set_ylabel('Imaginary part')
    ax.set_xlabel('Real part')
    ax.grid()

    # --- Impulse response
    ax = plt.subplot2grid((3, 2), (2, 1))
    ax.plot(y_impluse, label='System')
    ax.plot(x_impluse, label='Unit impulse')
    ax.set_ylabel('Magnitude')
    ax.set_xlabel('Samples')
    ax.legend()
    ax.grid()

    return fig
	
class TestIirDesign():
    def __init__(self):
        self.fs = 250                # sampling frequency
        self.sec = 3                 # time

        # Waveform you want to extract
        self.A = 1                   # amplitude
        self.f = 5                   # frequency

        # Noise｜sin wave
        self.A_n = 0.1
        self.f_n = 50

        # --- IIR filter
        self.wp = 10                 # Passband cutoff frequency [Hz]
        self.ws = 50                 # Stopband cutoff frequency [Hz]
        self.gpass = 0.5             # Passband maximum loss [dB]
        self.gstop = 30              # Minimum stopband attenuation [dB]
        self.analog = False          # False because it is a digital filter
        self.ftype = 'butter'        # Butterworth
        self.output = 'ba'           # ba or tf type

    def run(self):
        # create timespec and data
        t = np.arange(0, self.sec, 1/self.fs)
        wave = self.A * np.sin(2.0 * np.pi * self.f * t)

        # create noise 
        noise = self.A_n * np.sin(2.0 * np.pi * self.f_n * t)

        # create wave with noise 
        wave2 = wave + noise

        # --- IIR filter
        b, a = signal.iirdesign(self.wp, self.ws, self.gpass, self.gstop, self.analog, self.ftype, self.output, self.fs)
        plotiirba(b, a, self.fs)

        # --- result
        wave3 = signal.lfilter(b, a, wave2)

        fig = plt.figure(figsize=(10, 3), tight_layout=True)
        ax = fig.add_subplot()
        ax.plot(wave2, label='raw')
        ax.plot(wave3, label='filt')
        ax.legend()
        ax.grid()
        plt.show()
        plt.savefig("iir.png")
    
if __name__ == '__main__':
    # butterworth filter
    butter_filter()
    # test an IIR filter
    proc = TestIirDesign()
    proc.run()        
    # execute SG on the spectral data
    outFigCSV(X,Y,paramAsLS, paramSG)
    # convolve
    convolve()
    # minimise
    m = minimize_func()
    r = m.res()
    m.print_res(r)
    # solve non-linear
    o = opt_nonlin()
    oo = o.solve()
    # solve ivp
    s = solv_ivp()
    s.soln()   
    s.plot_ivp 
    # matrix
    mm = matrix_math2x2(1.0,0.9,-0.3,0.1)
    mm.make2x2()
    mm.inver()
    mm.eigen()