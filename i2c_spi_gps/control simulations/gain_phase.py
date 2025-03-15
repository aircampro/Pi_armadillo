#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Example plotting gain and phase
#
import control
from control import matlab
import numpy as np
import matplotlib.pyplot as plt
import math
import cmath

if __name__ == "__main__":

    tf = matlab.tf([1],[1 , 0])
    w_max = 1000
    w_min = 0.01
    w = np.linspace(w_min,w_max,10000000)
    fr = matlab.evalfr(tf,1j*w)                  #s = jw　Get the result of jw substitution The frequency response of the system

    gain = abs(fr)

    phase = []
    for i in fr:
        phase.append(cmath.phase(i))   
    phase = np.array(phase)                      # make array of the phases

    # plot results
    #
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
    ax2.set_ylim(-np.pi-0.1,np.pi+0.1)

    # show plots
    fig.tight_layout()