#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# sudo apt-get install python-pip
# sudo apt-get install gfortran
# sudo apt-get install libblas-dev libatlas-dev liblapack-dev
# pip install control
# pip install slycot
# conda install -c conda-forge control slycot
#
# simple pid simulation for various parameters read from ini file
#
import math
import control
import numpy as np
import matplotlib.pyplot as plt
%matplotlib inline
import configparser

# default parameters
#
gain=2
tc=20
dt=5
bias=13.5
Kp = 1
Ki = 0.05
Kd = 1.5

# read config
#
try:
    config_ini = configparser.ConfigParser()                                                  # read the pid_sim.ini for various configurations
    config_ini.read('pid_sim.ini', encoding='utf-8')
    sections = config_ini.sections()                                                          # check if we have the PID section included
    for sec in sections:
        if sec == "PID":
            gain = float(config_ini['PID']['GAIN'])
            tc = float(config_ini['PID']['TC'])
            dt = float(config_ini['PID']['DT'])
            bias = float(config_ini['PID']['BIAS'])                                                                                
            Kp = float(config_ini['PID']['KP'])
            Kd = float(config_ini['PID']['KD'])
            Ki = float(config_ini['PID']['KI'])
            iKp = float(config_ini['PID']['IKP'])                                             # use inverts
            iKd = float(config_ini['PID']['IKD'])
            iKi = float(config_ini['PID']['IKI'])
            if iKp == 1:
                Kp = 1.0 / Kp			
            if iKi == 1:
                Ki = 1.0 / Ki	
            if iKd == 1:
                Kd = 1.0 / Kd	
except:
    print("failed to load config from file using program constants")

def do_sim():	
    # First-order plus deadtime system
    num=np.array([gain])
    den=np.array([tc,1])
    #Add dead time
    n_pade = 10
    (num_pade,den_pade) = control.pade(dt,n_pade)
    H_pade=control.tf(num_pade,den_pade)
    H_noDelay = control.tf(num,den)
    H = control.series(H_pade,H_noDelay)
    # PID controller
    C = control.tf([Kd, Kp, Ki], [1, 0])
    # First-order plus deadtime system and PID Controller
    G = control.feedback(H*C, 1)
    t = np.linspace(0, 100, 100)
    t, pv = control.step_response(G, t)

    # set sp to 1.0
    sp=np.ones(len(pv))

    # Plot the results
    plt.plot(t, sp, label='SP')
    plt.plot(t, pv, label='PV')
    plt.legend()
    plt.show()
	
if __name__ == "__main__": 

    do_sim()
	
