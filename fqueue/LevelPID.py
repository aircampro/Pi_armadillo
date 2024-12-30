#!/usr/bin/env python
#
# ------------ PID loop with ADC and DAC -----------------
#
# Main specifications of LTC2450 on SPI 
#
# Number of bits; 16
# Number of channels; 1 channel
# Reference voltage; Common to power supply terminals
# Supply voltage; 2.7~5.5V
# Current consumption; 300uA
# Input; Single-ended
# Conversion method; ΔΣ (Delta・Sigma) method
# Sampling rate; 30sps
# Interface; SPI
#
import configparser
import pickle
import argparse

import spidev
import time
# Vref = 3.29989

# adafruit MCP4725 DAC on i2c
# sudo pip3 install adafruit-circuitpython-mcp4725
import board
import busio
import adafruit_mcp4725

# Optionally you can specify a different addres if you override the A0 pin.
# amp = adafruit_max9744.MAX9744(i2c, address=0x63)

import numpy as np
#import matplotlib.pyplot as plt
from scipy import signal

class MA_Filter():
    def __init__(self, master=None):
        print("MA_Filter")

    def character_MA(self, N, fs, a=1):
        b = np.ones(N) / N
        fn = fs / 2                                         # Nyquist frequency calculation

        w, h = signal.freqz(b, a)                           # Frequency response calculation
        x_freq_rspns = w / np.pi * fn
        y_freq_rspns = db(abs(h))                           # Convert complex numbers to decibels

        p = np.angle(h) * 180 / np.pi                       # phase characteristics
        w, gd = signal.group_delay([b, a])
        x_gd = w / np.pi * fn
        y_gd = gd

        return x_freq_rspns, y_freq_rspns, p, x_gd, y_gd

    def MA_filter(self, x, move_num):
        move_ave = []
        ring_buff = np.zeros(move_num)
        for i in range(len(x)):
            num = i % move_num
            if i < move_num:
                ring_buff[num] = x[i]
                move_ave.append(0)
            else:
                ring_buff[num] = x[i]
                ave = np.mean(ring_buff)
                move_ave.append(ave)
        return move_ave
        
class Bessel_Filter():
    def __init__(self, master=None):
        print("Bessel_Filter")

    def get_filtertype(self, selected_num):
        if selected_num == 0:
            ftype1 = "lowpass"
        elif selected_num == 1:
            ftype1 = "highpass"
        else:
            ftype1 = "bandpass"
        return ftype1

    def get_filterparam(self, dt, fl, fh, order, filtertype):
        dt = float(dt)

        fs = 1 / dt
        fn = fs / 2
        if filtertype == "bandpass":
            Wn = [float(fl)/fn, float(fh)/fn]
            N=order//2
        elif filtertype == "lowpass":
            Wn = float(fh)/fn
            N=order
        else:
            Wn = float(fl)/fn
            N=order
        return Wn, N

    def character_Bessel(self, dt, order, fl, fh, ftype):
        fs = 1 / dt
        fn = fs / 2                                                    # Nyquist frequency calculation
        Wn, N = self.get_filterparam(dt, fl, fh, order, ftype)
        b, a = signal.bessel(N, Wn, ftype)
        w, h = signal.freqz(b, a)                                      # Analog filter freqs, digital filter freqz

        x_freq_rspns = w / np.pi * fn
        y_freq_rspns = db(abs(h))                                      # Convert complex numbers to decibels

        p = np.angle(h) * 180 / np.pi                                  # phase characteristics

        w, gd = signal.group_delay([b, a])
        x_gd = w / np.pi * fn
        y_gd = gd

        return x_freq_rspns, y_freq_rspns, p, x_gd, y_gd

    def Bessel_filter(self, data, dt, order, fl, fh, ftype, worN=8192):
        fs = 1 / dt
        fn = fs / 2                                                       # Nyquist frequency calculation
        Wn, N = self.get_filterparam(dt, fl, fh, order, ftype)
        b, a = signal.bessel(N, Wn, ftype)

        zi = signal.lfilter_zi(b, a)
        z, _ = signal.lfilter(b, a, data, zi=zi*data[0])

        return z
        
class Chebyshev_Filter():
    def __init__(self, master=None):
        print("Chebyshev_Filter")

    def get_filtertype(self, selected_num):
        if selected_num == 0:
            ftype1 = "lowpass"
        elif selected_num == 1:
            ftype1 = "highpass"
        else:
            ftype1 = "bandpass"
        return ftype1

    def get_filterparam(self, dt, fl, fh, order, filtertype):
        dt = float(dt)

        fs = 1 / dt
        fn = fs / 2
        if filtertype == "bandpass":
            Wn = [float(fl)/fn, float(fh)/fn]
            N=order//2
        elif filtertype == "lowpass":
            Wn = float(fh)/fn
            N=order
        else:
            Wn = float(fl)/fn
            N=order
        return Wn, N

    def character_Chebyshev(self, dt, order, fl, fh, ripple, ftype):
        fs = 1 / dt
        fn = fs / 2                                                               # Nyquist frequency calculation
        Wn, N = self.get_filterparam(dt, fl, fh, order, ftype)
        b, a = signal.cheby1(N, ripple, Wn, ftype)
        w, h = signal.freqz(b, a)                                                 # Analog filter freqs, digital filter freqz

        x_freq_rspns = w / np.pi * fn
        y_freq_rspns = db(abs(h))                                                  # Convert complex numbers to decibels
 
        p = np.angle(h) * 180 / np.pi                                              # phase characteristics

        w, gd = signal.group_delay([b, a])
        x_gd = w / np.pi * fn
        y_gd = gd

        return x_freq_rspns, y_freq_rspns, p, x_gd, y_gd

    def Chebyshev_filter(self, data, dt, order, fl, fh, ripple, ftype):
        fs = 1 / dt
        fn = fs / 2                                                                # Nyquist frequency calculation
        Wn, N = self.get_filterparam(dt, fl, fh, order, ftype)
        b, a = signal.cheby1(N, ripple, Wn, ftype)

        zi = signal.lfilter_zi(b, a)
        z, _ = signal.lfilter(b, a, data, zi=zi*data[0])

        return z
        
def butter_lowpass(lowcut, fs, order=4):
    "'Function to design Butterworth low-pass filter'"
    nyq = 0.5 * fs
    low = lowcut / nyq
    b, a = signal.butter(order, low, btype='low')
    return b, a

def butter_lowpass_filter(x, lowcut, fs, order=4):
    "'Function to lowpass filter data'"
    b, a = butter_lowpass(lowcut, fs, order=order)
    y = signal.filtfilt(b, a, x)
    return y
    
# Calibration values for the level gauge
CAL_FACTOR = 100.0                                                     # Adjust this based on your calibration data
ZERO_LEVEL_V = 0.1                                                     # Adjust this based on your calibration data
RAW_COUNTS = 65535                                                     # 16-bit ADC
REF_V = 3.3                                                            # with a 3.3V reference voltage

# filters
N = 30                                                                 # Number of samples
dt = 1.0/30.0                                                          # Sampling period [s]
fs = 1 / dt

# chev
ftype = 0                                                              # lowpass=0, highpass=1, bandpass=2
flc = None                                                             # Cut-off frequency (cut-off frequency) Low frequency side
fh = 20                                                                # Cut-off frequency (cut-off frequency) High frequency side
order = 4                                                              # order
attenuation = 5                                                        # [dB])

# besel
ftypeB = 0                                                             # lowpass=0, highpass=1, bandpass=2
flb = None                                                             # Cut-off frequency (cut-off frequency) Low frequency side
fhb = 20                                                               # Cut-off frequency (cut-off frequency) High frequency side
orderb = 4 

# MA filter
move_num = 5

class PidController():
    """A classical PID controller which maintains state between calls.

    This class is intended to be integrated into an external control loop. It
    remembers enough of the state history to compute integral and derivative
    terms, and produces a combined control signal with the given gains.
    """

    def __init__(self,
                 kp: float,
                 ki: float,
                 kd: float,
                 target: float,
                 initial_state: float,
                 acc_error: float,
                 out_start: float,
                 fwd_rev: int,
                 t_0: int) -> None:
        """Create a PID controller with the specified gains and initial state.

        Parameters
        ----------
        kp, ki, kd    : The PID control gains.
        target        : The desired system state, also called a "setpoint".
        initial_state : The starting state of the system.
        t_0           : The starting time.
        """
        # Gains for the proportional, integral, and derivative terms.
        self._kp: float = kp
        self._ki: float = ki
        self._kd: float = kd


        self._target: float = target                                              # The target state which the controller tries to maintain.
        self._remote_spt = 0                                                      # remote set-point for cascade mode
        self._remote_local = 0                                                    # 0=single loop 1=cascade loop
        self._accumulated_error: float = acc_error                                # Tracks the integrated error over time. This starts at 0 as no time has passed.
        self._acc_max = 5000                                                      # limit on the accumulator prevent wind-up
        self._last_error: float = initial_state - target                          # Tracks the previous sample's error to compute derivative term.
        self._last_t: int = t_0                                                   # Tracks the previous sample time point for computing the d_t value used in I and D terms.
        self._term: float = 0.0
        self._out: float = out_start                                              # output
        self._mode: int = 1                                                       # mode auto/manual
        self._rev: int = fwd_rev                                                  # forward or reverse acting PID
        self._i_mode: int = 0                                                     # mode to calculate integral term
        self.state = 0
        
    def next(self, t: int, state: float, deadb: float) -> float:
        """Incorporate a sample of the state at time t and produce a control value.

        Because the controller is stateful, calls to this method should be
        monotonic - that is, subsequent calls should not go backwards in time.

        Parameters
        ----------
        t     : The time at which the sample was taken.
        state : The system state at time t.
        deadb : do nothing if in the deadband or -1 is ignore
        """
        self.state = state 
        if self._mode == 1 and deadb == -1.0 :
            if self._remote_local == 0: 
                error = self.state - self._target
            elif self._remote_local == 1:
                error = self.state - self._remote_spt 
            #d_t = (t - self._last_t)
            d_t = t
            p = self._proportional(error)
            i = self._integral(d_t, error)
            d = self._derivative(d_t, error)
            self._last_t = t
            self._last_error = error
            self._term = (p + i + d)
            if self._rev == 1:
                self._out = self._out - self._term
            else:
                self._out = self._out + self._term
        if self._mode == 1 and deadb >= 0.0 :
            if self._remote_local == 0: 
                error = self.state - self._target
            elif self._remote_local == 1:
                error = self.state - self._remote_spt
            if abs(error) > deadb:
                #d_t = (t - self._last_t)
                d_t = t 
                p = self._proportional(error)
                i = self._integral(d_t, error)
                d = self._derivative(d_t, error)
                self._last_t = t
                self._last_error = error
                self._term = (p + i + d)
                if self._rev == 1:
                    self._out = self._out - self._term
                else:
                    self._out = self._out + self._term
            else:
                self._last_t = t
                self._last_error = error
                self._accumulated_error = 0                 
        elif self._mode == 0:
            if self._remote_local == 0: 
                error = self.state - self._target
            elif self._remote_local == 1:
                error = self.state - self._remote_spt            
            self._last_error = error
            
        return self._term, self._out

    def set_auto(self) :
        self._mode = 1

    def set_manual(self) :
        self._accumulated_error = 0                                     # reset acc error if we go into manual state
        self._mode = 0
        
    def set_output(self, v) :
        set_manual()
        self._out = v 
        if self._remote_local == 0:        
            error = self.state - self._target
        elif self._remote_local == 1:
            error = self.state - self._remote_spt
        self._last_error = error
            
    def set_spt(self, sp) :
        self._target = sp

    def set_rspt(self, rsp) :
        self._remote_spt = rsp

    def set_remote(self):
        self._remote_local = 1    

    def set_local(self):
        self._remote_local = 0 
        
    def set_p(self, p) :
        self._kp = p

    def set_i(self, i) :
        self._ki = i

    def set_d(self, d) :
        self._kd = d

    def get_ac(self):
        return self._accumulated_error   
        
    def _proportional(self, error: float) -> float:
        return self._kp * error

    def _integral(self, d_t: float, error: float) -> float:
        if self._i_mode == 0:
            # The constant part of the error.
            base_error = min(error, self._last_error) * d_t
            # Adjust by adding a little triangle on the constant part.
            error_adj = abs(error - self._last_error) * d_t / 2.0
            if self._accumulated_error >= self._acc_max:
                self._accumulated_error += base_error + error_adj        
            return self._ki * self._accumulated_error
        elif self._i_mode == 1:
            return self._ki * d_t * error
            
    def _derivative(self, d_t: float, error: float) -> float:
        d_e = (error - self._last_error)
        if d_t > 0:
            return self._kd * (d_e / d_t)
        else:
            return 0

# PID Params
TARGET_LEVEL = 50.0                                                                          # required level
PBAND_STR=1.0                                                                                # p i d params
INT_STR=0.2
DER_STR=0.1
OUT_H=4095                                                                                   # pid out scale
OUT_L=100
PID_USES_FILTER=1                                                                            # 0=raw 1=butterworth, 2=chevyshev 3=besel 4=Moving avagerge
ACC_ERR = 0                                                                                  # accumulated error start point
OUT_ST = 0
DEADBAND=-1.0                                                                                # define control deadband or -1.0 for none
C_ACTION=0                                                                                   # forward or reverse action
PID_LOOP_TIME=1                                                                              # pid loop time in seconds
PID_OT=1                                                                                     # choose output=1 or term=0

# class of level controller to be called from another program
#
class levelPID():

    def __init__(self, cFlag=1, config_file_name="config.ini" ):

        # Initialize I2C bus.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # Initialize MCP4725. and create object
        self.dac = adafruit_mcp4725.MCP4725(self.i2c)   
        # initialize the spi device and create an object
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)                                                                        # port 0,cs 0
        self.spi.max_speed_hz = 1000000                                                           # speed 1MHz

        config_ini = configparser.ConfigParser()                                                  # read the config.ini for various configurations
        config_ini.read(config_file_name, encoding='utf-8')
        sections = config_ini.sections()                                                          # check if we have the PID section included
        for sec in sections:
            if sec == "PID":
                PBAND_STR = float(config_ini['PID']['PB'])
                INT_STR = float(config_ini['PID']['IN'])
                DER_STR = float(config_ini['PID']['DE'])
                OUT_H = float(config_ini['PID']['OH'])                                                                                
                OUT_L = float(config_ini['PID']['OL'])
                PID_USES_FILTER = float(config_ini['PID']['FC'])
                TARGET_LEVEL = float(config_ini['PID']['SP'])
                OUT_ST = float(config_ini['PID']['OP'])
                DEADBAND = float(config_ini['PID']['DB'])
                C_ACTION = float(config_ini['PID']['CA'])
                PID_LOOP_TIME = float(config_ini['PID']['LT'])
                PID_OT = int(config_ini['PID']['OT'])
            elif sec == "CALIB":
                CAL_FACTOR = float(config_ini['CALIB']['CF'])
                ZERO_LEVEL_V = float(config_ini['CALIB']['ZF'])
                RAW_COUNTS = float(config_ini['CALIB']['RC'])
                REF_V = float(config_ini['CALIB']['RV'])

        x = []                                                                                    # initialise an array to store the readings from the DAC 
        # chev filter
        self.CFilter = Chebyshev_Filter()
        self.ftype = CFilter.get_filtertype(ftype)
        # besel
        self.BeFilter = Bessel_Filter()
        self.ftypeB = BeFilter.get_filtertype(ftypeB)
        # moveing average
        self.MFilter = MA_Filter()
        # Create our PID controller with some initial values for the gains. or if commanded from cmd line load the last known state from the saved pickle file
        if cFlag == 1:
            print("\033[34m re-loading saved PID controller! \033[0m")
            with open('pid.pickle', 'rb') as f:
                self.controller = pickle.load(f)
        else:
            self.controller = PidController(PBAND_STR, INT_STR, DER_STR, TARGET_LEVEL, TARGET_LEVEL, ACC_ERR, OUT_ST, C_ACTION, 0)

    def __del__(self):
        self.spi.close()
        
    # read LTC2450 ADC on SPI
    def adc_read_u16(self):
        ret = 0
        dataV = 0
        try:
            adc = self.spi.xfer2([0x00,0x00])
            dataV = (adc[0] << 8) | adc[1]
        except :
            self.spi.close() 
            time.sleep(0.1)
            self.spi.open(0,0)                                                  # port 0,cs 0
            self.spi.max_speed_hz = 1000000                                     # speed 1MHz  
            try:
                adc = self.spi.xfer2([0x00,0x00])
                dataV = (adc[0] << 8) | adc[1]  
            except : 
                ret = 1            
        return ret, dataV

    # calculate level from raw DAC reading
    def read_level(self):
        ret, raw_value = self.adc_read_u16()                                         # Read the raw analog value
        level = 0
        if ret == 0:
            voltage = (raw_value / RAW_COUNTS) * REF_V                         # Convert the raw value to voltage                                      
            level = (voltage - ZERO_LEVEL_V ) * CAL_FACTOR                     # Convert voltage to level using calibration data

        return ret, level

    # set adafruit MCP4725 DAC on i2c
    def set_DAC(self, raw, rwarng):
        dacV = (raw / rwarng) * 4095
        self.dac.raw_output = round(dacV)
    
    def pid_man(self):
        self.controller.mode = 0

    def pid_aut(self):
        self.controller,mode = 1
        
    def run_pid_loop(self):
    
        st_time = time.time()                                                                      # start timer
        loopExit = 0                                                                               # set infinite loop until you interrupt and type x
        while loopExit == 0:
            try:    
                while 1:  
                    ret, strn = self.read_level() 
                    if ret == 0:            
                        x.append(strn)
                    if len(x) == round(30*PID_LOOP_TIME):                                           # we have 30 samples in 1second i.e 1s for 30sps
                        now_time = time.time()                                                      # end PID sampling time
                        # butterworth
                        y0 = butter_lowpass_filter(x, 100, fs, order=4)
                        # chebyshev               
                        y1 = self.CFilter.Chebyshev_filter(x, dt, order, flc, fh, attenuation, ftype)
                        x_freq_rspns_ch, y_freq_rspns_ch, p_ch, x_gd_ch, y_gd_ch = self.CFilter.character_Chebyshev(dt, order, flc, fh, ripple, ftype)
                        # besel
                        y2 = self.BeFilter.Bessel_filter(x, dt, orderb, flb, fhb, ftypeB)
                        x1_freq_rspns_be, y1_freq_rspns_be, p_be, x_gd_be, y_gd_be = self.BeFilter.character_Bessel(dt, orderb, flb, fhb, ftypeB) 
                        # MA Filter
                        y3 = self.MFilter.MA_filter(x, move_num) 
                
                        # now plot the raw and filtered data
                        #fig, ax = plt.subplots()
                        #t = np.arange(0, N * dt, dt)                                           # timespan for plot
                        #ax.plot(t, x,  c="blue", label='raw data')
                        #ax.plot(t, y0, c="red", label='butterworth filtered')
                        #ax.plot(t, y1, c="green", label='chevyshev filtered')
                        #ax.plot(t, y2, c="cyan", label='besel filtered')
                        #ax.plot(t, y3, c="yellow", label='Moving Avg filtered')
                        #ax.set_xlabel("Time [s]")
                        #ax.set_ylabel("Signal")
                        #ax.set_xlim([0, 0.1])
                        #ax.grid()
                        #plt.legend(loc='strain gauage readings with filters')
                        #plt.show()            

                        # example read ADC pass through a butterworth filter and then pass the result to PID with this scaled result being sent to a DAC
                        #
                        filter_for_pid = [x, y0, y1, y2, y3]                        # list available filters
                        chosen_filter = filter_for_pid[PID_USES_FILTER]             # choose filter 0 which is butterworth
                        sum_y = 0
                        for i in range(0, len(chosen_filter)):
                            sum_y += chosen_filter[i]
                        str_avg = sum_y / len(chosen_filter) 
                        t = (now_time - st_time) 
                        # if you want to use a fixed sample time then make t = 100 it might be better                
                        outTerm, outPID = self.controller.next(t, str_avg, DEADBAND)  
                        choice_of_op = [ outTerm, outPID ]                     
                        if choice_of_op[PID_OT] > OUT_H:
                            choice_of_op[PID_OT] = OUT_H
                        elif choice_of_op[PID_OT] < OUT_L:
                            choice_of_op[PID_OT] = OUT_L 
                        self.set_DAC(choice_of_op[PID_OT], OUT_H)                    
                    # reset x
                        x = []
                        st_time = time.time()
                    
            except KeyboardInterrupt:
                v=input("\033[32m enter p= i= d= s= e.g. p=32 a=auto o= or x to end anything else returns back to reading \033[0m")
                if not v.find("p") == -1:
                    self.controller.set_p(float(v.split("=")[1]))  
                elif not v.find("i") == -1:
                    self.controller.set_i(float(v.split("=")[1])) 
                elif not v.find("d") == -1:
                    self.controller.set_d(float(v.split("=")[1])) 
                elif not v.find("s") == -1:
                    self.controller.set_spt(float(v.split("=")[1])) 
                elif not v.find("o") == -1:
                    self.controller.set_output(float(v.split("=")[1])) 
                elif not v.find("a") == -1:
                    self.controller.set_auto() 
                elif not v.find("x") == -1 or not v.find("X") == -1:
                    print("stop of ADC reader....")
                    loopExit = 1
                else:            
                    print("returning to reading ADC....")
        
        self.spi.close()
        # dump the controller state 
        with open('pid.pickle', 'wb') as f:
            pickle.dump(controller, f)