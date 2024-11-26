#!/usr/bin/env python
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
import spidev
import time
# Vref = 3.29989

# initialize the spi device
spi = spidev.SpiDev()
spi.open(0,0)                                            #port 0,cs 0
spi.max_speed_hz = 1000000                               #speed 1MHz

import numpy as np
import matplotlib.pyplot as plt
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
 "'Function to design Butterworth low-pass filter
'
    nyq = 0.5 * fs
    low = lowcut / nyq
    b, a = signal.butter(order, low, btype='low')
    return b, a

def butter_lowpass_filter(x, lowcut, fs, order=4):
 "'Function to lowpass filter data
'
    b, a = butter_lowpass(lowcut, fs, order=order)
    y = signal.filtfilt(b, a, x)
    return y
 
def adc_read_u16():
    ret = 0
    dataV = 0
    try:
        adc = spi.xfer2([0x00,0x00])
        dataV = (adc[0] << 8) | adc[1]
    except :
        spi.close() 
        time.sleep(0.1)
        spi.open(0,0)                                                  # port 0,cs 0
        spi.max_speed_hz = 1000000                                     # speed 1MHz  
        try:
            adc = spi.xfer2([0x00,0x00])
            dataV = (adc[0] << 8) | adc[1]  
        except : 
            ret = 1            
    return ret, dataV
    
# Calibration values for the strain gauge
CAL_FACTOR = 10.0                                                      # Adjust this based on your calibration data
ZERO_FORCE_V = 0.5                                                     # Adjust this based on your calibration data
RAW_COUNTS = 65535                                                     # 16-bit ADC
REF_V = 3.3                                                            # with a 3.3V reference voltage

# filters
N = 30                                                                 # Number of samples
dt = 1/30                                                              # Sampling period [s]
fs = 1 / dt
t = np.arange(0, N * dt, dt)                                           # timespan for plot

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

def read_strain():
    ret, raw_value = adc_read_u16()                                         # Read the raw analog value
    force = 0
    if ret == 0:
        # Convert the raw value to voltage
        voltage = (raw_value / RAW_COUNTS) * REF_V                                 

        # Convert voltage to force using calibration data
        force = (voltage - ZERO_FORCE_V ) * CAL_FACTOR

    return ret, force

if __name__ == "__main__": 

    x = [] 
    # chev filter
    CFilter = Chebyshev_Filter()
    ftype = CFilter.get_filtertype(ftype)
    # besel
    BeFilter = Bessel_Filter()
    ftypeB = BeFilter.get_filtertype(ftypeB)
    # moveing average
    MFilter = MA_Filter()
    try:    
        while 1:  
            ret, strn = read_strain() 
            if ret == 0:            
                x.append(strn)
            if len(x) == 30:                                                                # we have 30 samples
                # butterworth
                y0 = butter_lowpass_filter(x, 100, fs, order=4)
                # chebyshev               
                y1 = CFilter.Chebyshev_filter(x, dt, order, flc, fh, attenuation, ftype)
                x_freq_rspns_ch, y_freq_rspns_ch, p_ch, x_gd_ch, y_gd_ch = CFilter.character_Chebyshev(dt, order, flc, fh, ripple, ftype)
                # besel
                y2 = BeFilter.Bessel_filter(x, dt, orderb, flb, fhb, ftypeB)
                x1_freq_rspns_be, y1_freq_rspns_be, p_be, x_gd_be, y_gd_be = BeFilter.character_Bessel(dt, orderb, flb, fhb, ftypeB) 
                # MA Filter
                y3 = MFilter.MA_filter(x, move_num) 
                
                # now plot the raw and filtered data
                fig, ax = plt.subplots()
                ax.plot(t, x,  c="blue", label='raw data')
                ax.plot(t, y0, c="red", label='butterworth filtered')
                ax.plot(t, y1, c="green", label='chevyshev filtered')
                ax.plot(t, y2, c="cyan", label='besel filtered')
                ax.plot(t, y3, c="yellow", label='Moving Avg filtered')
                ax.set_xlabel("Time [s]")
                ax.set_ylabel("Signal")
                ax.set_xlim([0, 0.1])
                ax.grid()
                plt.legend(loc='strain gauage readings with filters')
                plt.show()            
            
                # reset x
                x = []
        #time.sleep(0.1)
    except KeyboardInterrupt:
        print("stop of ADC reader test")
        
spi.close()