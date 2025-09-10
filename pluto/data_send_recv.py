# SDR Example of sending SCD30 i2c data via analog devices pluto
#
import numpy as np
from math import sqrt
import random

# SDR Pluto
import adi
import uhd

# data SCD30 on i2c
from scd30_i2c import SCD30
import time
import pytz
from datetime import datetime
import os

# Scale a input stream to an normalized output ranged 0-1 the max min of the input stream shall be in the data...
# e.g. list = [ 0, 500, data] for data in the range 0-500
#
def norm_list(nums):
    # Find the minimum and maximum values in the list and assign them to variables 'a' and 'b'
    a = min(nums)
    b = max(nums)
    if b - a == 0:                                       # Check if the range between the minimum and maximum values is zero
        return [0.0] + [1.0] * (len(nums) - 1)           # If the range is zero, return a list with 0.0 as the first element and 1.0 for the remaining elements
    for i in range(len(nums)):                           # Iterate over the indices of the list
        nums[i] = (nums[i] - a) / (b - a)                # Rescale and shift each element in the list to cover the range [0, 1]		
    return nums                                          # Return the modified list

# rescale a value from a input range to a new output range
# 
def rescale(val, in_min, in_max, out_min, out_max):
    rs = out_min + (val - in_min) * ((out_max - out_min) / (in_max - in_min))
    return rs

# The PlutoSDR expects samples to be between -2^14 and +2^14, not -1 and +1 like some SDRs
def set_pluto_range(listv):
    listnew = []
    for i in range(0,len(listv)):
        a = int(listv[i]) 
        a *= 2**14 
        listnew.append(a)
    return listnew
 
# The PlutoSDR expects samples to be between -2^14 and +2^14, not -1 and +1 like some SDRs
def normalize_pluto_range(listv):
    listnew = []
    for i in range(0,len(listv)):
        a = int(listv[i]) 
        a /= 2**14
        listnew.append(int(a))
    return listnew

# normalise analog values
#
def norm_analog_list(ll):
    nal = []
    for l in ll:
        if l >= 0.5:
            nal.append(1)
        else:
            nal.append(0)
    return nal

# convert ascii alpha numeric char to binary list for transmission
#
def ascii_to_bin(chv, bitsnum=16):
    f=bin(ord(chv))[2:]
    padded_num = str(f).rjust(bitsnum, '0')
    m = []
    for c in padded_num:
        m.append(c)
    return m

# convert binary number list to a string representing the binary
#
def convert_list_to_str(binar_list):
    s = ""
    for g in binar_list:
        s += str(g)
    return s

# convert binary string to a ascii equivalant
#
def convert_bin_to_ascii(padded_num):	
    o = "0b" + str(padded_num)
    a = chr(int(o, 2))
    return a

# ================ QPSK modulation ========================
#
A=1                                                     # number of repeats (we use N_SEND 100 so i will set it to 1 here
B=16                                                    # sending 16 [1 or 0] states for each ascii characture
t = np.linspace(0,1,A)                                  # Time
tb = 1;
fc = 1;                                                 # carrier frequency
c1 = sqrt(2/tb)*np.cos(2*np.pi*fc*t)                    # carrier frequency cosine wave
c2 = sqrt(2/tb)*np.sin(2*np.pi*fc*t)                    # carrier frequency sine wave
m = []
t1 = 0;
t2 = tb;
# Get Your send data and write each value to a 16 bit array m, modulate and send it.

# QPSK modulation input list [0 to 1] output list [-2 to 2]
#
def qpsk_mod(m):
    t1 = 0;
    t2 = tb;
    odd_sig = np.zeros((B,A))
    even_sig = np.zeros((B,A))
    for i in range(0,B-1,2):
        t = np.linspace(t1,t2,A)
        if (m[i]>0.5):
            m[i] = 1
            m_s = np.ones((1,len(t)))
        else:
            m[i] = 0
            m_s = (-1)*np.ones((1,len(t)))
        odd_sig[i,:] = c1*m_s
        if (m[i+1]>0.5):
            m[i+1] = 1
            m_s = np.ones((1,len(t)))
        else:
            m[i+1] = 0
            m_s = (-1)*np.ones((1,len(t)))
        even_sig[i,:] = c2*m_s
        qpsk = odd_sig + even_sig                              # modulated wave = oddbits + evenbits
        t1 = t1 + (tb+0.01)
        t2 = t2 + (tb+0.01)
    return qpsk

# for testing ans read back of modulated value channel = qpsk 
# we shall read it back 

## demodulation
#
def qpsk_demod(channel):
    t1 = 0
    t2 = tb
    demod = np.zeros((B,1))    # demodulated signal  (demodulation of noise + qpsk modulated wave)
    for i in range(0,B-1,1):
        t = np.linspace(t1,t2,A)
        x1 = sum(c1*channel[i,:])
        x2 = sum(c2*channel[i,:])
        if(x1>0 and x2>0):
            demod[i] = 1
            demod[i+1] = 1
        elif (x1>0 and x2<0):
            demod[i] = 1
            demod[i+1] = 0
        elif (x1<0 and x2<0):
            demod[i] = 0
            demod[i+1] = 0
        elif (x1<0 and x2>0):
            demod[i] = 0
            demod[i+1] = 1
        t1 = t1 + (tb+0.01)
        t2 = t2 + (tb+0.01)
    return demod

PLUTO_IPA = "ip:192.168.2.1"
N_SEND = 100                                                      # no of times we send a char
QPSK = 0
T_MAX = 50.0
TDB = 2.0

# sends a char via sdr
#
def send_a_char(v, sdr):
    # send the data
    sendbin = ascii_to_bin(v) 
    if QPSK == 1:                              # enable QPSK modulation (which reduces bandwidth)
        sendbin = qpsk_mod(sendbin)            # range -2 to 2
        sendbin = rescale(val, -2, 2, 0, 1)    # re-scale 0 to 1                           
    psend = set_pluto_range(sendbin)           # re-scale to pluto range
    # Transmit our batch of samples N_SEND times, so it should be 1 second worth of samples total, if USB can keep up
    for i in range(N_SEND-1):                  # N_SEND minus the first char denoting what type of data it is
        sdr.tx(psend)                          # transmit the data from the tx buffer to the sdr

# process the data received in 16 bit blocks
#
def ptocess_char(v):
    binv = normalize_pluto_range(v[:16])       # re-scale 0 to 1 pluto data 16 bits in length
    binv = norm_analog_list(binv)              # fix analog values to 0 or 1
    if QPSK == 1:                              # enable QPSK modulation
        binv = rescale(binv, 0, 1, -2, 2)      # re-scale -2 to 2
        binv = qpsk_demod(binv)                # range -2 to 2
        binv = rescale(binv, -2, 2, 0, 1)      # re-scale 0 to 1
    ss = convert_list_to_str(binv)                           
    asv = convert_bin_to_ascii(ss)
    return asv

# receives the chars via the sdr
#
def rcv_data(sdr):
    v = sdr.rx()                                   # rcv the data from the sdr rx buffer
    len_read = len(v)                              # store length of data received
    asv_a = [] 
    try:    
        asv = ptocess_char(v)
        asv_a.append(asv)                          # store the value we got in the result list
        len_read = len_read - 16
        while len_read > 0:                        # still have data in the buffer
            v = v[16:]                             # remove the bits (word) we just processed 
            asv = ptocess_char(v)
            asv_a.append(asv) 
            len_read = len_read - 16   
    except:
        len_read = 0                              # we failed so exit
        pass    
    return asv_a

if __name__ == "__main__":

    # scd30 sensor set-up
    try:
        scd30 = SCD30()
        scd30.set_measurement_interval(10)
        scd30.start_periodic_measurement()
        time.sleep(2)
    except TimeoutError as e:
        print(e)
        os.system('exit')
        pass

    # ========== SDR Parameters ========== 
    sample_rate = 16*N_SEND                                      # Hz
    center_freq = 915e6                                          # Hz
    num_samps = 16*N_SEND   
    sdr = adi.Pluto(PLUTO_IPA)                                   # connect to pluto
    sdr.sample_rate = int(sample_rate)
    # set pluto as transmitter  
    sdr.tx_rf_bandwidth = int(sample_rate)                       # filter cutoff, just set it to the same as sample rate
    sdr.tx_lo = int(center_freq)
    sdr.tx_hardwaregain_chan0 = -50                              # Increase to increase tx power, valid range is -90 to 0 dB
    # set pluto as same receiver
    sdr.rx_lo = int(center_freq)
    sdr.rx_rf_bandwidth = int(sample_rate)
    sdr.rx_buffer_size = num_samps
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = 0.0                              # dB, increase to increase the receive gain, but be careful not to saturate the ADC

    # pluto GPIO which is controlled from the trip amp on the temperature
    # First, defining some of the terms Ettus uses. CTRL sets whether the pin is controlled by ATR (automatic), 
    # or by manual control only (1 for ATR, 0 for manual). DDR (Data Direction Register) defines whether or not a GPIO is an output (0) 
    # or an input (1). OUT is used to manually set the value of a pin (only to be used in manual CTRL mode).
    usrp = uhd.usrp.MultiUSRP()
    usrp.set_gpio_attr('FP0A', 'CTRL', 0x000, 0xFFF)
    usrp.set_gpio_attr('FP0A', 'DDR', 0xFFF, 0xFFF)

    while True:
        # if scd30 isn't ready, wait 0.2sec
        try:
            if scd30.get_data_ready():
                m = scd30.read_measurement()                                                     # get the data from the sensor
                if m is not None:
                    tim = '"timestamp":"'+datetime.now(pytz.timezone('GMT')).strftime('%Y-%m-%d %H:%M:%S.%f')+'"'
                    temp = '"' + "temp(degree)" + '"' + ":" + '"' + str(round(m[1]-3.5,3)) + '"'
                    hum = '"' + "humid(%)" + '"' + ":" + '"' + str(round(m[2],3)) + '"'
                    co2 = '"' + "co2(ppm)" + '"' + ":" + '"' + str(round(m[0],3)) + '"'
                    mylist = [tim, temp, hum, co2]
                    mystr = '{' + ','.join(map(str, mylist))+'}'
                    if str(round(m[1],3))!="nan":
                        print(mystr)
                        send_a_char("T", sdr)                                                    # send the temperature data
                        t = str(round(m[1]-3.5,3)    
                        for q in t:
                            send_a_char(q, sdr)       
                    if str(round(m[2],3))!="nan":
                        send_a_char("H", sdr)                                                    # send the humidity data
                        h = str(round(m[2]-3.5,3)    
                        for q in h:
                            send_a_char(q, sdr)      
                    if str(round(m[0],3))!="nan":
                        send_a_char("C", sdr)                                                    # send the co2 data
                        co2 = str(round(m[0]-3.5,3)    
                        for q in co2:
                            send_a_char(q, sdr) 
                    for q in tim:
                        send_a_char(q, sdr)                                                      # send the timestamp data 
                    if t > T_MAX:                                                                # set GPIO output for temperature trip amp
                        usrp.set_gpio_attr('FP0A', 'OUT', 0xFFF, 0xFFF)
                    elif t > (T_MAX-TDB): 
                        usrp.set_gpio_attr('FP0A', 'OUT', 0x000, 0xFFF)                    
                    time.sleep(1)
                    else:
                        print("value is nan")
                        time.sleep(1)
            else:
                cha = rcv_data(sdr)                                                              # read the data received
                print(cha)
                time.sleep(0.01)
                # time.sleep(0.2)
        except OSError as e:
            print(e)
            pass