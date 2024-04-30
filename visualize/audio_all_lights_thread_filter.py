#!/usr/bin/env python3
# -*- coding: utf-8 -+-
# 
# a program for listening to a sound grabbing the bpm of that sound and setting the lights in time with it
# version modified to be multi-threded to keep lights running while sampling new data then set the new frequency
#

# for recording the microphone
# pip install SoundCard
# pip install SoundFile
import soundcard as sc
import soundfile as sf

# sudo apt-get install -y git openssl libssl-dev libbz2-dev libreadline-dev libsqlite3-dev
# sudo apt-get install portaudio19-dev
# sudo apt-get install python3-pyaudio
#
import signal
import sys
import pyaudio
import wave

# The "Button SHIM" used this time is a thin expansion board for Raspberry Pi equipped with 5 buttons and RGB status LEDs.
# install it with sudo apt-get install python3-buttonshim
import buttonshim
BUTTONS = [buttonshim.BUTTON_A, buttonshim.BUTTON_B, buttonshim.BUTTON_C, buttonshim.BUTTON_D, buttonshim.BUTTON_E]

# for bpm reading
#
import numpy as np
import librosa

import struct
import math
import os
from scipy import fromstring, int16
# comment out this if you dont want to plot the results
import matplotlib.pyplot as plt 

# for the lighting system
from xknx.remote_value import RemoteValueColorRGBW
import time
import threading
import ctypes

bpm_min, bpm_max = 60, 240

output_file_name = "out.wav"

# Globals used by the handler
SET_ACTIVE = 0
SET_PATTERN = 0
NUM_PATTERNS = 5
PERIOD_BIAS = 1
MAX_PERIOD_BIAS = 10
LIGHT_DLY=0.5                      # default 120 bpm

# boot up to use the last known time setting if it was calculated previously
LIGHT_DLY_FL="light_duration.txt"
try:
    with open(LIGHT_DLY_FL, "r") as f:
        while True:
            line = f.readline()
            if not line:
                break
            else:
                try:
                    LIGHT_DLY = float(line.split("\n")[0])
                    break
                except:
                    print("error : parsing the data saved")                
except:
    print("warning : no light_duration file has been saved to disk")
    
def get_BPM(filepath, duration = 30, x_sr = 200):

    # Read music signal    
    # y, sr = librosa.load(filepath, offset=38, duration=duration, mono=True)
    y, sr = librosa.load(filepath, offset=38, duration=duration)   
    
    # Generate beat detection signals
    # Resampling & Power signal extraction
    x = np.abs(librosa.resample(y, sr, x_sr)) ** 2
    x_len = len(x)

    # Generate a complex sinusoidal matrix for each BPM
    M = np.zeros((bpm_max, x_len), dtype=np.complex)
    for bpm in range(bpm_min, bpm_max): 
        thete = 2 * np.pi * (bpm/60) * (np.arange(0, x_len) / x_sr)
        M[bpm] = np.exp(-1j * thete)

    # Calculation of matching degree with each BPM
    # (inner product of complex sine wave matrix and beat detection signal)
    x_bpm = np.abs(np.dot(M, x))

    # Calculate BPM
    bpm = np.argmax(x_bpm)
    return bpm

# get the tempo bpm by another method of librosa

def cut_wav(filename, time):
    # open the wav file
    wavf = filename + '.wav'
    wr = wave.open(wavf, 'r')

    # get file info
    ch = wr.getnchannels()
    width = wr.getsampwidth()
    fr = wr.getframerate()
    fn = wr.getnframes()
    total_time = 1.0 * fn / fr
    integer = math.floor(total_time)
    t = int(time)
    frames = int(ch * fr * t)
    num_cut = int(integer//t)

    # print it
    print("total time(s) : ", total_time)
    print("total time(integer) : ", integer)
    print("time : ", t)
    print("number of cut : ", num_cut)

    # convert to numpy array
    data = wr.readframes(wr.getnframes())
    wr.close()
    X = np.frombuffer(data, dtype=int16)

    print()

    for i in range(num_cut):
        print(str(i) + ".wav --> OK!")
        output the cuts
        outf = 'output/' + str(i) + '.wav' 
        start_cut = i*frames
        end_cut = i*frames + frames
        Y = X[start_cut:end_cut]
        outd = struct.pack("h" * len(Y), *Y)

        # write the file
        ww = wave.open(outf, 'w')
        ww.setnchannels(ch)
        ww.setsampwidth(width)
        ww.setframerate(fr)
        ww.writeframes(outd)
        ww.close()
    return num_cut

#----------------------------------------
# Find the overall tempo
#---------------------------------------
def totaltempo(filename):
    # create name  recorded file
    name = filename

    # Import wav files
    y, sr = librosa.load(name)

    # Tempo and beat extraction
    tempo , beat_frames = librosa.beat.beat_track(y=y, sr=sr)

    # print results
    print()
    print("total tempo : ", int(tempo))
    print()
    return int(tempo)

#----------------------------------------
# Find split tempo
#---------------------------------------

def temposearch(num, time):
    #Declare a variable for return
    l = []
    t = []
    t_time = 0
    before_tempo = 0

    print("division tempo")

    # Loading music
    for i in range(0,num,1):
        # Create file name to search => output/i.wav
        name = "output/" + str(i) + ".wav"

        # Import wav files
        y, sr = librosa.load(name)

        # Tempo and beat extraction
        tempo , beat_frames = librosa.beat.beat_track(y=y, sr=sr)
        int_tempo = int(tempo)

        # show tempo
        print(str(i+1) +  ":" + str(int_tempo))

        #return results as array
        l.append(int_tempo)
        t_time = t_time + int(time)
        t.append(t_time)

    return l, t

#----------------------------------------------
# Run tempo reader on full file and split files
#---------------------------------------------
def run_tempo_reader(f_name, cut_time):
    # Check if there is already a directory with the same name as we use it for cut files to be produced
    file = os.path.exists("output")
    print(file)

    if file == False:
        # if not make it
        os.mkdir("output")
    else:
        # remove any old wav files before we create new ones
        #
        dir_1=os.listdir("output")
        for fil in dir_1:
            if not fil.find(".wav") == -1:
                os.remove(fil)
                
    # cut the wav into pieces
    n = int(cut_wav(f_name, cut_time))

    # get the total tempo
    tempo_whole_file = totaltempo(f_name)
    tempo, time = temposearch(n, cut_time)

    # For the title
    name = "Tempo analysis" + cut_time + "split in seconds"

    # find an average for the cuts data
    tot_tempo = 0
    n_cuts = 0
    for z in tempo:
        tot_tempo = tot_tempo + z
        n_cuts += 1
    avg_tempo = tot_tempo / n_cuts
    
    # plot the cuts data
    # comment out this if you dont want to plot the results
    plt.title(name, fontname="MS Gothic")
    plt.xlabel("time(s)", fontname="MS Gothic")
    plt.ylabel("tempo (bpm)", fontname="MS Gothic")
    plt.ylim(60, 180)
    plt.plot(time, tempo)
    plt.show()
    
    return tempo_whole_file, avg_tempo

#----------------------------------------------
# average the bpm using the 3 methods
#---------------------------------------------
def calc_light_freq(output_file):
    # use method 1
    bpm1 = get_BPM()
    cut_time = 2                                                                # cut the recording into 2 second samples
    # use method 2
    bpm2, bpm3 = run_tempo_reader(output_file, cut_time) 
    # check they are all in range and valid then find the mean
    bpm_list1 = [ bpm1, bpm2, bpm3 ]
    bpm_list2 = []
    for el in bpm_list:
        if el >= bpm_min and el <= bpm_max :
            bpm_list2.append(el)  
    tot = 0
    num_el = 0
    for el2 in bpm_list2 :
        tot = tot + el2
        num_el += 1  
    if not (num_el == 0):
        ret_v = 60.0 / float(tot / num_el)   
    else:
        print("using default of 0.5[s] as none of the values sampled were good")
        ret_v = 0.5
    return ret_v        

#--------------------------------------------------------------------------------
# set the light groups to the colors in the chosen sequence and time frequency
# which was calculated in tempo with the bpm of the music
# rd, gd, bd,  are default rgb in mode 4  ga1 ga2 gas1 gas2  are the addresses
#-------------------------------------------------------------------------------
def set_lights(rd, gd, bd, ga1 = None, ga2 = None, gas1 = None, gas2 = None):

    # set rgb ligting
    rgbw = RemoteValueColorRGBW(
        xknx,
        group_address=ga1,
        group_address_state=gas1,
        device_name="RGBWLight",
    )
    rgbw1 = RemoteValueColorRGBW(
        xknx,
        group_address=ga2,
        group_address_state=gas2,
        device_name="RGBWLight2",
    )
    #  if the next button press can not be activated during this then consider making a thread
    #  import threading
    #  thread = threading.Thread(target=code_below, args=(set_active, set_pattern, tim))
    #  thread.start()
    while True:
        # use a bias to extend the delay between transitions if you want to
        tim = float(PERIOD_BIAS) * LIGHT_DLY
        
        if SET_PATTERN == 0:                                     # each pattern can be as you like
            rgbw.set([255, 0, 0, 0])  # red
            time.sleep(tim)
            rgbw.set([0, 255, 0, 0])  # green
            time.sleep(tim)
            rgbw.set([0, 0, 255, 0])  # blue
            time.sleep(tim)
            rgbw.set([0, 0, 0, 0, 15])  # off
            time.sleep(tim)
            rgbw1.set([255, 0, 0, 0])  # red
            time.sleep(tim)
            rgbw1.set([0, 255, 0, 0])  # green
            time.sleep(tim)
            rgbw1.set([0, 0, 255, 0])  # blue
            time.sleep(tim)
            rgbw1.set([0, 0, 0, 0, 15])  # off
            time.sleep(tim)
        elif SET_PATTERN == 1:
            rgbw.set([255, 100, 0, 0])  
            time.sleep(tim)
            rgbw.set([0, 255, 100, 0])  
            time.sleep(tim)
            rgbw.set([200, 50, 255, 0])  
            time.sleep(tim)
            rgbw.set([0, 0, 0, 0, 15])  # off
            time.sleep(tim)
            rgbw1.set([0, 100, 255, 0])  
            time.sleep(tim)
            rgbw1.set([100, 255, 0, 0])  
            time.sleep(tim)
            rgbw1.set([255, 0, 70, 0])  
            time.sleep(tim)
            rgbw1.set([0, 0, 0, 0, 15])  # off
            time.sleep(tim)
        elif SET_PATTERN == 2:
            r = 0
            g = 255
            b = 100
            for zz in range(0, 255):
                rgbw.set([r, g, b, 0]) 
                r = (r + 10) % 256
                g = (g + 10) % 256                
                b = (b + 10) % 256 
                rgbw1.set([g, b, r, 0])
                time.sleep(tim) 
        elif SET_PATTERN == 3:
            rgbw.set([100, 100, 100, 0])  
            time.sleep(tim)
            rgbw1.set([0, 0, 0, 0, 15])  # off
            time.sleep(tim)
        elif SET_PATTERN == 4:
            rgbw.set([rd, gd, bd, 0])    # set to default colot specified by the function
            time.sleep(tim)
            rgbw1.set([0, 0, 0, 0, 15])  # off
            time.sleep(tim)

# using USB-RS485 Converter
#
import serial
import time
import numpy as np

class PyDMX:

    DMX512_STX_LIGHT = 0x00                           # start transmission for lighting  Default Null Start Code for Dimmers per DMX512 & DMX512/1990
    DMX512_STX_MSB_DPT = 0x01                         # Most significant Byte of double precision transmission or Soundlight
    DMX512_STX_FP_16BITW = 0x021                      # Following packet is 256 16-Bi words in Lo byte/Hi byte order
    DMX512_STX_R_A_Gray = 0x03                        # manufacturer specific RA gray or white rabbit co
    DMX512_STX_Checksum = 0x04                        # T_Recursive
    DMX512_STX_Answerback = 0x05                      # T_Recursive
    DMX512_STX_16bitLow = 0x06                        # T_Recursive
    DMX512_STX_COMP_DATA = 0x07                       # T_Recursive
    DMX512_STX_COMP_16BIT = 0x08                      # T_Recursive
    DMX512_STX_EntTech = 0x09                         # Entertainment Technology
    DMX512_STX_MODE_LIGHT = 0x0A                      # Mode Lighting Second universe of 512 channels on one data link
    DMX512_STX_GODDARD = 0x0B                         # Goddard Design
    DMX512_STX_SGM = 0x0C                             # SGM
    DMX512_STX_ENG_ART = 0x0D                         # Engineering Arts
    DMX512_STX_CETronics = 0x0E                       # CE Tronics
    DMX512_STX_MORPH = 0x0F                           # Morpheus Lights
    DMX512_STX_MPAS1 = 0xBB                           # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS2 =0xCB                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS3 =0xDE                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS4 =0xDF                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS5 =0xE0                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS6 =0xED                            # Download Dimmer Information Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_AVO_DCS =0xFF                          # AVO Dimmer Curve Select
    DMX512_STX_DCD =0xEH                              # e:cue control GmbH Device configuration data
    DMX512_STX_NSI_DCD =0xE1                          # NSI colortran Dim/Non-Dim Control
    DMX512_STX_NSI_ENR =0xE0                          # NSI colortran ENR Mode Control
    DMX512_STX_EDC_PRIO =0xDD                         # Alternate start code DD is for use in transmitting per channel priority for use in merging streams in multi-source DMX applications. Priorities will range from 0 at the low end, which means do not use the data in the corresponding slot, to 200, which means use this data over any slot data supplied with a priority between 0 and 199. Values above 200 are reserved for future use.
    DMX512_STX_E1_SIP =0xD0                           # E1 system info packet
    DMX512_STX_E1_RDM =0xCC                           # E1 E1.20 (RDM) start code
    DMX512_STX_SUN_SPECIAL =0xAA                      # Sun
    DMX512_STX_PLASA_SPECIAL =0x92                    # BSR E1.45 Alternate START Code
    DMX512_STX_E1_MAN_ID =0x91                        # 2-byte Manufacturer ID serves as an identifier that the data following in that packet is proprietary to that entity and should be ignored by all others
    DMX512_STX_PLASA_UTF8 =0x90                       # utf8 packet
    DMX512_STX_MP_SPECIAL =0x8B                       # Martin Professional A/S
    DMX512_STX_CLAY_PAKY_SPECIAL =0x8A                # CLAY PAKY S.p.A
    DMX512_STX_ANYTRON_SYNC =0x83                     # To synchronise both the memory contents and the internal clocks of lighting control equipment. Min packet length 24 bytes. Max 512.
    DMX512_STX_WYBRON_SPECIAL =0x57                   # Wybron, Inc.
    DMX512_STX_E1_TEST =0x55                          # E1 Test
    DMX512_STX_LP_SPECIAL =0x50                       # LightProcessor Ltd
    DMX512_STX_OSCAR_BACKUP =0x4F                     # Oscar Lighting AB Backup States
    DMX512_STX_AVO_SPECIAL =0x4D                      # Avolites Ltd.Proprietary function with ART2000 products
    DMX512_STX_LUX_STX =0x4C                          # 4Ch is the START Code used for OpenDMX messages, a protocol developed for use on LUX Italia products, and published by them for royalty-free use by anyone. Visit the LUX Italia website (http://www.luxitalia.eu) for a copy of the specification.
    DMX512_STX_ENFIS_SPECIAL =0x48                    # ASC is used for passing proprietary data for applications such as factory test, configuration, and software update.
    DMX512_STX_GVA_CONFIG =0x47                       # Manufacturer-specific configuration data and remote control
    DMX512_STX_COEMAR_SPECIAL =0x44                   # Coemar Spa
    DMX512_STX_CITY_SPECIAL =0x43                     # Purpose: firmware updates and product configuration
    DMX512_STX_LSC_SPECIAL =0x42                      # Proprietary remote peripheral control
    DMX512_STX_MICROLITE_SPECIAL =0x41                # mircolite
    DMX512_STX_SAND_SPECIAL =0x3F                     # SAND
    DMX512_STX_AVAB_SPECIAL =0x3E                     # AVAB spsecific info
    DMX512_STX_AVAB_SMART =0x3D                       # AVAB Smart 16 Bit Format
    DMX512_STX_AVAB_INTERNAL =0x3C                    # AVAB Internal Functions
    DMX512_STX_TIR_SPECIAL =0x35                      # Programmable DMX512-based LED controllers. Alternate START code used to specify various operating parameters for DMX512 network and standalone operation. Min.frames: 15; Max frames: 40
    DMX512_STX_TESI_SPECIAL =0x33                     # The start code's purpose is to send/receive application-specific information and execute product software update
    DMX512_STX_PR_FIRMWARE =0x30                      # The Start Code purpose for now is to be able to perform firmware updates to our products. In the future we might add more functions to it.
    DMX512_STX_JOHN_SPECIAL =0x2A                     # johnson
    DMX512_STX_HES_SPECIAL =0x26                      # high end systems
    DMX512_STX_GDS_SPECIAL =0x22                      # gds
    DMX512_STX_ELETRO_SPECIAL =0x21                   # eletro lab
    DMX512_STX_RJULIAT_SPECIAL =0x1E                  # Robert Juliat Used to update old products which do not have RDM capabilities but still supported. Also, to modify remotely some factory settings.
    DMX512_STX_DROSS_SPECIAL =0x1D                    # Dangeross Design
    DMX512_STX_KLH_RTC =0x1C                          # The alternate byte is to provide real-time updates for triple precision data ( either 20 or 24 bit ) for use with Photon Cannon project. This will allow the use of the standard to control 100 lamp fixtures at greater than 75 Hz update rate
    DMX512_STX_ESCAPE_SPECIAL =0x1B                   # The purpose is to toggle inner program of the receiver. If the Start code is 0, the machine answers with Program 1. If the Start code is 27, the machine answers with Program 2. Rest of the DMX trame would remain exactly the same as in USITT description. The purpose for us is to control different types of machines with the same DMX values (Program 2) using a switch box sending Start Code 27 and predetermined DMX values. The program run by Start Code 0 would be adapted to a fader control.
    DMX512_STX_IT_SPECIAL =0x1A                       # integrated theatre
    DMX512_STX_HUB_SPECIAL =0x19                      # hubbell
    DMX512_STX_AND_SPECIAL =0x18                      # andera
    DMX512_STX_E1_TEXT =0x17                          # E1 ANSI E1.11 Text Packet
    DMX512_STX_AL_TEXT =0x17                          # Artistic License Text Packet (matches use in ANSI E1.11)
    DMX512_STX_PER_SPECIAL =0x16                      # Peradise We build specialFx and moving set parts. The startcode will be used to identify data that is used for tacticle feedback from devices (Position, status, errors, etc)
    DMX512_STX_CDCA_CONFIG =0x15                      # Firmware update and configuration info
    DMX512_STX_SSI_SPECIAL =0x14                      # We are implementing a message-based protocol that is optimized for safe and secure motor control, embedded with lighting control. A unique Start Code is the ideal way to identify this alternate data type on the DMX network, while keeping packets as short as possible.
    DMX512_STX_ZERO88_SPECIAL =0x13                   # zero 88
    DMX512_STX_BJA_FIRMWARE =0x12                     # For updating the firmware in my equipment and to control it (Reset). My packet size will between the 3 and 256 bytes.
    DMX512_STX_TBS_SPECIAL =0x11                      # Tokyo Broadcast systems
    DMX512_STX_ADB_SPECIAL =0x10                      # ADB
    DMX512_STX_ELDO_FIRM =0xDO                        # eldo LED Configuration, firmware updates and standalone configuration. Framelength to vary from 12 to 512 bytes.

    def __init__(self,COM='COM8',Cnumber=512,Brate=250000,Bsize=8,StopB=2,use_prev_data=True,preserve_data_name="preserved_data.txt"):
        #start serial
        self.channel_num = Cnumber
        self.ser = serial.Serial(COM,baudrate=Brate,bytesize=Bsize,stopbits=StopB)
        self.data = np.zeros([self.channel_num+1],dtype='uint8')
        self.data[0] = 0                                                                                       # StartCode
        self.sleepms = 50.0
        self.breakus = 176.0
        self.MABus = 16.0
        # save filename
        self.preserve_data_name = preserve_data_name
        self.use_prev_data = use_prev_data
        # load preserved DMX data
        if use_prev_data:
            try:
                self.load_data()
            except:
                print("Its first time run or something is wrong. please check data format!")

    def set_stx_to_manufacturer(manu_code=self.DMX512_STX_JOHN_SPECIAL): # set the stx code to a manufacturer default johnson
        self.data[0] = manu_code
        
    def set_random_data(self):
        self.data[1:self.channel_num+1]= np.random.rand(self.channel_num)*255

    def set_data(self,id,data):
        self.data[id]=data

    def set_datalist(self,list_id,list_data):
        try:
            for id,data in zip(list_id,list_data):
                self.set_data(id,data)
        except:
            print('list of id and data must be the same size!')

    def send(self):
        # Send Break : 88us - 1s
        self.ser.break_condition = True
        time.sleep(self.breakus/1000000.0)
        
        # Send MAB : 8us - 1s
        self.ser.break_condition = False
        time.sleep(self.MABus/1000000.0)
        
        # Send Data
        self.ser.write(bytearray(self.data))
        
        # Sleep
        time.sleep(self.sleepms/1000.0) # between 0 - 1 sec

    def sendzero(self):
        self.data = np.zeros([self.channel_num+1],dtype='uint8')
        self.send()

    def load_data(self):
        self.data = np.loadtxt(self.preserve_data_name,dtype='int')        

    def preserve_data(self):
        np.savetxt(self.preserve_data_name,self.data)        

    def __del__(self):
        print('Close serial server!')
        # close with preserving current DMX data, I guess you may not need to reset DMX signal in this option.
        if self.use_prev_data:
            self.preserve_data()
        else:
            self.sendzero()
        self.ser.close()

    def __exit__(self):
        print('Close serial server!')
        # close with preserving current DMX data, I guess you may not need to reset DMX signal in this option.
        if self.use_prev_data:
            self.preserve_data()
        else:
            self.sendzero()
        self.ser.close()

# this thread runs the dmx light
#        
def do_dmx(tim=0, kw_args = None):

    dmx = PyDMX('/dev/ttyUSB0')                       # the port you have connected the rs485 converter
    
    while True:
        tim = float(PERIOD_BIAS) * LIGHT_DLY 
        if SET_PATTERN == 0:    
            dmx.set_data(3, 51)                           # to pink
            dmx.send()
            time.sleep(tim)
            dmx.set_data(3, 16)                           # to orange
            dmx.send()
            time.sleep(tim)
            dmx.set_data(4, 129)                          # gobo scroll fast/slo
            dmx.send()
            time.sleep(tim)
            dmx.set_data(5, 241)                          # random strobe
            dmx.send()        
            time.sleep(tim)
        elif SET_PATTERN == 1: 
            dmx.set_data(8, 251)                           # sound active mode
            dmx.send()
            time.sleep(tim*2)
            dmx.set_data(3, 16)                           # to orange
            dmx.send()
            time.sleep(tim)
            dmx.set_data(3, 44)                           # to blue
            dmx.send()
            time.sleep(tim)           
        elif SET_PATTERN == 2: 
            dmx.set_data(3, 25)                           # to yellow
            dmx.send()
            time.sleep(tim)
            dmx.set_data(3, 30)                           # to green
            dmx.send()
            time.sleep(tim)
            dmx.set_data(4, 67)                           # gobo open shake
            dmx.send()
            time.sleep(tim)
            dmx.set_data(5, 140)                          # shutter fast open slow close
            dmx.send()  
            time.sleep(tim)            
        elif SET_PATTERN == 3:  
            dmx.set_data(3, 44)                           # to blue
            dmx.send()
            time.sleep(tim)
            dmx.set_data(3, 16)                           # to orange
            dmx.send()
            time.sleep(tim)        
            dmx.set_data(3, 30)                           # to green
            dmx.send()
            time.sleep(tim)
            dmx.set_data(3, 25)                           # to yellow
            dmx.send()
            time.sleep(tim)
         
#--------------------------------------------------------------------------------
# record the sound using soundcard library the analyse and set lights
#-------------------------------------------------------------------------------            
def record_using_soundcard( op_file_name, record_sec = 10,  channels=1, samplerate = 48000  ) :

    #SET_ACTIVE = 0
    with sc.get_microphone(id=str(sc.default_speaker().name), include_loopback=True).recorder(samplerate=samplerate) as mic:
        # Recording from default microphone
        data = mic.record(numframes=samplerate*record_sec)
    
        # write the output file 
        if channels == 1:
            sf.write(file=op_file_name, data=data[:, 0], samplerate=samplerate)
        else:
            sf.write(file=op_file_name, data=data, samplerate=samplerate)	

        #SET_ACTIVE = 1
        light_duration = calc_light_freq(op_file_name)
        #set_lights(light_duration) 
        return light_duration       
        
#-------------------------------------------------------------------------------------------
# record the sound using pyaudio library then analyse and calculate bpm hence light duration
#-------------------------------------------------------------------------------------------       
def record_using_pyaudio( fn, recordSeconds=10, channels=2, rate=44100 ):

    #SET_ACTIVE = 0
    chuck = 1024
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chuck)
    frames = []

    for i in range(0, int(rate / chuck * recordSeconds)):
        data = stream.read(chuck)
        frames.append(data)
            
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(fn, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()
    
    light_duration = calc_light_freq(fn)
    #SET_ACTIVE = 1 
    
    return light_duration
    #set_lights(light_duration)  

# this wraps the threading library so you can stop the thread by raising an exception
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')
            
#--------------------------------------------------------------------------------
# handle the button presses and perform the dedicated actions
#------------------------------------------------------------------------------- 
@buttonshim.on_press(BUTTONS)
def button_p_handler(button, pressed):
    # button A pressed use pyadio to stereo
    if (button == 0):
        buttonshim.set_pixel(0x00, 0x00, 0xff)                           # LED to blue  
        LIGHT_DLY = record_using_pyaudio(output_file_name, 20, 2, 44100)
        with open(LIGHT_DLY_FL, "w") as f:                               # save the setting to disk for recall after reboot
            f.write("\n".join(str(LIGHT_DLY)))   
        try:
            if x.is_alive(): x.raise_exception()
        except:
            print("starting the light task with delay = ",LIGHT_DLY)  
        try:
            if y.is_alive(): y.raise_exception()
        except:
            print("starting the dmx task with delay = ",LIGHT_DLY)            
        # start the thread for light control
        # default color 0,127,127 and addresses passed via json msg
        #
        x = twe(name = 'Thread A', target=set_lights, args=(0, 127, 127), kwargs={'ga1': '1/1/40', 'ga2': '1/1/43', 'gas1': '1/1/41', 'gas2' : '1/1/44'})
        x.start()
        # now start the dmx thread
        y = twe(name = 'Thread B', target=do_dmx, args=(0), kwargs={'kw_args': 'dummy_for_now'})
        y.start()
        buttonshim.set_pixel(0x00, 0xff, 0x00)                            # LED to green  
    # button B pressed use soundcard to record to mono
    elif (button == 1):
        buttonshim.set_pixel(0x00, 0x00, 0xff)                            # LED to blue  
        LIGHT_DLY = record_using_soundcard(output_file_name, 20, 1, 48000)
        with open(LIGHT_DLY_FL, "w") as f:                                # save the setting to disk for recall after reboot
            f.write("\n".join(str(LIGHT_DLY)))                                                      
        try:
            if x.is_alive(): x.raise_exception()
        except:
            print("starting the light task first time with delay = ",LIGHT_DLY)
        try:
            if y.is_alive(): y.raise_exception()
        except:
            print("starting the dmx task with delay = ",LIGHT_DLY) 
        x = twe(name = 'Thread A', target=set_lights, args=(0, 127, 127), kwargs={'ga1': '1/1/40', 'ga2': '1/1/43', 'gas1': '1/1/41', 'gas2' : '1/1/44'})
        x.start()
        # now start the dmx thread
        y = twe(name = 'Thread B', target=do_dmx, args=(0), kwargs={'kw_args': 'dummy_for_now'})
        y.start()
        buttonshim.set_pixel(0x00, 0xff, 0x00)                            # LED to green  
    # button C pressed change the pattern
    elif (button == 2):
        SET_PATTERN = (SET_PATTERN + 1) % NUM_PATTERNS	
    # button D pressed prolong the delay up to max
    elif (button == 3):
        PERIOD_BIAS = ((PERIOD_BIAS + 1) % MAX_PERIOD_BIAS) + 1  
    # button E pressed on/off the sequencer
    elif (button == 4):
        try:
            if y.is_alive(): 
                y.raise_exception()                                         # kill thread and stop lights
            else:
                y = twe(name = 'Thread B', target=do_dmx, args=(0), kwargs={'kw_args': 'dummy_for_now'})
                y.start()                        
        except:
            print("starting the dmx task first time with delay = ",LIGHT_DLY)                                             
            y = twe(name = 'Thread B', target=do_dmx, args=(0), kwargs={'kw_args': 'dummy_for_now'})
            y.start()   
        try:
            if x.is_alive(): 
                x.raise_exception()                                         # kill thread and stop lights
                buttonshim.set_pixel(0xff, 0x00, 0x00)                      # LED to red
            else:
                x = twe(name = 'Thread A', target=set_lights, args=(0, 127, 127), kwargs={'ga1': '1/1/40', 'ga2': '1/1/43', 'gas1': '1/1/41', 'gas2' : '1/1/44'})
                x.start()     
                buttonshim.set_pixel(0x00, 0xff, 0x00)                      # LED to green                   
        except:
            print("starting the light task first time with delay = ",LIGHT_DLY)                                             
            x = twe(name = 'Thread A', target=set_lights, args=(0, 127, 127), kwargs={'ga1': '1/1/40', 'ga2': '1/1/43', 'gas1': '1/1/41', 'gas2' : '1/1/44'})
            x.start()  
            buttonshim.set_pixel(0x00, 0xff, 0x00)                          # LED to green               

@buttonshim.on_hold(buttonshim.BUTTON_D, hold_time=2)
def hold_handler(button):
    # button D pressed for 2 seconds or more than reset the bias on the lights delay back to fastest
    PERIOD_BIAS = 1  
    
signal.pause()                                                          # Stop script from immediately exiting


    
