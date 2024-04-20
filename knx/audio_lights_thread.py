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
        # start the thread for light control
        # default color 0,127,127 and addresses passed via json msg
        #
        x = twe(name = 'Thread A', target=set_lights, args=(0, 127, 127), kwargs={'ga1': '1/1/40', 'ga2': '1/1/43', 'gas1': '1/1/41', 'gas2' : '1/1/44'})
        x.start()
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
        x = twe(name = 'Thread A', target=set_lights, args=(0, 127, 127), kwargs={'ga1': '1/1/40', 'ga2': '1/1/43', 'gas1': '1/1/41', 'gas2' : '1/1/44'})
        x.start()
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


    
