#!/usr/bin/env python
# a simple song maker programmed in python to play a buzzer(synth), midi interface and play a wav sample all together
# a more complex file which converts midi to wav and applies effects and mixing will be posted later here...
#
#
import RPi.GPIO as GPIO
import time
import math

# ======================== synth using buzzer connected to GPIO PIN of pi ===============================
#
# connect synth buzzer to defined below
SYN_CONNECT=18

# set the track tempo in bpm
SONG_TEMPO=132

# global for loop running sound
RUN_IT=1

# note to [midinote , frequency] table
# International style	Yamaha-style	Sound name	Note number	Frequency	
#-1 -2
# the tagname is note_[International style number]_[Yamaha-style number]	
C_m1_m2 = [0,8.2]
Cs_m1_m2 = [1,8.7]
D_m1_m2	= [2,9.2]
Ds_m1_m2 = [3,9.7]
E_m1_m2 = [4,10.3]
F_m1_m2 = [5,10.9]
Fs_m1_m2 = [6,11.6]
G_m1_m2 = [7,12.2]
Gs_m1_m2 = [8,13.0]
A_m1_m2 = [9,13.8]
As_m1_m2 = [10,14.6]
B_m1_m2 = [11,15.4]
# 0 -1 	
C_0_m1 = [12,16.4]
Cs_0_m1 = [13,17.3]
D_0_m1 = [14,18.4]
Ds_0_m1 = [15,19.4]
E_0_m1 = [16,20.6]
F_0_m1 = [17,21.8]
Fs_0_m1 = [18,23.1]
G_0_m1 = [19,24.5]
Gs_0_m1 = [20,26.0]
A_0_m1 = [21,27.5]
As_0_m1 = [22,29.1]
B_0_m1 = [23,30.9]
# 1	0	
C_1_0 =	[24,32.7]
Cs_1_0 = [25,34.6]
D_1_0 =	[26,36.7]
Ds_1_0 = [27,38.9]
E_1_0 =	[28,41.2]
F_1_0 =	[29,43.7]
Fs_1_0 = [30,46.2]
G_1_0 =	[31,49.0]
Gs_1_0 = [32,51.9]
A_1_0 =	[33,55.0]
As_1_0 = [34,58.3]
B_1_0 =	[35,61.7]
# 2	1	
C_2_1 =	[36,65.4]
Cs_2_1 = [37,69.3]
D_2_1 =	[38,73.4]
Ds_2_1 = [39,77.8]
E_2_1 =	[40,82.4]
F_2_1 =	[41,87.3]
Fs_2_1 = [42,92.5]
G_2_1 =	[43,98.0]
Gs_2_1 = [44,103.8]
A_2_1 =	[45,110.0]
As_2_1 = [46,116.5]
B_2_1 = [47,123.5]
#3	2	
C_3_2 =	[48,130.8]
Cs_3_2=	[49,138.6]
D_3_2=	[50,146.8]
Ds_3_2=	[51,155.6]
E_3_2=	[52,164.8]
F_3_2=	[53,174.6]
Fs_3_2=	[54,185.0]
G_3_2=	[55,196.0]
Gs_3_2=	[56,207.7]
A_3_2=	[57,220.0]
As_3_2=	[58,233.1]
B_3_2=	[59,246.9]
#4	3	
C_4_3=	[60,261.6]
Cs_4_3=	[61,277.2]
D_4_3=	[62,293.7]
Ds_4_3=	[63,311.1]
E_4_3=	[64,329.6]
F_4_3=	[65,349.2]
Fs_4_3=	[66,370.0]
G_4_3=	[67,392.0]
Gs_4_3=	[68,415.3]
A_4_3=	[69,440.0]
As_4_3=	[70,466.2]
B_4_3=	[71,493.9]
#5	4	
C_5_4=	[72,523.3]
Cs_5_4=	[73,554.4]
D_5_4=	[74,587.3]
Ds_5_4=	[75,622.3]
E_5_4=	[76,659.3]
F_5_4=	[77,698.5]
Fs_5_4=	[78,740.0]
G_5_4=	[79,784.0]
Gs_5_4=	[80,830.6]
A_5_4=	[81,880.0]
As_5_4=	[82,932.3]
B_5_4=	[83,987.8]
#6	5	
C_6_5=	[84,1046.5]
Cs_6_5=	[85,1108.7]
D_6_5=	[86,1174.7]
Ds_6_5=	[87,1244.5]
E_6_5=	[88,1318.5]
F_6_5=	[89,1396.9]
Fs_6_5=	[90,1480.0]
G_6_5=	[91,1568.0]
Gs_6_5=	[92,1661.2]
A_6_5=	[93,1760.0]
As_6_5=	[94,1864.7]
B_6_5=	[95,1975.5]
#7	6	
C_7_6=	[96,2093.0]
Cs_7_6=	[97,2217.5]
D_7_6=	[98,2349.3]
Ds_7_6=	[99,2489.0]
E_7_6=	[100,2637.0]
F_7_6=	[101,2793.8]
Fs_7_6=	[102,2960.0]
G_7_6=	[103,3136.0]
Gs_7_6=	[104,3322.4]
A_7_6=	[105,3520.0]
As_7_6=	[106,3729.3]
B_7_6=	[107,3951.1]
# 8	7	
C_8_7=	[108,4186.0]
C_8_7=	[109,4434.9]
D_8_7=	[110,4698.6]
Ds_8_7=	[111,4978.0]
E_8_7=	[112,5274.0]
F_8_7=	[113,5587.7]
Fs_8_7=	[114,5919.9]
G_8_7=	[115,6271.9]
Gs_8_7=	[116,6644.9]
A_8_7=	[117,7040.0]
As_8_7=	[118,7458.6]
B_8_7=	[119,7902.1]
#9	8	
C_9_8=	[120,8372.0]
Cs_9_8=	[121,8869.8]
D_9_8=	[122,9397.3]
Ds_9_8=	[123,9956.1]
E_9_8=	[124,10548.1]
F_9_8=	[125,11175.3]
Fs_9_8=	[126,11839.8]
G_9_8=	[127,12543.9]

# note duration table @ 120 bpm
W = 1000
T = 750
H = 500
Q = 250
E = 125
S = 62
DURATIONS = [W, T, H, Q, E, S]

# modifier functions for the synth pwm connected to the GPIO pin.
def note(val):
    return note
def minor_second(val):
    return ((val)*1.0595)                          
def major_second(val):
    return ((val)*1.225) 
def minor_third(val):
    return ((val)*1.1892)
def major_third(val):   
    return ((val)*1.2599)
def perfect_fourth(val):
    return ((val)*1.13348)  
def tritone(val): 
    return ((val)*math.sqrt(2))
def perfect_fifth(val): 
    return ((val)*1.5)
def minor_sixth(val): 
    return ((val)*1.6)
def major_sixth(val): 
    return ((val)*1.66667)
def minor_seventh(val):
    return ((val)*1.75)
def major_seventh(val): 
    return ((val)*1.83333)
def octave(val): 
    return ((val)*2.0)

# list the above functions to choose via a index number
MODS = [note, minor_second, major_second, minor_third, major_third, perfect_fourth, tritone, perfect_fifth, minor_sixth, major_sixth, minor_seventh, major_seventh, octave]

# modify the duration settings according to a specified tempo
def make_duration_for_tempo(new_tempo, durations):
    new_durations = []
    for d in durations:
        append.new_durations(d*(120.0/new_tempo))
    return new_durations

# play tone at freq specified for the duration specified using the methos listed in MODS array    
def playTone(frequency, duration, mode=0):
    apply_mod = MODS[mode]
    D2A = GPIO.PWM(SYN_CONNECT, apply_mod(frequency))    # create object D2A for PWM on port 25 at 1KHz
    D2A.start(100)                                       # start the PWM with a 100 percent duty cycle (on)
    time.sleep(duration);
    D2A.stop()
    GPIO.cleanup()                                       # not sure if we must do it    

# as above but ramp up to the frequency  
def playRampUpTone(frequency, duration, mode=0):
    apply_mod = MODS[mode]
    D2A = GPIO.PWM(SYN_CONNECT, apply_mod(frequency))    # create object D2A for PWM on port 25 at 1KHz
    D2A.start(0)                                         # start the PWM with a 100 percent duty cycle (off)
    ramps = [ 25, 50, 75 100 ]
    for z in ramps:
        D2A.ChangeDutyCycle(z)
        time.sleep(0.1)
    time.sleep(duration-(0.4));
    D2A.stop()
    GPIO.cleanup()                                       # not sure if we must do it    

def playPulseTone(frequency, duration, mode=0):
    apply_mod = MODS[mode]
    D2A = GPIO.PWM(SYN_CONNECT, apply_mod(frequency))    # create object D2A for PWM on port 25 at 1KHz
    D2A.start(0)                                         # start the PWM with a 100 percent duty cycle (off)
    ramps = [ 100, 0, 100, 0, 100, 100, 0, 100, 0, 100 ]
    for z in ramps:
        D2A.ChangeDutyCycle(z)
        time.sleep(0.05)
    time.sleep(duration-(0.05*4));
    D2A.stop()
    GPIO.cleanup()                                       # not sure if we must do it 

# list of the functions to apply when playing the arrangement   
FUNKS = [playTone, playRampUpTone, playPulseTone]

def synth_play_tune(song, duration_list, duration_method_list):
    for i,freq_note in enumerate(song):
        execute_method = FUNKS[duration_method_list[i][1]]
        execute_method(freq_note, duration_list[duration_method_list[i][0]], duration_method_list[i][2])

# modify this for your song
# this plays the synth as specified in song and duration_method_list which ia list of indexs to [duration_method_mode]
#
FREQ=1                                           # select the freqiency element of the number in the array
def play_synth():
    # song notes specific to this arrangement
    song = [D_4_3[FREQ], E_4_3[FREQ], D_4_3[FREQ], A_3_2[FREQ]]
    # duration is H,Q,Q,H function_method = playT,playT,pulseT,rampT none,none,minor2nd,tritone ]
    duration_method_list = ["200", "300", "321", "216"]  
    if not [len(duration_method_list) == len(song)]: 
        print("song and duration_method_list must contain the same number of elements in the list")
        return -1        
    song_tempo = SONG_TEMPO
    d_list = make_duration_for_tempo(song_tempo, DURATIONS):   
    synth_play_tune(song, d_list, duration_method_list)
    return 0

def loop_synth():
    # song notes specific to this arrangement
    song = [D_4_3[FREQ], E_4_3[FREQ], D_4_3[FREQ], A_3_2[FREQ]]
    # duration is H,Q,Q,H function_method = playT,playT,pulseT,rampT none,none,minor2nd,tritone ]
    duration_method_list = ["200", "300", "321", "216"]  
    if not [len(duration_method_list) == len(song)]: 
        print("song and duration_method_list must contain the same number of elements in the list")
        return -1        
    song_tempo = SONG_TEMPO
    d_list = make_duration_for_tempo(song_tempo, DURATIONS):   
    while RUN_IT:
        synth_play_tune(song, d_list, duration_method_list)
    return 0
    
# =============================================== midi =====================================================
# use rtmidi so install it as :- pip install mido python-rtmidi
#
import mido
from mido import Message, MidiFile, MidiTrack, MetaMessage

# modify this for your song
# this writes the composed midi sequence for playing over the midi interface to external synths
#
MID=0
def make_midi_arrangement(song_tempo):
    mid = MidiFile()
    track = MidiTrack()
    mid.tracks.append(track)
    track.append(MetaMessage('set_tempo', tempo=mido.bpm2tempo(song_tempo)))
    track.append(Message('note_on', note=D_4_3[MID], velocity=127, time=0))
    track.append(Message('note_off', note=D_4_3[MID], time=480))
    track.append(Message('note_on', note=E_4_3[MID], velocity=127, time=0))
    track.append(Message('note_off', note=E_4_3[MID], time=240))
    track.append(Message('note_on', note=D_4_3[MID], velocity=127, time=0))
    track.append(Message('note_off', note=D_4_3[MID], time=240))
    track.append(Message('note_on', note=A_3_2[MID], velocity=127, time=0))
    track.append(Message('note_off', note=A_3_2[MID], time=480))
    mid.save('mid_arrangement.mid')
    
def play_midi_arrangement():
    ports = mido.get_output_names()
    with mido.open_output(ports[0]) as outport:
        for msg in mido.MidiFile('mid_arrangement.mid'):
            time.sleep(msg.time)
            if not msg.is_meta:
                print(outport, msg)
                outport.send(msg) 

def loop_midi_arrangement():
    ports = mido.get_output_names()
    with mido.open_output(ports[0]) as outport:
        while RUN_IT:
            for msg in mido.MidiFile('mid_arrangement.mid'):
                time.sleep(msg.time)
                if not msg.is_meta:
                    print(outport, msg)
                    outport.send(msg)
                
# ------------------- sample loop -------------------------
# note the sample must be in the tempo defined in this project
#
import simpleaudio
FILENM="sample.wav"
def set_file_name(fl):
    global FILENM
    FILENM=fl
    
def loop_sample():
    while RUN_IT:
        wav_obj = simpleaudio.WaveObject.from_wave_file(FILENM)
        play_obj = wav_obj.play()
        play_obj.wait_done()

def play_sample():
    wav_obj = simpleaudio.WaveObject.from_wave_file(FILENM)
    play_obj = wav_obj.play()
    play_obj.wait_done()    
    
# ---------------- start all 4 channels ----------------------
if __name__ == '__main__':

    make_midi_arrangement(SONG_TEMPO)
    set_file_name("my_own_loop.wav")
    loop_synth_task = threading.Thread(target=loop_synth, daemon=True)
    loop_midi_task = threading.Thread(target=loop_midi_arrangement, daemon=True)
    loop_wave_task = threading.Thread(target=loop_sample, daemon=True)
    try:
        loop_synth_task.start()
        loop_midi_task.start()    
        loop_wave_task.start()
    except KeyboardInterrupt:
        RUN_IT=0
        loop_synth_task.join()
        loop_midi_task.join()    
        loop_wave_task.join()    