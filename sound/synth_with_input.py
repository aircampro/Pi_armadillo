#!/usr/bin/env python
# a simple song maker programmed in python to play a buzzer(synth), midi interface and play a wav sample all together
# also has capability to convert midi to wav and add various effects as well as read an input (mic/guitar) and mix it with the wav file 
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

# ----------------- play a midi file as a wav file -----------
"""

This set of functions have been modified from this good work on github

https://github.com/morikatron/snippet/blob/master/python_and_music/sample5_play_midi.py

© Morikatron Inc. 2020
written by matsubara@morikatron.co.jp

"""
import math
import sys
import os

import numpy as np                          # install : conda install numpy
import pyaudio                              # install : conda install pyaudio
import mido                                 # install : pip install mido
import soundfile as sf                      # install : pip install soundfile

SAMPLE_RATE = 44100

# MIDI
IX_ON_MSEC = 0                                                           # int note on
IX_OFF_MSEC = 1                                                          # int note off
IX_NOTE_NUMBER = 2                                                       # int 
IX_VELOCITY = 3                                                          # float 
IX_DULATION = 4                                                          # float 
MIDI_2_WAVE_FILE="midwav.mid"

# function to take a given note and make a corresponding wav file
def notenumber2wave(notenumber: int, duration: float, volume: float) -> np.array:
    # convert the midi note number to a frequency
    freq = 440.0 * 2 ** ((notenumber - 69) / 12)
    # create a numpy array for the note at that freq
    samples = np.sin(np.arange(int(duration * SAMPLE_RATE)) * freq * np.pi * 2 / SAMPLE_RATE) * volume
    # implement fade
    fade_len = min(100, samples.size)                                                        # fade length
    slope = (np.arange(fade_len) - 1) / fade_len                                             # slope
    samples[:fade_len] = samples[:fade_len] * slope                                          # fade in
    slope = ((fade_len - 1) - np.arange(fade_len)) / fade_len                               
    samples[-fade_len:] = samples[-fade_len:] * slope                                       
    return samples


# pad and stack
def padding_and_stack(inputs: list) -> np.array:
    if len(inputs) < 1:
        return None

    if len(inputs) == 1:
        return inputs[0]

    maxlen = 0
    for x in inputs:
        if maxlen < len(x):
            maxlen = len(x)

    for i, x in enumerate(inputs):
        if len(x) < maxlen:
            inputs[i] = np.pad(x, (0, maxlen - len(x)))

    return np.vstack(inputs)


# play （np.array） as a sound stream
def play_wave_pyaudio(wave) -> None:

    p = pyaudio.PyAudio()

    stream = p.open(format=pyaudio.paFloat32, channels=1,  rate=SAMPLE_RATE, frames_per_buffer=1024,  output=True)

    stream.write(wave.astype(np.float32).tostring())

    stream.close()

    p.terminate()

# MIDI printer
def print_midi(file_path: str) -> None:
    print("----------------- PRINT MIDI -----------------")
    file = mido.MidiFile(file_path)
    print(file)
    for track in file.tracks:
        print("-----------------", track)
        for message in track:
            print(message)
    print("----------------------------------------------")


# notes[note[],note[],...] 
def notes2track(notes: list) -> np.array:
    # いnote off）
    track_msec = 0
    for note in notes:
        if track_msec < note[IX_OFF_MSEC]:
            track_msec = note[IX_OFF_MSEC]

    track_base = np.zeros(int((track_msec / 1000.0) * SAMPLE_RATE))

    for note in notes:
        if note[IX_DULATION] > 0:  
            wave = notenumber2wave(note[IX_NOTE_NUMBER], note[IX_DULATION], note[IX_VELOCITY])
            fromix = int((note[IX_ON_MSEC] / 1000.0) * SAMPLE_RATE)
            toix = fromix + wave.size
            stacked = np.vstack((wave, track_base[fromix:toix]))
            newwave = stacked.mean(axis=0)
            track_base[fromix:toix] = newwave
    return track_base


# write the midi file to a wav file output and return a numpy array equal to it                                      
def midi2wave(file_path: str, proj_tempo: float, samplerate=SAMPLE_RATE) -> np.array:
    tempo = 500000.0 * (120.0 / proj_tempo)
    file = mido.MidiFile(file_path)
    ticks_per_beat = file.ticks_per_beat
    abs_time_tick_msec = tempo / ticks_per_beat / 1000.0

    tracks = []
    # Search tracks
    for track in file.tracks:
        now = 0                                                          # （msec）
        notes = []                                                    
        for event in track:
            now = now + event.time * abs_time_tick_msec
            if event.type == 'set_tempo':
                tempo = event.tempo
                abs_time_tick_msec = tempo / ticks_per_beat / 1000.0
                print("BPM = ", 60000000.0 / tempo)
            elif event.type == 'note_on' and event.channel == 9:
                pass
            elif event.type == 'note_off' or (event.type == 'note_on' and event.velocity == 0):
                for note in notes:
                    if (note[IX_OFF_MSEC] == 0) and (note[IX_NOTE_NUMBER] == event.note):
                        note[IX_OFF_MSEC] = now
                        note[IX_DULATION] = (note[IX_OFF_MSEC] - note[IX_ON_MSEC]) / 1000.0
                        note[IX_VELOCITY] = note[IX_VELOCITY] / 127.0
            elif event.type == 'note_on':
                notes.append([math.floor(now), 0, event.note, event.velocity, 0])
        if len(notes) > 0:  
            print("midi2wave: converting track #", len(tracks), "...")
            tracks.append(notes2track(notes))  

    print("midi2wave: mixising", len(tracks), "tracks ...")
    mixed_wave = padding_and_stack(tracks).mean(axis=0)
    print("midi2wave: finish!")
    # wrute out the data to a wav file which we can either use else where or play as a loop
    sf.write("out_midi.wav", mixed_wave, samplerate, subtype="PCM_24")
    return mixed_wave

# sets the midi file to what is passed as function argument not the default name
def set_midi_wav_file(file_path: str) -> None:
    global MIDI_2_WAVE_FILE
    MIDI_2_WAVE_FILE = file_path
    
# plays the midi as audio once using a sound stream
def play_midi_wav_stream() -> None:
    print_midi(MIDI_2_WAVE_FILE)                                         # MIDI
    wave = midi2wave(MIDI_2_WAVE_FILE,SONG_TEMPO)                        # MIDI
    play_wave_pyaudio(wave)                                              

# opens the output wav made from the midi and continuosly plays it
def loop_midi_wav() -> None:
    print_midi(MIDI_2_WAVE_FILE)                                         # MIDI
    wave = midi2wave(MIDI_2_WAVE_FILE,SONG_TEMPO)                        # MIDI
    while True:
        wav_obj = simpleaudio.WaveObject.from_wave_file("out_midi.wav")
        play_obj = wav_obj.play()
        play_obj.wait_done()

def write_wav_from_freq(freq=600, samplerate=48000)  -> np.array :
    n = np.arange(samplerate*2)                                          
    data = np.zeros((samplerate*2,2), dtype=float)  
    data[:,0] = np.sin(2.0*np.pi*freq*n/samplerate)                      
    data[:,1] = np.sin(np.random.rand(samplerate*2)*2-1.0)               
    sf.write("freq_out.wav", data, samplerate, subtype="PCM_24")         
    return data

def midi_note_from_freq(freq=500.0) -> int:
    note_float = 12 * math.log2(freq / 440) + 69
    note = round(note_float)
    return note
    
# ---------------- join two wav files ----------------------------
from pydub import AudioSegment
from pydub.effects import normalize, compress_dynamic_range, low_pass_filter, high_pass_filter, fade_in, fade_out, reverse

# apply a series of effects on an open audiostream
# usage :
# drum_track = AudioSegment.from_wav("drums.wav")
# effected_audio_segment = apply_effect(drum_track, 1)
#
def apply_effect(audio: AudioSegment, option: int, freq=1000) -> AudioSegment:
    """applies an effect to the audio"""
    effects = [
        ("Fade In", lambda a: a.fade_in(min(1000, len(a)))),
        ("Fade Out", lambda a: a.fade_out(min(1000, len(a)))),
        ("High Pass Filter", lambda a: a.high_pass_filter(freq)),
        ("Low Pass Filter", lambda a: a.low_pass_filter(freq)),
        ("Reverse", lambda a: a.reverse())
    ]
    if option <= 4:
        effect_name = effects[option][0]
        effect_func = effects[option][1]
    else:          
        effect_name, effect_func = random.choice(effects)
    print(f"applied effect chosen as : {effect_name}")
    return effect_func(audio) 
    
# join 2 wav files with audio adjustment
def join_two_wavs(file1: str, file2: str, vdb1=0, vdb2=0, ef1=-1, ef2=-1, f1=1000, f2=500) -> AudioSegment:
    s1 = AudioSegment.from_file(file1)
    s2 = AudioSegment.from_file(file2)
    # s2 -= 20   reduce volume on stream2 by 20dB 
    if not (vdb1 == 0):
        s1 -= vdb1
    if not (vdb2 == 0):
        s2 -= vdb2 
    if not (ef1 == -1):
        apply_effect(s1, ef1, f1)
    if not (ef2 == -1):
        apply_effect(s2, ef2, f2) 
    output = s1.overlay(s2, position=0)
    output.export('all_mixed.wav', format='wav')
    return output

# master the audio segment
# usage :
# drum_track = AudioSegment.from_wav("drums.wav")
# mastered_audio_segment = master_wav(drum_track, 1)
def master_wav(audio: AudioSegment, option: int) -> AudioSegment:
    if option == 0:
        mas_track = compress_dynamic_range(audio)
    else:
        mas_track = normalize(audio)    
        mas_track = compress_dynamic_range(mas_track, threshold=-20, ratio=2.5, attack=5, release=50)
        mas_track = mas_track.compress_dynamic_range(threshold=-1, ratio=20, attack=5, release=50)
    mas_track.export('mastered.wav', format='wav')
    return mas_track
    
# mix as many wavs as you like with this method for your audio mix channel
# usage :
# drum_track = AudioSegment.from_wav("drums.wav")
# bass_track = AudioSegment.from_wav("bass.wav")
# guitar_track = AudioSegment.from_wav("guitar.wav")
# vocal_track = AudioSegment.from_wav("vocals.wav")
# then call this function as :- 
# final_mix = mix_tracks(drum_track, bass_track, guitar_track, vocal_track)
def mix_tracks(*tracks):
    mixed = AudioSegment.silent(duration=len(tracks[0]))
    for track in tracks:
        mixed = mixed.overlay(track)
    mixed.export('mix_tracks_list.wav', format='wav')
    return mixed

# noise reduction
NOISE_PROFILE=None
from scipy.fftpack import fft, ifft
from scipy.signal import resample
# example of use
# drum_track = AudioSegment.from_wav("drums.wav")
# processed_audio = noise_reduction(np.array(drum_track.get_array_of_samples()).astype(np.float32))
def noise_reduction(audio_data):
    global NOISE_PROFILE
    
    if NOISE_PROFILE is None:
        NOISE_PROFILE = np.abs(fft(audio_data))
        return audio_data
    
    spectrum = fft(audio_data)
    noise_reduction_factor = 0.9
    reduced_spectrum = spectrum - noise_reduction_factor * NOISE_PROFILE
    reduced_spectrum = np.maximum(reduced_spectrum, 0)
    
    return np.real(ifft(reduced_spectrum))

# shift pitch of audio segment data
# processed_audio = pitch_shift(processed_audio, semitones=2)
def pitch_shift(audio_data, semitones):
    factor = 2 ** (semitones / 12)
    stretched = resample(audio_data, int(len(audio_data) / factor))
    return resample(stretched, len(audio_data))

# change the tempo of the wav file
def change_wav_tempo(input_fl="drums.wav",speed_adj=2.0,cf=0) -> AudioSegment:
    as_track = AudioSegment.from_wav(input_fl)
    af2 = as_track.speedup(playback_speed=speed_adj, crossfade=cf)
    af2.export("speedup.wav", format="wav")
    return af2
    
# apply reverb to a wav file and output a new one (the libs repeat but ti makes it easier to cipy and paste them
import soundfile as sf
import numpy as np
import scipy.signal as sg

def reverb_wav_file(inp="input.wav", outp="output.wav", ta=40, gain=0.5):
    tau  = 40            
    gain = 0.5     
    wav_in_name  = inp  
    wav_out_name = outp 
    x, fs = sf.read(wav_in_name)
    D = int(tau * fs / 1000)
    b = np.zeros(D+1)
    a = np.zeros(D+1)
    b[0] = 1.0
    a[0] = 1.0
    a[D] = gain
    # apply reverb
    y = sg.lfilter(b, a, x)
    y = y/np.max(np.abs(y))   
    sf.write(wav_out_name, y, fs, subtype='PCM_16')

# this is an effects library and uses this sound source library
#
from . import source

def modulated_delay(data, modwave, dry, wet):
    ''' Use LFO "modwave" as a delay modulator (no feedback)
    '''
    out = data.copy()
    for i in range(len(data)):
        index = int(i - modwave[i])
        if index >= 0 and index < len(data):
            out[i] = data[i] * dry + data[index] * wet
    return out


def feedback_modulated_delay(data, modwave, dry, wet):
    ''' Use LFO "modwave" as a delay modulator (with feedback)
    '''
    out = data.copy()
    for i in range(len(data)):
        index = int(i - modwave[i])
        if index >= 0 and index < len(data):
            out[i] = out[i] * dry + out[index] * wet
    return out


def chorus(data, freq, dry=0.5, wet=0.5, depth=1.0, delay=25.0, rate=44100, mode=0, phase=0.0):
    ''' Chorus effect is mode 0 others are weird variants 
        http://en.wikipedia.org/wiki/Chorus_effect
    '''
    length = float(len(data)) / rate
    mil = float(rate) / 1000
    delay *= mil
    depth *= mil
    if mode == 0:
        modwave = (source.sine(freq, length) / 2 + 0.5) * depth + delay
    elif mode == 1:
        modwave = (source.sawtooth(freq, length, 44100, phase) / 2 + 0.5) * depth + delay    
    elif mode == 2:
        modwave = (source.square(freq, length, 44100, phase) / 2 + 0.5) * depth + delay 
    elif mode == 3:
        modwave = (source.pluck(freq, length) / 2 + 0.5) * depth + delay        
    return modulated_delay(data, modwave, dry, wet)


def flanger(data, freq, dry=0.5, wet=0.5, depth=20.0, delay=1.0, rate=44100, mode=0):
    ''' Flanger effect
        http://en.wikipedia.org/wiki/Flanging
    '''
    length = float(len(data)) / rate
    mil = float(rate) / 1000
    delay *= mil
    depth *= mil
    if mode == 0:
        modwave = (source.sine(freq, length) / 2 + 0.5) * depth + delay
    elif mode == 1:
        modwave = (source.sawtooth(freq, length, 44100, phase) / 2 + 0.5) * depth + delay    
    elif mode == 2:
        modwave = (source.square(freq, length, 44100, phase) / 2 + 0.5) * depth + delay 
    elif mode == 3:
        modwave = (source.pluck(freq, length) / 2 + 0.5) * depth + delay         
    return feedback_modulated_delay(data, modwave, dry, wet)

def flanger1(wav_in_name, wav_out_name, d_fac=0.003, depth_fac=0.002, rate=0.25, factor=0.5):	
    s0, fs = sf.read(wav_in_name)
    length_of_s=len(s0)	
	s1 = np.zeros(length_of_s)
    d=fs*d_fac;
    depth=fs*depth_fac;
    for n in range(0,length_of_s):
        s1[n] = s0[n]    
        tau=d+depth*math.sin(2.0*math.pi*rate*(n-1)/fs)    
        t=n-tau
        m = math.floor(t)
        delta=t-m
        if m>0 & m+1<=length_of_s:
            s1[n]=s1[n]+delta*s0[m+1]+(1.0-delta)*s0[m]	
        if n >= 1 :
            if s1[n-1] < s1[n]:
                smax = s1[n]
        else:
            smax = s1[0]
    for n in range(0,length_of_s):
        s1[n] = s1[n] / smax * factor	
    sf.write(wav_out_name, s1, fs, subtype='PCM_16')	
    return s1

def flanger2(wav_in_name, wav_out_name, d_fac=0.003, depth_fac=0.002, rate=0.25, factor=0.5, factor2=0.7):	
    s0, fs = sf.read(wav_in_name)
    length_of_s=len(s0)	
	s1 = np.zeros(length_of_s)
    d=fs*d_fac;
    depth=fs*depth_fac;
    for n in range(0,length_of_s):
        s1[n] = s0[n]    
        tau=d+depth*math.sin(2.0*math.pi*rate*(n-1)/fs)    
        t=n-tau
        m = math.floor(t)
        delta=t-m
        if m>0 & m+1<=length_of_s:
            s1[n]=s1[n]+factor2*(delta*s1[m+1]+(1.0-delta)*s1[m])	
        if n >= 1 :
            if s1[n-1] < s1[n]:
                smax = s1[n]
        else:
            smax = s1[0]
    for n in range(0,length_of_s):
        s1[n] = s1[n] / smax * factor	
    sf.write(wav_out_name, s1, fs, subtype='PCM_16')	
    return s1
    
def tremolo(data, freq, dry=0.5, wet=0.5, rate=44100, mode=0):
    ''' Tremolo effect
        http://en.wikipedia.org/wiki/Tremolo
    '''
    length = float(len(data)) / rate
    if mode == 0:
        modwave = (source.sine(freq, length) / 2 + 0.5)
    elif mode == 1:
        modwave = (source.sawtooth(freq, length, 44100, phase) / 2 + 0.5)    
    elif mode == 2:
        modwave = (source.square(freq, length, 44100, phase) / 2 + 0.5)  
    elif mode == 3:
        modwave = (source.pluck(freq, length) / 2 + 0.5)  
    return (data * dry) + ((data * modwave) * wet)

def tremolo1(wav_in_name, wav_out_name, depth=0.8, rate=8):	
    x, fs = sf.read(wav_in_name)
    length_of_s=len(x)	
	s1 = np.zeros(length_of_s)
    for n in range(0,length_of_s):    
        a=1.0+depth*sin(2.0*math.pi*rate*(n - 1)/fs)    
        s1[n] = a*x[n]
    sf.write(wav_out_name, s1, fs, subtype='PCM_16')	
    return s1

def vibrato(wav_in_name, wav_out_name, d_fac=0.025, depth_fac=0.01, rate=0.5):	
    s0, fs = sf.read(wav_in_name)
    length_of_s=len(s0)	
	s1 = np.zeros(length_of_s)
    d=fs*d_fac;
    depth=fs*depth_fac;
    for n in range(0,length_of_s):    
        tau=d+depth*math.sin(2.0*math.pi*rate*(n-1)/fs)    
        t=n-tau
        m = math.floor(t)
        delta=t-m
        if m>0 & m+1<=length_of_s:
            s1[n]=s1[n]+delta*s0[m+1]+(1.0-delta)*s0[m]		
    sf.write(wav_out_name, s1, fs, subtype='PCM_16')	
    return s1

def delay(wav_in_name, wav_out_name, a=0.8, d_fac=0.375, repeat=1):	
    x, fs = sf.read(wav_in_name)
    length_of_s=len(x)	
	s1 = np.zeros(length_of_s)
    d=fs*d_fac
    for n in range(0,length_of_s): 
        s1[n] = s0[n]
        for i in range(0,repeat):		
            m=math.floor(n-i*d) 
            if m>=1	:
                s1[n]=s1[n]+math.pow(a,i)*s0[m]			
    sf.write(wav_out_name, s1, fs, subtype='PCM_16')	
    return s1

def chorus(wav_in_name, wav_out_name, d_fac=0.025, depth_fac=0.01, rate=0.1):	
    s0, fs = sf.read(wav_in_name)
    length_of_s=len(s0)	
	s1 = np.zeros(length_of_s)
    d=fs*d_fac;
    depth=fs*depth_fac;
    for n in range(0,length_of_s): 
        s1[n] = s0[n]	
        tau=d+depth*math.sin(2.0*math.pi*rate*(n-1)/fs)    
        t=n-tau
        m = math.floor(t)
        delta=t-m
        if m>0 & m+1<=length_of_s:
            s1[n]=s1[n]+delta*s0[m+1]+(1.0-delta)*s0[m]		
    sf.write(wav_out_name, s1, fs, subtype='PCM_16')	
    return s1
    
# apply these effects to wav files 
# the default parameter_lists for each effect are writen here (modify these or make new ones and pass them to the function
#
# dry=0.5, wet=0.5, depth=1.0, delay=25.0, rate=44100, mode=0, phase=0.0
chorus_param_list = [0.5, 0.5, 1.0, 25.0, 44100, 0, 0.0]
# dry=0.5, wet=0.5, depth=20.0, delay=1.0, rate=44100, mode=0
flanger_param_list = [0.5, 0.5, 20.0, 1.0, 44100, 0]
# dry=0.5, wet=0.5, rate=44100, mode=0
trem_param_list = [ 0.5, 0.5, 44100, 0 ]
def effect_wav_file(inp="input.wav", outp="output.wav", mode=0, freq=C_5_4, parameter_data_list):
    
    wav_in_name  = inp  
    wav_out_name = outp 
    x, fs = sf.read(wav_in_name)
    if mode == 0:
        y = chorus(x, freq, parameter_data_list[0], parameter_data_list[1], parameter_data_list[2], parameter_data_list[3], parameter_data_list[4], parameter_data_list[5], parameter_data_list[6])  
    elif mode == 1:
        y = flanger(x, freq, parameter_data_list[0], parameter_data_list[1], parameter_data_list[2], parameter_data_list[3], parameter_data_list[4], parameter_data_list[5])  
    elif mode == 2:
        y = tremolo(x, freq, parameter_data_list[0], parameter_data_list[1], parameter_data_list[2], parameter_data_list[3])         
    sf.write(wav_out_name, y, fs, subtype='PCM_16')
    
# convert mp3 to wav
def mp3_2_wav(inp="input.mp3", outp="output.wav"):
    af = AudioSegment.from_mp3(inp)
    af2.export(outp, format="wav")

# convert mp3 to wav
def wav_2_mp3(inp="input.wav", outp="output.mp3"):
    af = AudioSegment.from_wav(inp)
    af2.export(outp, format="mp3")
    
# ---------------- mixing with a mic stream (live instrument or vocal) and a wav file looped overlayed --------------
# based on this code :- https://qiita.com/tocoteron/items/f82babc96e05a80b810a
#
import numpy as np
import wave
import pyaudio
import socket
import threading
import ctypes

#   usage example :-
#   mss_client = MixedWavMicSoundStream("sample.wav")
#   mss_client.start()
#   mss_client.join()   
class MixedWavMicSoundStream(threading.Thread):
    def __init__(self, wav_filename):
        threading.Thread.__init__(self)
        self.WAV_FILENAME = wav_filename

    def __del__(self):
        mic_stream.stop_stream()
        mic_stream.close()
        stream.stop_stream()
        stream.close()        
        audio.terminate()
        
    def run(self):
        audio = pyaudio.PyAudio()

        wav_file = wave.open(self.WAV_FILENAME, 'rb')

        FORMAT = pyaudio.paInt16
        CHANNELS = wav_file.getnchannels()
        RATE = wav_file.getframerate()
        CHUNK = 1024

        mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
        stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, output=True, frames_per_buffer=CHUNK)
        
        while RUN_IT:

            wav_data = wav_file.readframes(CHUNK)
            mic_data = mic_stream.read(CHUNK)

            if wav_data == b'':
                wav_file.rewind()
                wav_data = wav_file.readframes(CHUNK)

            data_chunk = self.mix_sound(wav_data, mic_data, CHANNELS, CHUNK, 0.5, 0.5)
            if not data_chunk:
                break
            stream.write(data_chunk)

        mic_stream.stop_stream()
        mic_stream.close()
        stream.stop_stream()
        stream.close()
        
        audio.terminate()


    def mix_sound(self, data1, data2, channels, frames_per_buffer, volume1, volume2):

        if volume1 + volume2 > 1.0:
            return None

        decoded_data1 = np.frombuffer(data1, np.int16).copy()
        decoded_data2 = np.frombuffer(data2, np.int16).copy()

        decoded_data1.resize(channels * frames_per_buffer, refcheck=False)
        decoded_data2.resize(channels * frames_per_buffer, refcheck=False)

        return (decoded_data1 * volume1 + decoded_data2 * volume2).astype(np.int16).tobytes()

# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
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

# if you want to use a clean mic class instead of the one above with no wav file playing overlayed then uncomment and try this..
#
#  import clean_mic_class
#  mic_client = clean_mic_class.CleanMicSoundStream() 
#  mic_client.start() 
# 
#  mic_client.join() - when stopping
 
# ---------------- start all 3 channels [ synth, midi, audio ] ----------------------
if __name__ == '__main__':

    # compose midi as per your program
    make_midi_arrangement(SONG_TEMPO)
    # make a wav file from the midi arrangement above
    wave = midi2wave("mid_arrangement.mid", SONG_TEMPO)   
    # now join it with the sample loop
    join_two_wavs("out_midi.wav","my_sample_loop.wav")    
    # set the loop of audio to play 
    set_file_name("all_mixed.wav")

    # define the aynth thread    
    # loop_synth_task = threading.Thread(target=loop_synth, daemon=True)
    loop_synth_task = twe(name = 'Thread Synth', target=loop_synth, args=(), kwargs={})
    # define the midi thread
    # loop_midi_task = threading.Thread(target=loop_midi_arrangement, daemon=True)
    loop_midi_task = twe(name = 'Thread midi', target=loop_midi_arrangement, args=(), kwargs={})
    # define the wave thread comment out and use below if also want input mic active   
    #loop_wave_task = threading.Thread(target=loop_sample, daemon=True)
    loop_wave_task = twe(name = 'Thread beat', target=loop_sample, args=(), kwargs={})
    # wave stream has a mic/audio_input active too... then comment out above and uncomment below
    # mss_client = MixedWavMicSoundStream(FILENM)    
    try:
        loop_synth_task.start()
        loop_midi_task.start()    
        loop_wave_task.start()
        # mss_client.start()
    except KeyboardInterrupt:
        RUN_IT=0
        loop_synth_task.join()
        loop_midi_task.join()    
        loop_wave_task.join() 
        # mss_client.join()   
