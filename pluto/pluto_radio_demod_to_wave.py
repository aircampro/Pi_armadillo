#sdr=SdrInit() to set Pluto and get the object.
#Samples are extracted with sdr.rx(), and delayed detection processing is performed with demLoop() to obtain audio data.
#After delayed detection, downsample to the sample rate of the audio device and apply de-emphasis.
#In the main loop, the audio data retrieved by demLoop is stored in a buffer (pack) once, and when data accumulates 
#from the chunk buffer size of pyaudio, it is extracted for each chunk size, converted to binary data for pyaudio, and 
#written to a wav file
#PlutoFM.py
import adi                          #pluto 
#import matplotlib.pyplot as plt     graph
import numpy as np
import scipy.signal                 # US/DS/Filter
import math                         # pi.sin.cos
#import pyaudio
import wave

# System parameters
################################################
#-- application
################################################
SND_FS = 44100 # 48000
SND_CHUNK = 1024*2
SND_CH_OUT = 1

sample_rate_audio = SND_FS
################################################
#-- baseband IQ parameter
################################################
#(i) sample_rate_audio
decimate_demod = 5 
sample_rate_demod = sample_rate_audio * decimate_demod
################################################
#-- radio interface param
################################################
# PLUTO
# LO 325MHz to  3800MHz
# BW < 20MHz
# FS 521ksps to 61.4Msps # cat spec is 65.2Ksps to  61.4Msps
################################################
#(i) sample_rate_demod
decimate_capture = 4 
sdrSample_rate = sample_rate_demod * decimate_capture

sdrBufSize = 1024*20
sdrFcenter = 82500000 # 中心周波数82.5MHzの場合
#sdrFcenter = 80000000
sdrBandWidth = 400000
################################################
# Create radio object
def SdrInit():
    # (i) 
    # (o) radio Object
    sdr = adi.Pluto()
    sdr.rx_rf_bandwidth = sdrBandWidth
    sdr.rx_lo = sdrFcenter
    sdr.sample_rate = sdrSample_rate
    sdr.rx_buffer_size=sdrBufSize
    #sdr.tx_lo = 2000000000
    #sdr.tx_cyclic_buffer = True
    #sdr.tx_hardwaregain = -30
    sdr.gain_control_mode = "slow_attack"
    # Read back properties from hardware
    #print(sdr.rx_hardwaregain)
    # Get complex data back
    sdr.rx_enabled_channels = [0]
    return sdr

#=========================================================
#　FM　Demodulation
#=========================================================
deemp_n = np.array([1,1])*2
deemp_d = np.array([5.41,3.41])
demLastIQ=0
def demLoop(demLastIQ, sdr):
    smp = sdr.rx()                                             # must return vector of at least 27
    decim = scipy.signal.decimate(smp,decimate_capture) 
    bbIQ = np.insert(decim,0,demLastIQ)
    #bbIQ = decim
    demLastIQ = decim[-1]
    #------------------------
    # Delay detection
    # -+-[Delay]-- [x]---[ angle ]-->
    # | ^
    # +-[Conj ]----+
    #------------------------
    dem = np.angle(bbIQ[1:] * np.conj(bbIQ[:-1])) / math.pi 
    #  decimate to audio sample rate
    aud = scipy.signal.decimate(dem,decimate_demod)
    #deEmphasis
    demp = scipy.signal.lfilter(deemp_n,deemp_d,aud)
    return demp,demLastIQ

def bandpass(data: np.ndarray, edges: list[float], sample_rate: float, poles: int = 5):
    sos = scipy.signal.butter(poles, edges, 'bandpass', fs=sample_rate, output='sos')
    filtered_data = scipy.signal.sosfiltfilt(sos, data)
    return filtered_data

# do bandpass on ecg between 10 and 50 hz
def do_bandpass(data, lpf=50, hpf=10, sr=48000):
    # Load sample data from a WAV file
    sample_rate = sr
    times = np.arange(len(data))/sample_rate
    # Apply a 10-50 Hz high-pass filter to the original data
    filtered = bandpass(data, [hpf, lpf], sample_rate)
    return filtered, times

def writeWav(filename, data, bp=1, sr=48000):    
    sampleRate = sr
    w = wave.Wave_write(filename)
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(sampleRate)
    #w.writeframes(array.array('h', data*32767).tobytes())
    if bp == 1:
        data = do_bandpass(data)
    w.writeframes(array.array('h', data).tobytes())
    w.close()

if __name__ == "__main__":
    #===========================================================
    #--- Create Software Defined Radio Object to receive signals
    #==========================================================
    sdr = SdrInit()
    #====================================================
    #--- setup Audio
    #====================================================
    #pyAud = pyaudio.PyAudio()
    #sndStrm = pyAud.open(rate=SND_FS,channels=SND_CH_OUT,format=pyaudio.paInt16,input=False,output=True)
    #====================================================
    #--- setup FIFO
    #====================================================
    pack = np.array([])
    #====================================================
    #--- FM Receive loop
    #====================================================
    idx = 1
    for cnt in range(2000): # while: 
        #====================================================
        #--- Output to the sound device when it accumulates more than the chunk size
        #====================================================
        while pack.size > SND_CHUNK :
            aBuf = pack[0:SND_CHUNK]*8000
            sndBuf = aBuf.astype(np.int16).tobytes()
            #sndStrm.write(sndBuf)
            writeWav(f"egc{idx}.wav", sndBuf)
            pack = pack[SND_CHUNK:]
        #====================================================
        #--- Receive a new IF signal, demodulate it into audio data, and put it into FIFO.
        #====================================================
        aud, demLastIQ = demLoop(demLastIQ, sdr)
        pack = np.append(pack, aud)
    #====================================================
    # teardown Processing
    #====================================================
    #sndStrm.stop_stream()
    #sndStrm.close()
    pyAud.terminate()
	
	