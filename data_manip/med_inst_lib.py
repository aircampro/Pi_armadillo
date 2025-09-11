# useful functions used in ECG and blood pressure analysis (some also useful for sound)
#
import numpy as np
from scipy import stats
import numpy as np
import scipy.signal
import scipy.io.wavfile
import matplotlib.pyplot as plt
from sklearn.metrics import mean_absolute_error as mae

def lowpass(data: np.ndarray, cutoff: float, sample_rate: float, poles: int = 5):
    sos = scipy.signal.butter(poles, cutoff, 'lowpass', fs=sample_rate, output='sos')
    filtered_data = scipy.signal.sosfiltfilt(sos, data)
    return filtered_data

def do_lopass(lpf=50, egc_rec='ecg.wav'):
    # Load sample data from a WAV file
    sample_rate, data = scipy.io.wavfile.read(egc_rec)
    times = np.arange(len(data))/sample_rate
    # Apply a lpf Hz low-pass filter to the original data
    filtered = lowpass(data, lpf, sample_rate)
	return filtered

def highpass(data: np.ndarray, cutoff: float, sample_rate: float, poles: int = 5):
    sos = scipy.signal.butter(poles, cutoff, 'highpass', fs=sample_rate, output='sos')
    filtered_data = scipy.signal.sosfiltfilt(sos, data)
    return filtered_data

def do_highpass(hpf=20, egc_rec='ecg.wav'):
    # Load sample data from a WAV file
    sample_rate, data = scipy.io.wavfile.read(egc_rec)
    times = np.arange(len(data))/sample_rate
    # Apply a hpf Hz high-pass filter to the original data
    filtered = highpass(data, 20, sample_rate)
    return filtered

def bandpass(data: np.ndarray, edges: list[float], sample_rate: float, poles: int = 5):
    sos = scipy.signal.butter(poles, edges, 'bandpass', fs=sample_rate, output='sos')
    filtered_data = scipy.signal.sosfiltfilt(sos, data)
    return filtered_data

def do_bandpass(lpf=50, hpf=10, egc_rec='ecg.wav'):
    # Load sample data from a WAV file
    sample_rate, data = scipy.io.wavfile.read(egc_rec)
    times = np.arange(len(data))/sample_rate
    # Apply a 10-50 Hz high-pass filter to the original data
    filtered = bandpass(data, [hpf, lpf], sample_rate)
    return filtered, times

def do_gustafson(co=80, egc_rec='ecg.wav'):
    # Load sample data from a WAV file
    sample_rate, data = scipy.io.wavfile.read(egc_rec)
    times = np.arange(len(data))/sample_rate
    # Isolate a small portion of data to inspect
    segment = data[350:400]
    # Create a 5-pole low-pass filter with an 80 Hz cutoff
    b, a = scipy.signal.butter(5, co, fs=sample_rate)
    # Apply the filter using the default edge method (padding)
    filtered_pad = scipy.signal.filtfilt(b, a, segment)
    # Apply the filter using Gustafsson's method
    filtered_gust = scipy.signal.filtfilt(b, a, segment, method="gust")
    return filtered_gust, filtered_pad

def do_convol(egc_rec='ecg.wav'):
    # Load sample data from a WAV file
    sample_rate, data = scipy.io.wavfile.read(egc_rec)
    times = np.arange(len(data))/sample_rate
    # create a Hanning kernel 1/50th of a second wide
    kernel_width_seconds = 1.0/50
    kernel_size_points = int(kernel_width_seconds * sample_rate)
    kernel = np.hanning(kernel_size_points)
    # normalize the kernel
    kernel = kernel / kernel.sum()
    # Create a filtered signal by convolving the kernel with the original data
    filtered = np.convolve(kernel, data, mode='valid')	
    return filtered

# Mean absolute error MAE 
# model.fit(y_train, X_train)
# y_pred = model.predict(X_test)
#
def do_mae(y_test, y_pred):
    return mae(y_test, y_pred)

#%% Convert voltage to mmHg for blood pressure sensor
#
def volts_mmHg(volts, cFact = 2.50, aV = 0.710):
    ambientV = aV                                       #0.675 # from calibration
    mmHg_per_kPa = 7.5006157584566                      # from literature
    kPa_per_V = 50                                      # 20mV per 1kPa / 0.02 or * 50 - from sensor datasheet
    corrFact = cFact                                    # from calibration
    ymmHg = (volts - ambientV)  * mmHg_per_kPa * kPa_per_V * corrFact 
    return ymmHg

def calc_bp(ymmHg, lpf=5, f005=0.3, llp=.8):
    # 5 Hz LP filter
    f5 = lpf
    bLP, aLP = signal.butter(4, f5/fs*2, 'lowpass')
    yfLP = signal.lfilter(bLP, aLP, ymmHg)
    # 0.5 Hz HP filter
    f05 = f005 
    bHP, aHP = signal.butter(4, f05/fs*2, 'highpass')
    yfHP = signal.lfilter(bHP, aHP, yfLP)
    f5 = llp
    bLLP, aLLP = signal.butter(4, f5/fs*2, 'lowpass')
    yfLLP = signal.lfilter(bLLP, aLLP, ymmHg)
    yfBP = yfLP-yfHP
    localMax, _ = signal.find_peaks(yfHP, prominence = 0.3 )   
    # get values of local maximas in pressure and time
    yMaximas = yfBP[localMax]
    tMaximas = t[localMax]
    # the local max values of the oscillation
    oscMax = yfHP[localMax]
    # get indice of overal max pressure 
    xPumpedUp = np.argmax(yMaximas)
    # the pressure that was pumped up to
    yPumpedUP = yMaximas[xPumpedUp]
    # the start time of the deflation 
    tPumpedUP = tMaximas[xPumpedUp]
    localMin, _ = signal.find_peaks(-yfHP, prominence = 0.3 )
    # get values of local maximas in pressure and time
    yMinima = yfBP[localMin]
    tMinima = t[localMin]
    # the local max values of the oscillation
    oscMin = yfHP[localMin]
    deltaT = np.zeros(len(tMaximas))
    delta2T = np.zeros(len(tMaximas))
    validCnt = 0
    oscStartInd = 0
    oscEndInd = 0
    for i in range(1,len(tMaximas)-1):
        deltaT[i] = tMaximas[i]-tMaximas[i-1]
        delta2T[i] = deltaT[i]-deltaT[i-1]
    
        if oscStartInd == 0 :
            # check for start of oscillogram: 
            if np.abs(delta2T[i]) < 0.2 and i > (xPumpedUp + 5) :
                validCnt += 1
                if validCnt == 5 :
                    oscStartInd = i - (validCnt-1)
            else:
                validCnt = 0
        elif oscEndInd == 0:
            #check for end of oscillogram     
            if (oscMax[oscStartInd]*1.2) > oscMax[i]: # more info on left side
                oscEndInd = i-3
    if oscEndInd == 0:
        oscEndInd = len(tMaximas)-4
    # data for processing:
    yMaximasP = yMaximas[oscStartInd:oscEndInd+1]
    tStart_a = np.where(yfBP == yMaximasP[0])
    iStart = int(tStart_a[0]+100)
    tEnd_a = np.where(yfBP == yMaximasP[-1])
    iEnd = int(tEnd_a[0]-100)
    tMaxP = tMaximas[oscStartInd:oscEndInd+1]
    oscMaxP = oscMax[oscStartInd:oscEndInd+1]
    deltaP = deltaT[oscStartInd:oscEndInd+1]
    tP = t[iStart:iEnd+1]
    yhpP = yfHP[iStart:iEnd+1]
    ylpP = yfLP[iStart:iEnd+1]
    ybpP = yfBP[iStart:iEnd+1]
    yllpP = yfLLP[iStart:iEnd+1]
    # make sure minimas are (at least) defined for every maxima
    minStart = np.argmax(tMinima>tMaximas[oscStartInd])-1
    minEnd = np.argmax(tMinima>tMaximas[oscEndInd])
    tMinP = tMinima[minStart:minEnd+1]
    yMinimasP = yMinima[minStart:minEnd+1]
    oscMinP = oscMin[minStart:minEnd+1]
    dMaxMin = oscMaxP - oscMinP[0:len(tMaxP)]
    avPulse = np.average(deltaP)
    pulse=60/avPulse
    print("Pulse: ", np.around(pulse, decimals=1))
    # calculate systolic and diastolic blood pressure from derrivative
    dPres = np.zeros(len(dMaxMin))
    dPresN = np.zeros(len(dMaxMin))
    dPresNP = np.zeros(len(dMaxMin))
    yMaxPCalc = yMaximasP[::-1]
    dMaxMinCalc = dMaxMin[::-1] #oscMaxP[::-1]#
    for i in range(1,len(dMaxMin)):
        dPres[i] = (dMaxMinCalc[i]-dMaxMinCalc[i-1])
        dPresN[i] = (dMaxMinCalc[i]-dMaxMinCalc[i-1])/(tMaxP[i]-tMaxP[i-1])
        dPresNP[i] = (dMaxMinCalc[i]-dMaxMinCalc[i-1])/(yMaxPCalc[i]-yMaxPCalc[i-1])
    return dPres, dPresN, dPresNP

l = []
def moving_average(v):
    global l
    if len(l) == 0 or len(l) == 1:
        l.append(v)
    else:
        for o in range(0,len(l)-1):
            l[o] = l[o+1]
        l[o+1] = v
    return sum(l)/len(l)

def bp_check(systole, diastole): 
  if((systole<120) and (diastole<80)):
    return ("Normal Blood Pressure")
  elif ((systole>=120 and systole<130) and (diastole<80)):
    return ("Elevated Blood Pressure")
  elif ((systole>=130 and systole<140) or (diastole>=80 and diastole<=89)):
    return ("First Stage High Blood Pressure")
  else:
    return ("Second Stage High Blood Pressure")

def pearsons_correlation(x, y):
    return stats.pearsonr(x, y)

