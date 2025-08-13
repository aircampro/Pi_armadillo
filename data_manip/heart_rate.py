# Example codes to process heart beat information ECG and PPG measurements
# Two often used ways of measuring the heart rate are the electrocardiogram (ECG) and the Photoplethysmogram (PPG). 
#
# This shows examples of pre-processing the data and making other calculations from that data using the heartpy library. 
# the data is either shown as a list of recorded values or as a csv file
#

# Frequency Domain - FFT Welch's Method (using scipy)
#
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

rng = np.random.default_rng()
# Generate a test signal, a 2 Vrms sine wave at 1234 Hz, corrupted by 0.001 V**2/Hz of white noise sampled at 10 kHz.
fs = 10e3
N = 1e5
amp = 2*np.sqrt(2)
freq = 1234.0
noise_power = 0.001 * fs / 2
time = np.arange(N) / fs
x = amp*np.sin(2*np.pi*freq*time)
x += rng.normal(scale=np.sqrt(noise_power), size=time.shape)
# Compute and plot the power spectral density.
f, Pxx_den = signal.welch(x, fs, nperseg=1024)
plt.semilogy(f, Pxx_den)
plt.ylim([0.5e-3, 1])
plt.xlabel('frequency [Hz]')
plt.ylabel('PSD [V**2/Hz]')
plt.show()
# If we average the last half of the spectral density, to exclude the peak, we can recover the noise power on the signal.
np.mean(Pxx_den[256:])
# Now compute and plot the power spectrum.
f, Pxx_spec = signal.welch(x, fs, 'flattop', 1024, scaling='spectrum')
plt.figure()
plt.semilogy(f, np.sqrt(Pxx_spec))
plt.xlabel('frequency [Hz]')
plt.ylabel('Linear spectrum [V RMS]')
plt.show()
# If we average the last half of the spectral density, to exclude the peak, we can recover the noise power on the signal.
np.mean(Pxx_den[256:])
# Now compute and plot the power spectrum.
f, Pxx_spec = signal.welch(x, fs, 'flattop', 1024, scaling='spectrum')
plt.figure()
plt.semilogy(f, np.sqrt(Pxx_spec))
plt.xlabel('frequency [Hz]')
plt.ylabel('Linear spectrum [V RMS]')
plt.show()

# Frequency Domain - Lomb-Scargle Method
# t array_like or Quantity [:ref: ‘time’]
# sequence of observation times
#
# y array_like or Quantity
# sequence of observations associated with times t
# dy float, array_like, or Quantity, optional
# error or sequence of observational errors associated with times t
# fit_mean bool, optional
# if True, include a constant offset as part of the model at each frequency. This can lead to more accurate results, especially in the case of incomplete phase coverage.
# center_data bool, optional
# if True, pre-center the data by subtracting the weighted mean of the input data. This is especially important if fit_mean = False
# nterm sint, optional
# number of terms to use in the Fourier fit
# normalization{‘standard’, ‘model’, ‘log’, ‘psd’}, optional
# Normalization to use for the periodogram.
# The Lomb-Scargle periodogram is designed to detect periodic signals in unevenly spaced observations.
#

# you can use scipy
from scipy.signal import lombscargle

# Choose a period grid
periods = np.linspace(0.2, 1.4, 4000)
ang_freqs = 2 * np.pi / periods

# compute the (unnormalized) periodogram
# note pre-centering of y values!
power = lombscargle(t, mag - mag.mean(), ang_freqs)

# normalize the power
N = len(t)
power *= 2 / (N * mag.std() ** 2)

# plot the results
fig, ax = plt.subplots()
ax.plot(periods, power)
ax.set(ylim=(0, 0.8), xlabel='period (days)', ylabel='Lomb-Scargle Power');

# Example you can also use astropy library
# To detect periodic signals in unevenly spaced observations, consider the following data:
#
rand = np.random.default_rng(42)
t = 100 * rand.random(100)
y = np.sin(2 * np.pi * t) + 0.1 * rand.standard_normal(100)
# These are 100 noisy measurements taken at irregular times, with a frequency of 1 cycle per unit time.
# The Lomb-Scargle periodogram, evaluated at frequencies chosen automatically based on the input data, can be computed as follows using the LombScargle class:
from astropy.timeseries import LombScargle
frequency, power = LombScargle(t, y).autopower()
# Plotting the result with Matplotlib gives:
fig, ax = plt.subplots()
ax.plot(frequency, power)

# To tune the heuristic using keywords passed to the autopower() method:
dy=None
frequency, power = LombScargle(t, y, dy).autopower(nyquist_factor=2)
len(frequency), frequency.min(), frequency.max()
# Here the highest frequency is two times the average Nyquist frequency. If we increase the nyquist_factor, we can probe higher frequencies:
frequency, power = LombScargle(t, y, dy).autopower(nyquist_factor=10)
len(frequency), frequency.min(), frequency.max()
# Alternatively, we can use the power() method to evaluate the periodogram at a user-specified set of frequencies:
frequency = np.linspace(0.5, 1.5, 1000)
power = LombScargle(t, y, dy).power(frequency)
frequency = np.linspace(0.1, 1.9, 100)
power = LombScargle(t, y, dy).power(frequency)
fig, ax = plt.subplots()
ax.plot(frequency, power)
frequency, power = LombScargle(t, y, dy).autopower(minimum_frequency=0.1, maximum_frequency=1.9)
print(len(frequency))
fig, ax = plt.subplots()
ax.plot(frequency, power)

# One interesting property of the heart is that the frequency with which it beats is strongly 
# influenced by breathing, through the autonomous nervous system. It is one of the reasons why deep 
# breaths can calm nerves. We can also exploit this relationship to extract breathing rate from a segment 
# of heart rate data. For example, using a dataset from [1] which contains both CO2 capnometry signals as 
# well as PPG signals, we can see the relationship between breathing and the RR-intervals clearly. 
# Below are plotted the CO2 capnometry signal (breathing signal measured at the nose), 
# as well as the (upsampled) signal created by the RR-intervals
#
import heartpy as hp

# calculate the breathing rate
data = hp.get_data('data.csv')
fs = 100.0
working_data, measures = hp.process(data, fs, report_time=True)
print('breathing rate is: %s Hz' %measures['breathingrate'])

# filter the data
from .datautils import get_data, load_exampledata
data, _ = load_exampledata(0)
filtered = hampel_filter(data, filtsize = 6)
print('%i, %i' %(data[1232], filtered[1232]))

# Or specify a range (here: 0.75 - 3.5Hz), outside of which all frequencies are cut out.
filtered = filter_signal(data, cutoff = [0.75, 3.5], sample_rate = 100.0,order = 3, filtertype='bandpass')
print(np.around(filtered[0:6], 3))

# Hampel Correction
# apply altered version of hampel filter to suppress noise.
from .datautils import get_data, load_exampledata
data, _ = load_exampledata(1)
filtered = hampel_correcter(data, sample_rate = 116.995)
print(filtered)

x = [1, 3, 4, 5, 6, 7, 5, 3, 1, 1]
smoothed = smooth_signal(x, sample_rate = 2, window_length=4, polyorder=2)
print(np.around(smoothed[0:4], 3))

# Part of peak detection pipeline. Uses moving average as a peak detection threshold and rises it stepwise. 
# Determines best fit by minimising standard deviation of peak-peak distances as well as getting a 
# bpm that lies within the expected range.

from heartpy.datautils import rolling_mean
data, _ = hp.load_exampledata(0)
rol_mean = rolling_mean(data, windowsize = 0.75, sample_rate = 100.0)
# We can then call this function and let the optimizer do its work:
wd = fit_peaks(data, rol_mean, sample_rate = 100.0)
# Now the wd dict contains the best fit paramater(s):
print(wd['best'])

# This indicates the best fit can be obtained by raising the moving average with 20%.
#
# The results of the peak detection using these parameters are included too. To illustrate, these are the first five detected peaks:
print(wd['peaklist'][0:5])

# and the corresponding peak-peak intervals:
print(wd['RR_list'][0:4])

# First let’s import the module and load the data
data, timer = hp.load_exampledata(1)
sample_rate = hp.get_samplerate_mstimer(timer)
# After loading the data we call the function like so:
filtered_data = enhance_ecg_peaks(data, sample_rate, iterations = 3)
# By default the module uses the mean to aggregate convolutional outputs. It is also possible to use the median.
filtered_data = enhance_ecg_peaks(data, sample_rate, iterations = 3, aggregation = 'median', notch_filter = False)
print(filtered_data)

# Function that flips raw signal with negative mV peaks to normal ECG.
# Required for proper peak finding in case peaks are expressed as negative dips.
x = [200, 300, 500, 900, 500, 300, 200]
# We can call the function. If keep_range is False, the signal will be inverted relative to its mean.
print(flip_signal(x, keep_range=False))
# However, by specifying keep_range, the inverted signal will be put ‘back in place’ in its original range.
print(flip_signal(x, keep_range=True))
# It’s also possible to use the enhance_peaks function:
print(flip_signal(x, enhancepeaks=True))

data = hp.load_exampledata(0)
fs = 100.0 #example file 0 is sampled at 100.0 Hz
working_data, measures = hp.process(data, fs, report_time=True)
print(measures['bpm'])                                        # returns BPM value
print(measures['rmssd'])                                      # returns RMSSD HRV measure

# You can also use Pandas if you so desire
import pandas as pd
df = pd.read_csv("data.csv", names=['hr'])
# note we need calc_freq if we want frequency-domain measures
working_data, measures = hp.process(df['hr'].values, fs, calc_freq=True)
print(measures['bpm'])
print(measures['lf/hf'])

# heart rate
hrdata = hp.get_data('data2.csv', column_name='hr')
timerdata = hp.get_data('data2.csv', column_name='timer')
working_data, measures = hp.process(hrdata, hp.get_samplerate_mstimer(timerdata))

# plot with different title
hp.plotter(working_data, measures, title='Heart Beat Detection on Noisy Signal')

