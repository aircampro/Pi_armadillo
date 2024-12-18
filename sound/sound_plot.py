#!/usr/bin/env python
#
# record audio and analyse it - example animal welfare or vibration monitoring
# you could also use the plot pictures to compare using openCV for a certain situation or 
# via machine learning model .e.g. random forrest classifier
#
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
import matplotlib.pyplot as plt
from numpy import array, diff, where, split
from scipy import arange
import soundfile
import numpy, scipy
import matplotlib
matplotlib.use('tkagg')

# plot intensity
#
def plot_intensity(I, P, x, y, disp):
    fig, ax = plt.subplots()
    ax.quiver(x, y, I[0], I[1], color='black', angles='xy', scale_units='xy')
    p = P.real
    ax.imshow(p, cmap=plt.cm.bwr, vmin=-1, vmax=1, interpolation='bicubic', origin='lower', extent=disp)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.tight_layout()
    output_image_file3 = 'intensity.png'
    # Save the plot as a PNG image
    fig.savefig(output_image_file3)
    #fig.show()

# filter the electric hum sound is to set the amplitudes of the FFT values around 60 Hz to 0, 
#.
def remove_hum(freq, fft_spectrum):
    for i,f in enumerate(freq):
        if f < 62 and f > 58:
            fft_spectrum[i] = 0.0
        if f < 21 or f > 20000:
            fft_spectrum[i] = 0.0
    return fft_spectrum
    
# function to report ammount of sound over a ceratin amplitude
#
def amp_over(audio_data, threshold):
    cnt = 0
    sam_tot = 0
    for sam in audio_data:
        if sam > threshold:
            cnt += 1
        sam_tot += sam	
    sam_avg = sam_tot / len(audio_data)
    return sam_avg, cnt / len(audio_data)

# function to report ammount of sound over and below a ceratin freqencies
#
def freq_over(f_data, s_data, threshold, threshold2. t3):
    cnt = 0
    cnt2 = 0
    sam_tot = 0
    s_data = np.abs(s_data)
    for i,sam in enumerate(f_data):
        if sam > threshold and s_data[i] > t3:
            cnt += 1
        elif sam < threshold2 and s_data[i] > t3:
            cnt2 += 1
        sam_tot += sam	
    sam_avg = sam_tot / len(f_data)
    return sam_avg, cnt/len(f_data), cnt2/len(f_data)

def findPeak(magnitude_values, noise_level=2000):
    
    splitter = 0
    # zero out low values in the magnitude array to remove noise (if any)
    magnitude_values = numpy.asarray(magnitude_values)        
    low_values_indices = magnitude_values < noise_level  # Where values are low
    magnitude_values[low_values_indices] = 0  # All low values will be zero out
    
    indices = []
    
    flag_start_looking = False
    
    both_ends_indices = []
    
    length = len(magnitude_values)
    for i in range(length):
        if magnitude_values[i] != splitter:
            if not flag_start_looking:
                flag_start_looking = True
                both_ends_indices = [0, 0]
                both_ends_indices[0] = i
        else:
            if flag_start_looking:
                flag_start_looking = False
                both_ends_indices[1] = i
                # add both_ends_indices in to indices
                indices.append(both_ends_indices)
                
    return indices

def extractFrequency(indices, freq_threshold=2):
    
    extracted_freqs = []
    
    for index in indices:
        freqs_range = freq_bins[index[0]: index[1]]
        avg_freq = round(numpy.average(freqs_range))
        
        if avg_freq not in extracted_freqs:
            extracted_freqs.append(avg_freq)

    # group extracted frequency by nearby=freq_threshold (tolerate gaps=freq_threshold)
    group_similar_values = split(extracted_freqs, where(diff(extracted_freqs) > freq_threshold)[0]+1 )
    
    # calculate the average of similar value
    extracted_freqs = []
    for group in group_similar_values:
        extracted_freqs.append(round(numpy.average(group)))
    
    print("freq_components", extracted_freqs)
    return extracted_freqs
	
if __name__ == "__main__":

    # sample length and sample rate
    record_second = 3
    fs = 44100

    # do recording
    myrecording = sd.rec(int(record_second * fs), samplerate=fs, channels=2)

    # write if needed 
    #write('output.wav', fs, myrecording)

    # Load the wav file using soundfile
    #input_audio_file='output.wav'
    #audio_data, samplerate = sd.read(input_audio_file)
    audio_data, samplerate = myrecording, fs

    # Remove DC offset by subtracting the mean
    audio_data = audio_data - np.mean(audio_data)

    # Perform Fourier transform to get frequencies
    frequencies = np.fft.rfftfreq(len(audio_data), d=1/samplerate)
    spectrum = np.abs(np.fft.rfft(audio_data))

    # alarm if frequency count is high over 2000 Hx or under 100 Hz
    spectrum = remove_hum(frequencies, spectrum)
    f_avg, hf, lf = freq_over(frequencies, spectrum, 2000, 100, 1000.0)
    if hf > 1000:
        print("high freqs in sound")
    if lf > 1000:
        print("low freqs in sound")    
        
    # ----------- plot amplitude --------------
    # Generate time axis
    time = np.linspace(0, (len(audio_data) - 1)  / samplerate, num=len(audio_data))

    # Plot the audio data
    plt.figure(figsize=(12, 6))
    plt.plot(time, audio_data)
    plt.title('Time Series Audio Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)

    output_image_file = 'amp.png'
    # Save the plot as a PNG image
    plt.savefig(output_image_file)

    # Display the plot
    # plt.show()

    # ----------- plot freq --------------
    # Plot the spectrum
    plt.figure(figsize=(12, 6))
    plt.plot(frequencies, spectrum)
    plt.title('Spectrum')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.grid(True)

    output_image_file2 = 'spectrum.png'
    # Save the plot as a PNG image
    plt.savefig(output_image_file2)

    # Display the plot
    # plt.show()

    # ---------- plot intensity ------------
    freq = 1000
    omega = 2 * np.pi * freq
    rho_0 = 1.293
    kappa = 142.0e3
    c = np.sqrt(kappa/rho_0)
    k = omega / c
    row = 21
    col = 21
    width = 0.05
    y, x = audio_data, time
    disp = np.array([np.min(x), np.max(x), np.min(y), np.max(y)])

    # plane wave
    theta = np.pi/2
    k_vec = k*np.array([np.cos(theta), np.sin(theta)])
    I_p = k_vec / (2*omega*rho_0)
    P_p = np.exp(1j * (x*k_vec[0] + y*k_vec[1]))
    # drawing
    plot_intensity(I_p, P_p, x, y, disp)

    # ------------ flot frq / mag --------------------
    #import soundfile (another library you could use for opening test file)
    #audio_samples, sample_rate  = soundfile.read("to_do_test.wav", dtype='int16')
    audio_samples, sample_rate = myrecording, fs
    number_samples = len(audio_samples)
    #print('Audio Samples: ', audio_samples)
    #print('Number of Sample', number_samples)
    #print('Sample Rate: ', sample_rate)
    
    # duration of the audio file
    duration = round(number_samples/sample_rate, 2)
    #print('Audio Duration: {0}s'.format(duration))
    
    # list of possible frequencies bins
    freq_bins = arange(number_samples) * sample_rate/number_samples
    #print('Frequency Length: ', len(freq_bins))
    #print('Frequency bins: ', freq_bins)
    
    # FFT calculation this time using scipy
    fft_data = scipy.fft(audio_samples)
    #print('FFT Length: ', len(fft_data))
    #print('FFT data: ', fft_data)

    freq_bins = freq_bins[range(number_samples//2)]      
    normalization_data = fft_data/number_samples
    magnitude_values = normalization_data[range(len(fft_data)//2)]
    magnitude_values = numpy.abs(magnitude_values)
        
    indices = findPeak(magnitude_values=magnitude_values, noise_level=200)
    frequencies = extractFrequency(indices=indices)
    #print("frequencies:", frequencies)
    
    x_asis_data = freq_bins
    y_asis_data = magnitude_values
        
    plt.figure(figsize=(12, 6))
    plt.plot(x_asis_data, y_asis_data)
    plt.title('Audio Freq')
    plt.xlabel('Frequency Bins (Hz)')
    plt.ylabel('Magnitude - Voltage  Gain / Loss')
    plt.grid(True)

    output_image_file4 = 'freq_bins.png'
    # Save the plot as a PNG image
    plt.savefig(output_image_file4)

    # Display the plot
    # plt.show()        

