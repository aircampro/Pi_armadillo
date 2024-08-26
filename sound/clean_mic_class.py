# this is a class which provides a mic/input stream with noise reduction and band pass filter
#
import pyaudio
import numpy as np
from pydub import AudioSegment
import io
from pydub.effects import low_pass_filter, high_pass_filter
from scipy.fftpack import fft, ifft
from scipy.signal import resample
import threading

class CleanMicSoundStream(threading.Thread):
    def __init__(self, low_pass_lim=1000, high_pass_lim=300):
        threading.Thread.__init__(self)
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paFloat32
        self.CHANNELS = 1
        self.RATE = 44100
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.FORMAT,channels=self.CHANNELS, rate=self.RATE, input=True, output=True, frames_per_buffer=self.CHUNK, stream_callback=self.callback)
        self.noise_profile = None 
        self.audio_data = np.zeros(self.CHUNK)     
        # self.processing_thread = threading.Thread(target=self.audio_processing_thread)        
        self.lp_lim = low_pass_lim
        self.hp_lim = high_pass_lim
        
    def __del__(self):
        self.stream.stop_stream()
        self.stream.close()        
        self.p.terminate()
        # self.processing_thread.join()
        
    def run(self):
        # self.processing_thread = threading.Thread(target=self.audio_processing_thread) 
        # self.processing_thread.start()
        self.audio_processing_thread()
        
    def audio_processing_thread(self):
        self.stream.start_stream()
        while self.stream.is_active():
            pass
        self.stream.stop_stream()
        self.stream.close()        
        self.p.terminate()
        # self.processing_thread.join()
        
    def apply_effects(self, audio):
        audio = low_pass_filter(audio, self.lp_lim)
        audio = high_pass_filter(audio, self.hp_lim)
        return audio

    def noise_reduction(self, audio_data):
        if self.noise_profile is None:
            self.noise_profile = np.abs(fft(audio_data))
            return audio_data    
        spectrum = fft(audio_data)
        noise_reduction_factor = 0.9
        reduced_spectrum = spectrum - noise_reduction_factor * self.noise_profile
        reduced_spectrum = np.maximum(reduced_spectrum, 0)    
        return np.real(ifft(reduced_spectrum))

    def pitch_shift(self, audio_data, semitones):
        factor = 2 ** (semitones / 12)
        stretched = resample(audio_data, int(len(audio_data) / factor))
        return resample(stretched, len(audio_data))

    def process_audio(self, audio_data):
        # createw audio segment data to be effected
        audio = AudioSegment( audio_data.tobytes(), frame_rate=self.RATE, sample_width=audio_data.dtype.itemsize, channels=self.CHANNELS )
        processed_audio = self.apply_effects(audio)
        processed_audio = self.noise_reduction(np.array(processed_audio.get_array_of_samples()).astype(np.float32))
        processed_audio = self.pitch_shift(processed_audio, semitones=2)
    
        return processed_audio

    def callback(self, in_data, frame_count, time_info, status):
        self.audio_data = np.frombuffer(in_data, dtype=np.float32)
        processed_data = self.process_audio(audio_data)
        return (processed_data.astype(np.float32).tobytes(), pyaudio.paContinue)
