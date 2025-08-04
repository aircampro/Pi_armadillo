# use WhisperX or pyanote instead of vosk for speach recognition
#
# pip install git+https://github.com/m-bain/whisperx.git
#
import whisperx
import torch
import pyaudio
import wave

# simple sound recorder 
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"

p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
print("* recording")

frames = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("* done recording")

stream.stop_stream()
stream.close()
p.terminate()

wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()

if torch.backends.mps.is_available():
    device = torch.device("mps")
else:
    device = torch.device("cpu")
	
#device = "cpu"                                                                          # M1 Max MPS
with torch.no_grad():
    model = whisperx.load_model("large-v2", device)

audio = whisperx.load_audio(WAVE_OUTPUT_FILENAME)
result = model.transcribe(audio, batch_size=16)

model_a, metadata = whisperx.load_align_model(language_code=result["language"], device=device)
result = whisperx.align(result["segments"], model_a, metadata, audio, device)

diarize_model = whisperx.DiarizationPipeline(use_auth_token="YOUR_HF_TOKEN",   device=device)
diarize_segments = diarize_model(audio)
result = whisperx.assign_word_speakers(diarize_segments, result)

for speaker in set(segment['speaker'] for segment in result['segments']):
    speaker_segments = [s for s in result['segments'] if s['speaker'] == speaker]

# use pynote for the SR
# pip install pyannote.audio
# conda install -c conda-forge hmmlearn libsndfile  # M1対応
#
from pyannote.audio import Pipeline

pipeline = Pipeline.from_pretrained("pyannote/speaker-diarization-3.1", use_auth_token="YOUR_HF_TOKEN")

diarization = pipeline(WAVE_OUTPUT_FILENAME)

for turn, _, speaker in diarization.itertracks(yield_label=True):
    print(f"{speaker}: {turn.start:.1f}s - {turn.end:.1f}s")