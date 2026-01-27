#
# TTS Example using kokoro from Hugging Face
#
import torch
import soundfile as sf
from models import build_model
from kokoimport torch
import soundfile as sf
from models import build_model
from kokoro import generate
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('--txt1', type=str, default='first sentance')
parser.add_argument('--txt2', type=str, default='second sentance')
parser.add_argument('--voice_id', type=int, default=1, help='select voice 0 to 10')
parser.add_argument('--us_gb', type=str, default='gb')
parser.add_argument('--outfile', type=str, default='output_audio.wav',  help='audio output file name')
						 
device = 'cuda' if torch.cuda.is_available() else 'cpu'
MODEL = build_model('kokoro-v0_19.pth', device)

VOICE_NAME = [
    'af',                                 # (Bella & Sarah 50-50
    'af_bella', 'af_sarah', 'am_adam', 'am_michael',
    'bf_emma', 'bf_isabella', 'bm_george', 'bm_lewis',
    'af_nicole', 'af_sky',
]

VOICEPACK = torch.load(f'voices/{VOICE_NAME[args.voice_id % 11]}.pt', weights_only=True).to(device)
print(f'Loaded voice: {VOICE_NAME[args.voice_id % 11]}')
text = args.txt1 + "," + args.txt2

# 🇺🇸 'a' => American English => en-us
# 🇬🇧 'b' => British English => en-gb
if args.us_gb == "us":
    l="a"
elif args.us_gb == "gb":
    l="b"
else:
    print("unsupported option using us")
    l="a"

audio, out_ps = generate(MODEL, text, VOICEPACK, lang=l)
output_audio_path = args.outfile
sf.write(output_audio_path, audio, 24000)
print("Audio saved as 'output_audio.wav'")
if os.name == 'nt':  # Windows
    os.system(f'start {output_audio_path}')
elif os.uname().sysname == 'Darwin':  # macOS
    os.system(f'afplay {output_audio_path}')
else:  # Linux
    os.system(f'play {output_audio_path}')

print("Phonemes used in the generation:")
print(out_ps)
print(text)