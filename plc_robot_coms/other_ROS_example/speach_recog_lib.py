# A library for handling the vosk speach to text service
# you can call the methods directly or you can run it as a ros2 service
#
import json
import math
import queue
import struct
import sys
from collections import namedtuple
from typing import NamedTuple
import numpy as np
import sounddevice as sd
# SR Engine == VOSK e.g. pip install vosk
from vosk import KaldiRecognizer, Model, SetLogLevel                                      
import subprocess

# you need these if you wan to run this as a service rather than just a library
import rclpy
from rclpy.node import Node
from chatrover_msgs.srv import TextText

class VadConfig(NamedTuple):
    """Class for configuring speech activity detection.
    threshold (int): Power threshold (dB) for determining speech activity detection.
    vad_start (float): vad_start (float): Number of seconds (sec) for determining the start of a speech activity.
    vad_end (float): vad_end (float): Number of seconds (sec) for determining the end of a speech activity.
    """
    threshold: int = 45
    vad_start: float = 0.3
    vad_end: float = 3.0

class MicrophoneStream:
    """Class for microphone audio input."""

    def __init__(self, rate, chunk, vad_config):
        """set sample rate and chunk size and recording system v.a.d.
        Args:
           rate (int): Sampling rate (Hz)
           chunk (int): Unit of audio data received (number of samples)
           vad_config (VadConfig): Voice activity detection settings
        """
        self.rate = rate
        self.chunk = chunk

        # create queue
        self.buff = queue.Queue()

        # set vad
        self.vad_config = {
            "threshold": vad_config.threshold,
            "vad_start": vad_config.vad_start,
            "vad_end": vad_config.vad_end,
        }

        self.workspace = {
            "is_speaking": False,                               # Is the current speech segment recognized?
            "count_on": 0,                                      # The current number of consecutive segments above the threshold.
            "count_off": 0,                                     # The current number of consecutive segments below the threshold.
            "voice_end": False,                                 # Has speech ended?
            "str_current_power": "",                            # A string to check the current power value (referenced by the speech recognition class).
        }

        # Initializing microphone audio input
        self.input_stream = None

    def open_stream(self):
        """open raw input stream"""
        self.input_stream = sd.RawInputStream(
            samplerate=self.rate,
            blocksize=self.chunk,
            dtype="int16",
            channels=1,
            callback=self.callback,
        )

    def callback(self, indata, frames, time, status):
        """callback function.
        """
        if status:
            print(status, file=sys.stderr)

        # Save the input audio data to the queue
        self.buff.put(bytes(indata))

        # Calculate the power of the audio (the root mean square of the audio data)
        indata2 = struct.unpack(f"{len(indata) / 2:.0f}h", indata)
        rms = math.sqrt(np.square(indata2).mean())
        power = 20 * math.log10(rms) if rms > 0.0 else -math.inf  # RMS

        self.workspace["str_current_power"] = f"voice power {power:5.1f}[dB] "
        #print("\r" + self.workspace["str_current_power"], end="", flush=True)

        # When the audio power is above the threshold and the speech segment has not yet been recognized
        if (
            power >= self.vad_config["threshold"]
            and self.workspace["is_speaking"] is False
        ):

            # Increase the counter for sections above the thres
            self.workspace["count_on"] += 1

            # Convert the length of the section above the threshold into seconds
            count_on_sec = float(self.workspace["count_on"] * self.chunk) / self.rate

            # Comparison with threshold to identify the start of speech interv
            if count_on_sec >= self.vad_config["vad_start"]:
                self.workspace["is_speaking"] = True
                self.workspace["count_on"] = 0

        if power < self.vad_config["threshold"] and self.workspace["is_speaking"]:

            # Increment the counter for the section below the threshold
            self.workspace["count_off"] += 1

            # Convert the length of the section below the threshold into seconds
            count_off_sec = float(self.workspace["count_off"] * self.chunk) / self.rate

            # Compare with a threshold to determine the end of the speech section
            if count_off_sec >= self.vad_config["vad_end"]:
                self.workspace["voice_end"] = True
                self.workspace["count_off"] = 0

            # Compare with threshold and reset counter for opposite condition
            if power >= self.vad_config["threshold"]:
                self.workspace["count_off"] = 0
            else:
                self.workspace["count_on"] = 0

    def generator(self):
        """Functions for obtaining audio data needed for speech recognition"""
        while True:  
            # Get first data
            chunk = self.buff.get()
            if chunk is None:
                return
            data = [chunk]

            # If there is still data remaining in the queue, get it all
            while True:
                try:
                    chunk = self.buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            # By using yield, you can obtain queue data at any time
            yield b"".join(data)

def get_asr_result(vosk_asr):
    """Executes the speech recognition API to obtain the final recognition result.
    Args:
       vosk_asr (VoskStreamingASR): Speech recognition module
    Returns:
       recog_text (str): Speech recognition result
    """
    mic_stream = vosk_asr.microphone_stream
    mic_stream.open_stream()
    with mic_stream.input_stream:
        audio_generator = mic_stream.generator()
        for content in audio_generator:
            if vosk_asr.recognizer.AcceptWaveform(content):
                recog_result = json.loads(vosk_asr.recognizer.Result())
                recog_text = recog_result["text"].split()
                recog_text = "".join(recog_text)  
                if len(recog_text)>4:
                    return recog_text
        return None

# selecting the voice model for vosk and your native language and cpu capabilities small for raspi and android else large 
# https://huggingface.co/Derur/vosk-stt-models/tree/main
# https://alphacephei.com/vosk/models
# vosk-model-small-en-gb-0.15
# vosk-mode-large-en-us
# vosk-model-en-us-0.22
# vosk-model-en-in-0.5
# vosk-model-small-en-us-0.15
# vosk-model-small-en-in-0.4
# for japan vosk-model-ja-0.22
# https://huggingface.co/rhasspy/vosk-models/tree/main/ja
#

# recording volume adjustment dor the usb microphone
#
# amixer sset Mic 16 -c 1
#

def set_max_rec_vol(vol=16):
    cmd_arg = f"sset Mic {vol} -c 1"
    cp = subprocess.run(['amixer', cmd_arg], encoding='utf-8', stdout=subprocess.PIPE)
    print(f'recording volume set to {vol}：' + cp.stdout )
    
def translate_speach(chunk_size=8000, threshold=40, vad_start=0.3, vad_end=3.0):
    """record sound stream and translate to text using vosk
    Args:
    chunk_size (int): Unit of audio data received (number of samples)
    threshold (int): Power threshold (dB) for determining speech activity detection
    vad_start (float): Number of seconds (sec) for determining the start of a speech activity
    vad_end (float): Number of seconds (sec) for determining the end of a speech activity
    """
    SetLogLevel(-1)  # Suppress log display when VOSK starts
    # Obtain sampling frequency information based on input device information
    input_device_info = sd.query_devices(kind="input")
    sample_rate = int(input_device_info["default_samplerate"])
    # Configure voice activity detection
    vad_config = VadConfig(threshold, vad_start, vad_end)
    # Initialize and start microphone input
    mic_stream = MicrophoneStream(sample_rate, chunk_size, vad_config)
    # Build a speech recognizer using the language model downloaded from hugging face as above or other
    recognizer = KaldiRecognizer(Model(r"/home/humble/ros2_ws/src/voice_gpt/chat_rover/model/vosk-model-en-in-0.5"), sample_rate)
    # Store the microphone input stream and speech recognizer together
    VoskStreamingASR = namedtuple("VoskStreamingASR", ["microphone_stream", "recognizer"])
    vosk_asr = VoskStreamingASR(mic_stream, recognizer)
    
    print("＜Recognition Start＞", flush=True)
    recog_result = get_asr_result(vosk_asr)
    print(recog_result, flush=True)
    print("＜Recognition completed＞", flush=True)
    return recog_result

class VoiceTextSrvc(Node):
    def __init__(self):
        super().__init__('vosk_node')
        self.server = self.create_service(TextText, "/get_voice", self.get_voice_cb)
    def get_voice_cb(self, request, response):
        response.text = translate_speach()
        self.get_logger().info('Publishing: "%s"' % response.text)
        return response

# run this if you want a rts2 service      
def main_ros2_service(args=None):
    rclpy.init(args=args)
    voicetext_publisher = VoiceTextSrvc()
    rclpy.spin(voicetext_publisher)
    voicetext_publisher.destroy_node()
    rclpy.shutdown()

ROS_SERVICE_ON=False                                            # use as a library not service
if __name__ == '__main__':
    if ROS_SERVICE_ON == True:
        main_ros2_service()
	
