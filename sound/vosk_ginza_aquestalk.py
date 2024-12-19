#
# Example of how to use the vosk speach engine, Ginza NLP pipeline and then speak the results
# pip install vosk
#
# run.py
#
import json
import multiprocessing as mp
import numpy as np
import vosk
import sounddevice as sd
import os
import sys
import subprocess
import shutil
import spacy
import copy
import soundcard as sc

# check the platform and choose the relevant speach engine
#
import platform
p=platform.platform().split("-")
# example on raspi 'Linux-5.4.51-v7l+-armv7l-with-glibc2.28'
if p[3] == "armv7l" :
    plat = "raspi"
    TOOL_PATH = "/home/pi/aquestalkpi/AquesTalkPi"
else:
    pf = platform.system()
    if pf == 'Windows':
        plat = "win"
        TOOL_PATH = "AquesTalkPlayer.exe"
    elif pf == 'Darwin':
        print('Speach AquesTalk not supported on Mac')             # https://github.com/taku-o/myukkurivoice/releases or https://github.com/PickledChair/YukkuLips/releases
        sys.exit(-1)
    elif pt == 'Linux':
        plat = "lin"
    else:
        print('Speach AquesTalk not supported on ', pf)
        sys.exit(-1)    
        
# determine language en or jp and choose the nlp dictionary
#
if plat = "win":
    output_str = subprocess.run('Get-InstalledLanguage', capture_output=True, text=True).stdout
    if not output_str.find("ja-JA") == -1:
        LANG="jp"
    else:
        LANG="en"
else:   
    cmd_str = "locale -a | grep jp"
    z=subprocess.run(cmd_str, shell=True)
    if z.returncode == 0:
        LANG="jp"
    else:
        LANG="en"   

if plat == "lin":    
    from ctypes import *
    # this function calls directly the shared object library and is used in other linux than raspberry pi
    #
    def synthe_spk(text, speed=100, file_phont=None, engine=0):
        if file_phont is not None:
            with open(file_phont, 'rb') as f:
                phont = f.read()
        else:
            phont = None

        aqtk = cdll.LoadLibrary("libAquesTalk2Eva.so")
        engines = [ aqtk.AquesTalk2_Synthe_Utf8, aqtk.AquesTalk2_Synthe, aqtk.AquesTalk2_Synthe_Roman, aqtk.AquesTalk2_Synthe_Euc]  
        engines[engine].restype = POINTER(ARRAY(c_ubyte, 0))    
        #aqtk.AquesTalk2_Synthe_Utf8.restype = POINTER(ARRAY(c_ubyte, 0))
        size=c_int(0)
        wav_p = engines[engine](text.encode('utf-8'), speed, byref(size), phont)
        #wav_p = aqtk.AquesTalk2_Synthe_Utf8(text.encode('utf-8'), speed, byref(size), phont)
        if not bool(wav_p):
            print("ERR:", size.value)
            return None
        wav_p = cast(wav_p, POINTER(ARRAY(c_ubyte, size.value)))
        wav = bytearray(wav_p.contents)
        aqtk.AquesTalk2_FreeWave(wav_p)
        return wav

    def say_to_file(textmsg: str, outfile: str) -> None:
        with open(outfile, 'wb') as f:
            wav = synthe_spk(textmsg, speed=100)
            if not wav == None:
                f.write(wav)
        
    def say_to_file_with_phont(textmsg: str, f_p='aqtk2-lnx-eva/phont/aq_yukkuri.phont') -> None:        
        with open('yukkuri.wav', 'wb') as f:
            wav = synthe_spk(textmsg, speed=100, file_phont=f_p)
            if not wav == None:
                f.write(wav)

    def linux_speak(txt_msg: str, outfile: str) -> None:
        say_to_file(txt_msg, outfile)
        completedProcess = subprocess.run(['aplay', outfile])
        
# choose nlp engine
#
if LANG == "jp":
    nlp = spacy.load('ja_ginza')
elif LANG == "en":
    # python -m spacy download en_core_web_sm
    nlp = spacy.load("en_core_web_sm")

#import en_core_web_sm
#nlp = en_core_web_sm.load()

# captures the audio output
#
def capture_audio_output(audio_queue: mp.Queue, capture_sec: float, sample_rate: int) -> None: 
    
    num_frame: int = int(sample_rate * capture_sec)   
    while True:
        audio = sc.get_microphone(include_loopback=True, id=str(sc.default_speaker().name)) \
            .record(numframes=num_frame, samplerate=sample_rate, blocksize=sample_rate)
        audio_queue.put(copy.copy(audio[:, 0])) 
        
# checks with the list the closest response
#
def check_with_ginza_nlp(txt: str) -> None:
    msgs = []
    doc=nlp(txt) 
    with open('ginza_voice.txt',encoding='utf-8_sig') as m: 
        r=m.read().split('\n')  
    float_list=[] 
    for i in range(len(r)):
        k = nlp(r[i]) 
        float_list.append(doc.similarity(k)) 
        i=i+1 
        j=float_list.index(max(float_list)) 
    if max(float_list)<0.7: 
        txt='message not found' 
        msgs.insert(txt)     
    if max(float_list)>0.7: 
        txt=r[j]
        msgs.insert(txt)
    return msgs

# converts speach to text
#    
def speech_to_text(audio_queue: mp.Queue, sample_rate: int, outd : str) -> None: 
    NO_LOG: int = -1 
    if LANG == "jp":  
        MODEL_PATH = "vosk-model-ja-0.22"
    else:        
        MODEL_PATH = "vosk_model"                 # or "model" 
    
    vosk.SetLogLevel(NO_LOG)
    
    # model: vosk.Model = vosk.Model(model_name="vosk-model-ja-0.22")
    model: vosk.Model = vosk.Model(model_path=MODEL_PATH)
    recognizer = vosk.KaldiRecognizer(model, sample_rate)
    
    print("VOSK Speech recognizer is ready")
    print("Output sound from a speaker or a headphone")
    print("#" * 40)
    idx = 0
	preset = "れいむ"
    
    while True:
        audio = audio_queue.get()
        audio = map(lambda x: (x+1)/2, audio)
        audio = np.fromiter(audio, np.float16)
        audio = audio.tobytes()
        
        if recognizer.AcceptWaveform(audio):                         # call the recognizer with the audio
            recognizer.SetWords(True)
            result: json = json.loads(recognizer.Result())
            text = result["text"].replace(" ", "")
            
            if text != "":
                print(text)
                output = "\"%s/%02d_whatIgot.wav\""%(outd, idx)      
                idx += 1
                if plat == "win":
		            cmd = ["start", TOOL_PATH, "/T", "\""+text+"\"", "/P", preset, "/W", output]
		            subprocess.run(" ".join(cmd), shell=True) 
                elif plat == "raspi":
                    cmd = TOOL_PATH+" -s 130 -b \""+ text +"\" |aplay -q"
		            subprocess.run(cmd, shell=True)  
                    cmd = TOOL_PATH+" -s 130 -b \""+ text +"\" > "+output
		            subprocess.run(cmd, shell=True)                     
                elif plat == "lin":
                    linux_speak(text, output) 
                # now use the nlp engine GINZA                    
                marray = check_with_ginza_nlp(text) 
                for ii, w in enumerate(marray):
                    output = "\"%s/%02d_whatIgot_%02d.wav\""%(outd, idx, ii) 
                    if plat == "win":
		                cmd = ["start", TOOL_PATH, "/T", "\""+w+"\"", "/W", output]
		                subprocess.run(" ".join(cmd), shell=True)   
                    elif plat == "raspi":                    
                        cmd = TOOL_PATH+" -s 130 -b \""+ w +"\" |aplay -q"
		                subprocess.run(cmd, shell=True) 
                        cmd = TOOL_PATH+" -s 130 -b \""+ w +"\" > "+output
		                subprocess.run(cmd, shell=True)                        
                    elif plat == "lin":
                        linux_speak(w, output)                        
def main():
    CAPTURE_SEC: float = 4.0
	outdir = "output"
	if os.path.exists(outdir):
		shutil.rmtree(outdir)
	os.makedirs(outdir)   
    audio_queue: mp.Queue = mp.Queue() 
    sample_rate: int = int(sd.query_devices(kind="output")["default_samplerate"]) 
    stt_proc: mp.Process = mp.Process(target=speech_to_text, args=(audio_queue, sample_rate, outdir)) 
    
    print("Type Ctrl+C to stop")
    
    stt_proc.start()
    
    try:
        capture_audio_output(audio_queue=audio_queue, capture_sec=CAPTURE_SEC, sample_rate=sample_rate)
        stt_proc.join()
    except KeyboardInterrupt:
        stt_proc.terminate()
        
        print("\nDone")
