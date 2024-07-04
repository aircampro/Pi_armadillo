#!/usr/bin/python3
#
# get text from an image, speak the text and allow user interface to change the speed of the reading 
#
_SPEACH_ENG = "pytsx3"                                                          # speech engine is set to pytsx3
if _SPEACH_ENG == "pytsx3":
    #pip install pyttsx3
    #sudo apt install libespeak-dev
    import pyttsx3

    # initialise speach engine
    chose_voice=1                                                              # choose the 2nd voice
    engine = pyttsx3.init()
    voices = engine.getProperty('voices')
    engine.setProperty('voice',voices[chose_voice].id)
    print(voices[chose_voice].id)

    def tsx_speak(str):
        engine.setProperty('rate', 150)
        engine.say(str)
        engine.runAndWait()

    def tsx_speak_slower(str):
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate-50)
        engine.say(str)
        engine.runAndWait()

    def tsx_speak_faster(str):
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate+50)
        engine.say(str)
        engine.runAndWait()
        
    def tsx_speak_only(str):
        engine.say(str)
        engine.runAndWait()
        
    def save_speak(str):
        engine.save_to_file(str, 'spoken.mp3')	
	
    speak=tsx_speak
elif _SPEACH_ENG == "espeak":
    #sudo apt-get install espeak
    from subprocess import call
    def e_speak(str):
        call(["espeak",str])
        
    speak=e_speak        
elif _SPEACH_ENG == "pico":
    import os
    from subprocess import call
    file_path = os.path.abspath(__file__)

    # Define path
    speech_wave = file_path.replace('/text_reader.py', '/speech.wav')

    def pico_speak(content):
        call('amixer sset Master 90% -q --quiet', shell=True)                                  # big voice
        call(['pico2wave', '-w={}'.format(speech_wave), content])                              # write wav file
        call('aplay -q --quiet {}'.format(speech_wave), shell=True)                            # play wav file
        
    speak=pico_speak 

_LISTEN_ENG="julius"                                                                           # listen engine is set to julius
# for listening to speach we are using the julius engine
# follow instructions below to install items
#
# requires the julius speech engine it works under linux or windows
# for raspi refer https://www.pc-koubou.jp/magazine/19743
# sudo apt-get install build-essential zlib1g-dev libsdl2-dev libasound2-dev
# sudo apt-get install libasound2-dev libesd0-dev libsndfile1-dev
# sudo apt-get install alsa-utils sox libsox-fmt-all
# git clone https://github.com/julius-speech/julius
# cd julius
# ./configure --enable-words-int
# ./configure --with-mictype=alsa
# make -j4
# sudo make install
# git clone https://github.com/julius-speech/grammar-kit
# go to here https://sourceforge.net/projects/juliusmodels/files/
# load your model 
# e.g. ENVR-v5.4.Gmm.Ascii.zip
# GMM HMM Ascii English speech models (262K dictionary, 32 bit LM)
#
# ENVR-v5.4.Dnn.Bin.zip
# DNN English speech models (262K dictionary, 32 bit LM)
# To use these models you need a modified version of Julius from:
# https://github.com/palles77/julius
# unzip that file
# set mic.jconf as instructed
# edit dnn.jconf as instructed
#
if _LISTEN_ENG=="julius":
    import socket
    import time
    import re

    # julius installed locally
    host = '127.0.0.1'
    # Julius port
    port = 10500

    # Extract words recognized by regular expressions
    extracted_word = re.compile('WORD="([^"]+)"')
    data = ""

    listen_for_command():
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                    
            client.connect((host, port))                                          # connect to julius
            time.sleep(2)
            no_answer = 0
            while no_answer == 0:
                while (data.find("</RECOGOUT>\n.") == -1):
                    data += str(client.recv(1024).decode('utf-8'))

                recog_text = ""
                for word in filter(bool, extracted_word.findall(data)):
                    recog_text += word

                print("you spoke: " + recog_text)
                data = ""
                if recog_text.upper() == "YES" or recog_text.upper() == "SLOWER" or recog_text.upper() == "SLOW" or recog_text.upper() == "EYE":
                    no_answer = 1
                if recog_text.upper() == "NO" or recog_text.upper() == "NAR" :   
                    no_answer = 2                
            client.send("DIE".encode('utf-8'))
            client.close()
        except:
            print('PROCESS END')
            client.send("DIE".encode('utf-8'))
            client.close()
        return no_answer
 
# needs the following for image reading ocr
import numpy as np
import sys
import cv2
from scipy.signal import argrelmax
# we are using pyocr 
import pyocr
from PIL import Image
#from PIL import ImageDraw
#from PIL import ImageFont

# conda install -c conda-forge pdfminer.six
from pdfminer.high_level import extract_text

tools = pyocr.get_available_tools() 
tool = tools[0]

def pred_img(cv_img, layout):
    #img = Image.open(path)
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2) 
    gray = Image.fromarray(gray)
    img = Image.fromarray(cv_img)
    builder = pyocr.builders.TextBuilder(tesseract_layout=layout)
    text = tool.image_to_string(img, lang="en", builder=builder)
    return text

def calc_haarlike(crop_img, rect_h):
    pattern_h = rect_h // 2 
    height = crop_img.shape[0]
    out = np.zeros([height-rect_h])
    for index in range(height-rect_h):
        a1 = np.mean(crop_img[index: index+pattern_h, :])
        a2 = np.mean(crop_img[index+pattern_h: index+rect_h,:])
        out[index] = a1-a2 
    return out

def peak_detection(data, th):
    peak1 = argrelmax(data)[0]                                        # Find start of line
    peak2 = argrelmax(-data)[0]                                       # end of line detection
    peak1 = peak1[np.where(data[peak1] > th)]                         # Remove the peak of the response value less than or equal to constant
    peak2 = peak2[np.where(np.abs(data[peak2]) > th)]                 # Remove the peak of the response value less than or equal to a certain value 
    return peak1, peak2

def line_detection(peak1, peak2, rect_h, pad=5):
    lines = list()
    for p in peak1:
        bottom = np.min(peak2[peak2 > p]) +rect_h//2 + pad
        top = p + rect_h//2 - pad
        lines.append([p, bottom])
    return lines

if __name__ == "__main__":
    rect_h = 20                                                       # lines width
    pad = 5                                                           # spacing

    fil = sys.argv[1]                                                 # read the first argument as the filename.extension e.g. "pic_grab.jpg"
    if fil.split(".")[1] == "pdf":
        text = extract_text(fil)   
        phrases = text.split(".")
        first_line=0
        for sentance in phrases:                                      # for each line read from the pdf file passed
            speak(sentance)
            if first_line == 0:
                speak("am I speaking too fast")
                if (listen_for_command() == 1):                       # yes
                    if _SPEACH_ENG == "pytsx3": 
                        speak = tsx_speak_slower
                        speak(sentance)                               # repeat it slower
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed") 
                elif (listen_for_command() == 2):                     # no
                    speak("am I speaking too slow")
                    if (listen_for_command() == 1):                   # yes
                        if _SPEACH_ENG == "pytsx3": 
                            speak = tsx_speak_faster
                            speak(sentance)                           # repeat it slower
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed")                 
                first_line = 1  
            elif first_line > 0 and speak == tsx_speak_slower:         # either keep speaking slower or stay at that speed
                speak("am I speaking too fast")
                if (listen_for_command() == 2):
                    if _SPEACH_ENG == "pytsx3": 
                        speak = tsx_speak_only
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed") 
                else:
                    speak(sentance)                                    # repeat it slower
            elif first_line > 0 and speak == tsx_speak_faster:         # either keep speaking faster or stay at that speed
                speak("am I speaking too fast")
                if (listen_for_command() == 2):
                    if _SPEACH_ENG == "pytsx3": 
                        speak = tsx_speak_only
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed") 
                else:
                    speak(sentance)                                    # repeat it faster
    else:        
        img = cv2.imread(fil)                                           # pass the text to speak as a jpg or png file (snapshot from camera)
        out = calc_haarlike(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), rect_h)
        peak1, peak2 = peak_detection(out, 15)
        lines = line_detection(peak1, peak2, rect_h)

        # read the lines from the image
        first_line=0
        for l in lines:                                                  # for each line read from the image
            out = img[l[0]: l[1]]
            text = pred_img(out, 7)
            print(text)
            speak(text)
            if first_line == 0:
                speak("am I speaking too fast")
                if (listen_for_command() == 1):
                    if _SPEACH_ENG == "pytsx3": 
                        speak = tsx_speak_slower
                        speak(text)                               # repeat it slower
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed") 
                elif (listen_for_command() == 2):                     # no
                    speak("am I speaking too slow")
                    if (listen_for_command() == 1):                   # yes
                        if _SPEACH_ENG == "pytsx3": 
                            speak = tsx_speak_faster
                            speak(text)                           # repeat it slower
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed")  
                first_line = 1  
            elif first_line > 0 and speak == tsx_speak_slower:
                speak("am I speaking too fast")
                if (listen_for_command() == 2):
                    if _SPEACH_ENG == "pytsx3": 
                        speak = tsx_speak_only
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed") 
                else:
                    speak(text)                                    # repeat it slower     
            elif first_line > 0 and speak == tsx_speak_faster:         # either keep speaking faster or stay at that speed
                speak("am I speaking too fast")
                if (listen_for_command() == 2):
                    if _SPEACH_ENG == "pytsx3": 
                        speak = tsx_speak_only
                    else:
                        speak("you have to select the pytsx engine to change my speaking speed") 
                else:
                    speak(text)                                    # repeat it faster                    