#!/usr/bin/env python3
# coding:utf-8
# This example shows speach recognition with one keigan motor attached
#
import speech_recognition
import pyttsx3
import datetime
import locale
from pykeigan import usbcontroller
from pykeigan import utils

# e.g. for japanese MY_LANG="ja_JP"
# german 'de_DE'
MY_LANG="en_EN"
locale.setlocale(locale.LC_TIME, '')

# set-up ear and mouth for robot
robot_ear = speech_recognition.Recognizer()
robot_mouth = pyttsx3.init()
robot_brain = ""
rpm=10.0
dist=2

# call this function to show available voices if you issue identifying them
def show_voice(engine):
    voices = engine.getProperty('voices')
    for v in voices:
        print(v)

# use this function (incomplete only does 1-10) if you need to convert a string to a number from the engine output
def text2Num(stri):
    if "one" in stri:
        n=1
    rlif "two" in stri:
        n=2
    elif "three" in stri:
        n=3
    elif "four" in stri:
        n=4
    elif "five" in stri:
        n=5
    elif "six" in stri:
        n=6
    elif "seveb" in stri:
        n=7
    elif "eight" in stri:
        n=8
    elif "nine" in stri:
        n=9
    elif "ten" in stri:
        n=10
    return n        
        
# example of voices from above command it differs per your os and what you have loaded 
# windows its like
# en_voice_id = "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_EN-US_ZIRA_11.0"
# ru_voice_id = "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_RU-RU_IRINA_11.0"
# macOs
# com.apple.speech.synthesis.voice.Alex
# com.apple.speech.synthesis.voice.alice
# com.apple.speech.synthesis.voice.alva
# com.apple.speech.synthesis.voice.amelie
# com.apple.speech.synthesis.voice.anna
# ubuntu
# voice[0].id = male
# voice[1].id = female
# extra voice can be found here https://github.com/numediart/MBROLA
# https://github.com/numediart/MBROLA-voices#mbrola-voices-database
# e.g. voice afrikaans-mbrola-1
#
# function to allow change of voice
def change_voice(engine, language, gender='male'):
    for voice in engine.getProperty('voices'):
        if language in voice.languages and gender == voice.gender:
            engine.setProperty('voice', voice.id)
            return True
    raise RuntimeError("Language '{}' for gender '{}' not found".format(language, gender))

# set voice to first voice found
def set_default_voice(engine, rt=130, v=1.0, pit=75):
    engine.setProperty('rate', rt)                                                 
    engine.setProperty('volume', v)
    engine.setProperty('pitch', pit)                                       # Set the pitch (default 50) to 75 out of 100
    voice = engine.getProperty('voices'):
    engine.setProperty('voice', voice[0].id) 

def set_named_voice(engine, rt=130, v=1.0, vid='us-mbrola-3', pit=50):
    engine.setProperty('rate', rt)                                                 
    engine.setProperty('volume', v)
    voice = engine.getProperty('voices'):
    engine.setProperty('voice', vid)
    engine.setProperty('pitch', pit)                                       # Set the pitch (default 50) to 75 out of 100

# function to listen and control the drive whilst speaking back
def listen_and_act(port_1="/dev/ttyUSB0"):
    global rpm
    global dist
    kg_drive1 = usbcontroller.USBController(port_1, False)
    kg_drive1.enable_check_sum(True) 
    while True:
        # listen and process speach
        with speech_recognition.Microphone() as mic:
            print("Robot: I'm listening")
            audio = robot_ear.record(mic, duration = 2)
        try:
            you = robot_ear.recognize_google(audio, language=MY_LANG)
        except:
            you = "..."
        # process response
        if "..." in you:
            robot_brain = "I can't hear you, please say it again."
        elif "Hello" in you:
            robot_brain = "Hello, Please say a command fdr the motor"
        elif "time" in you:
            robot_brain = datetime.datetime.now().strftime("%B %d, %Y")
        elif "enable" in you:
            robot_brain = "enabled drive"
            kg_drive1.enable_action()
        elif "disable" in you:
            robot_brain = "disabled drive"
            kg_drive1.disable_action()
        elif "stop" in you:
            robot_brain = "stop drive"
            kg_drive1.stop_motor()
        elif "mode distance" in you:
            robot_brain = "distance drive mode"
            if "forward" in you:
                kg_drive1.move_by_dist(dist)
            else:
                kg_drive1.move_by_dist(-dist)   
        elif "change distance" in you:
            p=you.split("to")
            try:
                dist = float(p[1])                                                            # might have to use a lookup table word to number please try e.g. dist = text2num(p[1])
                robot_brain = "changed distance setting"
            except:
                robot_brain = "can not understand the distance value you said.. example change distance to one hundred"                 
        elif "mode velocity" in you:
            robot_brain = "velocity drive mode"
            kg_drive1.run_at_velocity(utils.rpm2rad_per_sec(rpm))
        elif "change velocity" in you:
            p=you.split("to")
            try:
                rpm = float(p[1])                                                            # might have to use a lookup table word to number please try
                robot_brain = "velocity drive mode"
                kg_drive1.run_at_velocity(utils.rpm2rad_per_sec(rpm))
            except:
                robot_brain = "can not understand the rpm you said.. example change velocity to one hundred"                
        elif "turn" in you:
            angle = 0
            p=you.split("to")
            try:
                dirstr = str(p[1])
                if not dirstr.find("left") == -1:
                    d = dirstr.split("left")
                    angle = float(d[1])
                elif not dirstr.find("right") == -1: 
                    d = dirstr.split("right")
                    angle = float(d[1]) * -1  
                else:  
                    angle = float(dirstr)                
                robot_brain = "turn mode"
                kg_drive1.set_speed(utils.rpm2rad_per_sec(rpm))
                kg_drive1.move_by_dist(utils.deg2rad(angle))                    
            except:
                robot_brain = "can not understand the turn command you said... example turn to left 39"               
        elif "bye bye" in you:
            robot_brain = "goodbye closing the program"
            change_voice(robot_mouth, MY_LANG, "female")
            robot_mouth.say(robot_brain)
            robot_mouth.runAndWait()
            break
        else:
            robot_brain = "did not understand"

        #change_voice(robot_mouth, MY_LANG, "VoiceGenderFemale")
        robot_mouth.say(robot_brain)
        robot_mouth.runAndWait()

if __name__ == "__main__":
    set_named_voice()
    listen_and_act()

