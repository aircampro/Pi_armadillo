#!/usr/bin/env python3
# example of use of the speech recognition library here :- https://github.com/Uberi/speech_recognition/tree/master/examples
#  pip install speechrecognition
#  pip install pyaudio
#  pip install pyttsx3
#
import speech_recognition as sr
import pyttsx3
import datetime
import locale
import os
import sys
# ================== functions for calling vatious speach engines =================================
# recognize speech using Sphinx
def use_sphinx(r,audio,lang=0):
    try:
        ans = r.recognize_sphinx(audio)
        print("Sphinx thinks you said " + ans)
    except sr.UnknownValueError:
        print("Sphinx could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Sphinx error; {0}".format(e))
        ans = "Sphinx error; {0}".format(e)
    return ans
# recognize speech using Google Speech Recognition
# lang can be changed e.g. lang='ja_JP'
GSR_KEY="GOOGLE_SPEECH_RECOGNITION_API_KEY"
def use_google_sr(r,audio,lang=0):
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key=GSR_KEY)`
        # instead of `r.recognize_google(audio)`
        if lang == 0:
            ans = r.recognize_google(audio)
        else:
            ans = r.recognize_google(audio, language=lang)        
        print("Google Speech Recognition thinks you said " + ans)
    except sr.UnknownValueError: 
        print("Google Speech Recognition could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        ans = "Google Speech Recognition service error; {0}".format(e)
    return ans
# recognize speech using Google Cloud Speech
# Before run, create local authentication credentials (``gcloud auth application-default login``)
def use_google_cl(r,audio,lang=0):
    try:
        if lang == 0:
            ans = r.recognize_google_cloud(audio)
        else:
            ans = r.recognize_google_cloud(audio, language=lang)        
        print("Google Cloud Speech thinks you said " + ans)
    except sr.UnknownValueError:
        print("Google Cloud Speech could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from Google Cloud Speech service; {0}".format(e))
        ans = "Cloud Speech service; error; {0}".format(e)
    return ans
# recognize speech using Wit.ai
WIT_AI_KEY = "INSERT WIT.AI API KEY HERE"  # Wit.ai keys are 32-character uppercase alphanumeric strings
def use_wit(r,audio,lang=0):
    try:
        if lang == 0:
            ans = r.recognize_wit(audio, key=WIT_AI_KEY)
        else:
            ans = r.recognize_wit(audio, key=WIT_AI_KEY, language=lang)        
        print("Wit.ai thinks you said " + ans)
    except sr.UnknownValueError:
        print("Wit.ai could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from Wit.ai service; {0}".format(e))
        ans = "wit error; {0}".format(e)
    return ans
# recognize speech using Microsoft Azure Speech
AZURE_SPEECH_KEY = "INSERT AZURE SPEECH API KEY HERE"  # Microsoft Speech API keys 32-character lowercase hexadecimal strings
def use_azure(r,audio,lang=0):
    try:
        if lang == 0:
            ans = r.recognize_azure(audio, key=AZURE_SPEECH_KEY)
        else:
            ans = r.recognize_azure(audio, key=AZURE_SPEECH_KEY, language=lang)        
        print("Microsoft Azure Speech thinks you said " + ans)
    except sr.UnknownValueError:
        print("Microsoft Azure Speech could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from Microsoft Azure Speech service; {0}".format(e))
        ans = "Microsoft Azure Speech service error; {0}".format(e)
    return ans
# recognize speech using Microsoft Bing Voice Recognition
BING_KEY = "INSERT BING API KEY HERE"  # Microsoft Bing Voice Recognition API keys 32-character lowercase hexadecimal strings
def use_bing(r,audio,lang=0):
    try:
        if lang == 0:
            ans = r.recognize_bing(audio, key=BING_KEY)
        else:
            ans = r.recognize_bing(audio, key=BING_KEY, language=lang)       
        print("Microsoft Bing Voice Recognition thinks you said " + r.recognize_bing(audio, key=BING_KEY))
    except sr.UnknownValueError:
        print("Microsoft Bing Voice Recognition could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from Microsoft Bing Voice Recognition service; {0}".format(e))
        ans = "Microsoft Bing Voice Recognition service error; {0}".format(e)
    return ans
# recognize speech using Houndify
HOUNDIFY_CLIENT_ID = "INSERT HOUNDIFY CLIENT ID HERE"  # Houndify client IDs are Base64-encoded strings
HOUNDIFY_CLIENT_KEY = "INSERT HOUNDIFY CLIENT KEY HERE"  # Houndify client keys are Base64-encoded strings
def use_houndify(r,audio,lang=0):
    try:
        if lang == 0:
            ans = r.recognize_houndify(audio, client_id=HOUNDIFY_CLIENT_ID, client_key=HOUNDIFY_CLIENT_KEY)
        else:
            ans = r.recognize_houndify(audio, client_id=HOUNDIFY_CLIENT_ID, client_key=HOUNDIFY_CLIENT_KEY, language=lang)        
        print("Houndify thinks you said " + ans)
    except sr.UnknownValueError:
        print("Houndify could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from Houndify service; {0}".format(e))
        ans = "Houndify Voice Recognition service error; {0}".format(e)
    return ans
# recognize speech using IBM Speech to Text
IBM_USERNAME = "INSERT IBM SPEECH TO TEXT USERNAME HERE"  # IBM Speech to Text usernames are strings of the form XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX
IBM_PASSWORD = "INSERT IBM SPEECH TO TEXT PASSWORD HERE"  # IBM Speech to Text passwords are mixed-case alphanumeric strings
def use_ibm(r,audio,lang=0):
    try:
        if lang == 0:
            ans = r.recognize_ibm(audio, username=IBM_USERNAME, password=IBM_PASSWORD)
        else:
            ans = r.recognize_ibm(audio, username=IBM_USERNAME, password=IBM_PASSWORD, language=lang)        
        print("IBM Speech to Text thinks you said " + ans)
    except sr.UnknownValueError:
        print("IBM Speech to Text could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print("Could not request results from IBM Speech to Text service; {0}".format(e))
        ans = "IBM Recognition service error; {0}".format(e)
    return ans	
# recognize speech using whisper
def use_whisper(r,audio,lang="english"):
    try:
        ans = r.recognize_whisper(audio, language=lang)
        print("Whisper thinks you said " + ans)
    except sr.UnknownValueError:
        print("Whisper could not understand audio")
        ans = "unknown"
    except sr.RequestError as e:
        print(f"Could not request results from Whisper; {e}")
        ans = "whisper error; {0}".format(e)
    return ans
# recognize speech using Whisper API
OPENAI_API_KEY = "INSERT OPENAI API KEY HERE"
def use_whisper_ai(r,audio,lang=0):
    os.environ["OPENAI_API_KEY"] = OPENAI_API_KEY
    try:
        ans = r.recognize_openai(audio)
        print(f"OpenAI Whisper API thinks you said {ans}")
    except sr.RequestError as e:
        print(f"Could not request results from OpenAI Whisper API; {e}")
        ans = "whisper ai; {0}".format(e)
    return ans  
# change voice of the mouth
def change_voice(engine, language, gender='VoiceGenderFemale'):
    for voice in engine.getProperty('voices'):
        if language in voice.languages and gender == voice.gender:
            engine.setProperty('voice', voice.id)
            return True
    raise RuntimeError("Language '{}' for gender '{}' not found".format(language, gender))

# list the engines available to us
sr_engines = [ use_google_sr, use_sphinx, use_google_cl, use_wit, use_azure, use_bing, use_houndify, use_ibm, use_whisper, use_whisper_ai]

# run the speach recognition
# opt chooses listen, record for +ve seconds, or use a file recorded.wav
# lang chooses the language
# engine chooses the speach recognition engine to use
#
def run_sr(robot_ear, robot_mouth, opt=0, lang=0, engine=0):

    if lang == 0:
        locale.setlocale(locale.LC_TIME, 'en_EN')
    elif lang == 1:
        locale.setlocale(locale.LC_TIME, 'ja_JP')  
    else:
        print("unsupported language")
        sys.exit(-1)        
    while True:
        if opt < 0:
            AUDIO_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), "recorded.wav")
            audio = sr.AudioData.from_file(AUDIO_FILE)
        else:
            with sr.Microphone() as mic:
                print("say request")
                robot_mouth.say("say request")
                robot_mouth.runAndWait()
                if opt == 0:
                    audio = robot_ear.listen(mic)
                else:
                    audio = robot_ear.record(mic, duration = opt)         
        try:
            if lang == 0:
                you = sr_engines[engine](robot_ear,audio)
            elif lang == 1:
                you = sr_engines[engine](robot_ear,audio, language='ja_JP')
        except:
            you = "..."

        if "..." in you:
            robot_brain = "I can't hear you, please say it again"
        elif "drone" in you:
            robot_brain = "drone"
        elif "robot" in you:
            robot_brain = "robot"
        elif "time" in you:
            robot_brain = datetime.datetime.now().strftime("%B %d, %Y")
        elif "color" in you:
            robot_brain = "blue"
        elif "exiting" in you:
            robot_brain = "closing speach application"
            print("exiting : " + robot_brain)
            change_voice(robot_mouth, "ja_JP", "VoiceGenderFemale")
            robot_mouth.say(robot_brain)
            robot_mouth.runAndWait()
            break
        else:
            robot_brain = "not in my brain"

        print("we answer: " + robot_brain)
        change_voice(robot_mouth, "ja_JP", "VoiceGenderFemale")
        robot_mouth.say(robot_brain)
        robot_mouth.runAndWait()

if __name__ == "__main__":
    # define the ears and the mouth
    robot_ear = sr.Recognizer()
    robot_mouth = pyttsx3.init()
    # do the speach recognition and talking
    run_sr(robot_ear, robot_mouth) 