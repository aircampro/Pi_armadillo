# !/usr/local/bin/python3
# -*- coding: utf-8 -*-
# raspberry pi camera to create a movie.py
#
import picamera
import time
import sys
import RPi.GPIO as GPIO

FILEPATH = '/data/'
USE_TIME=True                                            # when set False also use trigger button to stop recording 
MOVIE_INTERVAL = 600                                     # video recording length
GPIO_BCM_START = 18                                      # gpio pin for the start button to trigger the video capture

# control the camera recording from the gpio input pin
#
def main(now,w=1024,h=768,b=70):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_BCM_START, GPIO.IN)
    filename = FILEPATH + now + ".h264"
    while GPIO.input(GPIO_BCM_START) == 0:                # wait for the trigger button to be pressed
    with picamera.PiCamera() as camera:
        camera.hflip = True
        camera.vflip = True
        camera.resolution = (w,h)
        camera.brightness = b
        camera.start_recording(filename)
        if USE_TIME == True:
            time.sleep(MOVIE_INTERVAL)
        else:
            while GPIO.input(GPIO_BCM_START) == 0:        # wait for the trigger button to be pressed		
        camera.stop_recording()
    GPIO.cleanup(GPIO_BCM_START)
	
if __name__ == '__main__':
    if len(sys.argv[1:]) != 1:
        print("Invalid argument.")
        sys.exit(1)
    main(sys.argv[1])