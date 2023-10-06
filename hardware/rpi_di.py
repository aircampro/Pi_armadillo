#!/usr/bin/env python
 
"""
This simple switch input on the raspberry pi
"""

import RPi.GPIO as GPIO
import time

SWITCH_PORT = 23

GPIO.setmode(GPIO.BCM)         
GPIO.setup(SWITCH_PORT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

state = False
while True:
    current_state = GPIO.input(SWITCH_PORT)
    if current_state != state:
        if current_state:
            print('OFF')
        else:
            print('Pushed')

        state = current_state