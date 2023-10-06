#!/usr/bin/env python
 
"""
This drives LED port on pi
"""

import RPi.GPIO as GPIO
import time

LED_PORT = 18

GPIO.setmode(GPIO.BCM)          
GPIO.setup(LED_PORT, GPIO.OUT)

GPIO.output(LED_PORT, True)
time.sleep(5)
GPIO.output(LED_PORT, False)

GPIO.cleanup()

print('done')