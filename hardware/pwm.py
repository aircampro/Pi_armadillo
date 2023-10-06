#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# control PWM from raspberry pi

import time
import RPi.GPIO as GPIO

MOTOR_PORT = 19
WAIT_TIME = 1.0

# SG90
# 
# Position "0" (1.45 ms pulse) is middle,
# "90" (~2.4 ms pulse) is all the way to the right,
# "-90" (~ 0.5 ms pulse) is all the way left.

PWM_FREQUENCY = 50                     # Hz
PWM_CYCLE = 1/PWM_FREQUENCY * 1000     # ms
DUTY_MIDDLE   = 1.45 / PWM_CYCLE * 100 # %
DUTY_PLUS_90  = 2.4  / PWM_CYCLE * 100 # %
DUTY_MINUS_90 = 0.5  / PWM_CYCLE * 100 # %

# Software PWM
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PORT, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PORT, PWM_FREQUENCY)
pwm.start(DUTY_MIDDLE)
time.sleep(WAIT_TIME)

for i in range(3):
    for duty in [DUTY_MINUS_90,
                 DUTY_MIDDLE,
                 DUTY_PLUS_90,
                 DUTY_MIDDLE,]:
        pwm.ChangeDutyCycle(duty)
        time.sleep(WAIT_TIME)

time.stop()
GPIO.cleanup()