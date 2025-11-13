#!/usr/bin/env python3
#
# example of using GPIO pushbuttons e.g. with keigan pi kit
#
import RPi.GPIO as GPIO
from enum import Enum

# buttons as per keigan pi you can re-allocate here
BUTTON_RED_PIN = 13
BUTTON_RED_PIN_2 = 6 
BUTTON_YELLOW_PIN = 19
BUTTON_GREEN_PIN = 26
OUTPUT_IND_PIN = 11
RUN_IT = True

class State(Enum):
    """
    state engine controlled by buttons

    """
    STATE_IDLE = 0 
    STATE_LINE_TRACE = 1 
    STATE_DEBUG = 10 

def red_callback(gpio_pin):
    time.sleep(0.05)
    if GPIO.input(gpio_pin) == GPIO.LOW:
        set_state(State.STATE_IDLE)
        print("red pushed")
        GPIO.output(OUTPUT_IND_PIN, False)

def yellow_callback(gpio_pin):
    global RUN_IT
    time.sleep(0.05)
    RUN_IT = False
    print("yellow pushed: stopped program")

def green_callback(gpio_pin):
    time.sleep(0.05)
    if GPIO.input(gpio_pin) == GPIO.LOW:
        set_state(State.STATE_LINE_TRACE)
        print("green pushed")
        GPIO.output(OUTPUT_IND_PIN, True)

if __name__ == "__main__":

    GPIO.setmode(GPIO.BCM) 

    GPIO.setup(BUTTON_RED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON_RED_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
    GPIO.setup(BUTTON_YELLOW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
    GPIO.setup(BUTTON_GREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
    GPIO.setup(OUTPUT_IND_PIN, GPIO.OUT) 

    GPIO.add_event_detect(BUTTON_RED_PIN, GPIO.FALLING, callback=red_callback, bouncetime=50)
    GPIO.add_event_detect(BUTTON_RED_PIN_2, GPIO.FALLING, callback=red_callback, bouncetime=50)
    GPIO.add_event_detect(BUTTON_YELLOW_PIN, GPIO.FALLING, callback=yellow_callback, bouncetime=50)
    GPIO.add_event_detect(BUTTON_GREEN_PIN, GPIO.FALLING, callback=green_callback, bouncetime=50)

    while RUN_IT == True:
        time.sleep(0.01)
    GPIO.cleanup()