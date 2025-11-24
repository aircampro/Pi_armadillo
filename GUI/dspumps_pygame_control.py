#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Duty standby pump controller using pygame HMI and raspi i/o (by changing the scheduler reads data can come from other sources e.g. modbus, siemens etc)
#
# usage <prog> (time to start in 0.01 secs)
#
import sys
import pygame

# in this example we will use nose to check for the pump start up and if not raise the exception for the logic when it didnt start in time
# pip install nose 
#
from unittest import TestCase
from nose.tools import timed
import time
import threading
import RPi.GPIO as GPIO
import pickle
import os

# global data used by duty-standby pump logic
P1OUT=0
P1RUN=0
P1FAIL=0
P2OUT=0
P2RUN=0
P2FAIL=0
COUT1=0
COUT2=0
COUT3=0
COUT4=0
GT1=0
GT2=0
RFT=10                                               # anti noise debounce on the run signal
if len(sys.argv[0]) >= 1:
    TTS=int(sys.argv[1])                             # read time to start/stop from command line argument
else:
    TTS=600                                          # time to start/stop in 0.01 seconds
RUNNING_PMP=0                                        # no pump running
RUN_CMD_INTERVAL = 0.05                              # timed scheduled i/o routine
# GPIO set-up
p1run_pin = 13                                       # set these to how you wired it up physically
p2run_pin = 6
p1op_pin = 19
p2op_pin = 36
esd_pin = 7
running = True                                       # global run flag

class PowerSaveData:                                 # saves the retentive data for power cycle
    p_state = 0
    duty = 0

def change_duty(p):
    if p.duty == 0:
        p.duty = 1
    else:
        p.duty = 0

def reset_pump1():
    global P1FAIL
    global COUT1
    global COUT3
    global GT1
    if not GPIO.input(esd_pin) == GPIO.LOW:           # dont allow reset if esd still present
        P1FAIL = 0
        COUT1 = 0
        COUT3 = 0
        GT1 = 0

def reset_pump2():
    global P2FAIL
    global COUT2
    global COUT4
    global GT2
    if not GPIO.input(esd_pin) == GPIO.LOW:           # dont allow reset if esd still present
        P2FAIL = 0
        COUT2 = 0
        COUT4 = 0
        GT2 = 0

# start the pump and either get running feedback (started ok, or timeout fail)
def start_pump1_feedback():
    global P1OUT
    global COUT1
    global RUNNING_PMP
    P1OUT = 1
    while not P1RUN and COUT1 < TTS:
        time.sleep(0.01)
        COUT1 += 1
    if COUT1 >= TTS:
        P1OUT = 0
    else:
        RUNNING_PMP = 1

def start_pump2_feedback():
    global P2OUT
    global COUT2
    P2OUT = 1
    while not P2RUN and COUT2 < TTS:
        time.sleep(0.01)
        COUT2 += 1
    if COUT2 >= TTS:
        P2OUT = 0
    else:
        RUNNING_PMP = 2

def stop_pump1_feedback():
    global P1OUT
    global COUT3
    P1OUT = 0
    while P1RUN and COUT3 < TTS:
        time.sleep(0.01)
        COUT3 += 1
    if RUNNING_PMP == 1:
        RUNNING_PMP = 0

def stop_pump2_feedback():
    global P2OUT
    global COUT4
    P2OUT = 0
    while P2RUN and COUT4 < TTS:
        time.sleep(0.01)
        COUT4 += 1
    if RUNNING_PMP == 2:
        RUNNING_PMP = 0

# run the unit operation (pump start/stop) or timeout failure
class CheckPumpStatus(TestCase):
    @timed((TTS/100)-0.1)
    def test_pump1_start(self):
        start_pump1_feedback()
    @timed((TTS/100)-0.1)
    def test_pump2_start(self):
        start_pump2_feedback()
    @timed((TTS/100)-0.1)
    def test_pump1_stop(self):
        stop_pump1_feedback()
    @timed((TTS/100)-0.1)
    def test_pump2_stop(self):
        stop_pump2_feedback()

def run_pump2():
    global P2FAIL
    pmps=CheckPumpStatus()
    try:
        pmps.test_pump2_start()
    except:
        if not P2RUN:
            print("pump 2 failed to start in time")
            P2FAIL=1
            run_pump1()

def stop_pump2():
    global P2FAIL
    pmps=CheckPumpStatus()
    try:
        pmps.test_pump2_stop()
    except:
        print("pump 2 failed to stop in time")
        P2FAIL=2

def run_pump1():
    global P1FAIL
    pmps=CheckPumpStatus()
    try:
        pmps.test_pump1_start()
    except:
        if not P1RUN:
            print("pump 1 failed to start in time")
            P1FAIL=1
            run_pump2()

def stop_pump1():
    global P1FAIL
    pmps=CheckPumpStatus()
    try:
        pmps.test_pump1_stop()
    except:
        print("pump 1 failed to stop in time")
        P1FAIL=2

# update globals periodicaly with live data and perform change over if failure of running pump inline  
def scheduler():
    global P1RUN, P2RUN, P1FAIL, P2FAIL, GT1, GT2
    t = threading.Timer(RUN_CMD_INTERVAL, scheduler)
    t.start()
    P1RUN = GPIO.input(p1run_pin) == GPIO.HIGH
    P2RUN = GPIO.input(p2run_pin) == GPIO.HIGH
    GPIO.output(p1op_pin, P1OUT)
    GPIO.output(p2op_pin, P2OUT)
    # if a running pump stops try to change to the standby
    if RUNNING_PMP == 1 and not P1RUN:
        GT1 += 1
        if GT1 >= RFT:
            P1FAIL = 1
            stop_pump1()
            run_pump2()
    elif RUNNING_PMP == 2 and not P2RUN:
        GT2 += 1
        if GT2 >= RFT:
            P2FAIL = 1
            stop_pump2()
            run_pump1() 
    else:
        GT1 = 0
        GT2 = 0

def esd_callback(gpio_pin):
    stop_pump1()
    stop_pump2()
    P1FAIL=3
    P2FAIL=3

# define the screen size
WIDTH = 1280.0
HEIGHT = 720.0
# text (data) positions on the screen
TPOSX=200
TPOSY=100
TSPACER=50

# mouse actions to pygame
LEFT_CLK=0
MID_CLK=1
RIGHT_CLK=2

# init screen and controls 
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(" Control the Pump ")
clock  = pygame.time.Clock()
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
CRAD = 25
BX=TPOSX
BY=HEIGHT-100                                                         # buttons @ bottom of the screen
font = pygame.font.Font(None, 50)

# this is if you want a button to press on the screen to do something once centre mouse wheel clicked
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/stop_btn.png" 
btn_img = pygame.image.load(str(img_path)) 
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/start_btn.png" 
start_img = pygame.image.load(str(img_path)) 
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/duty_btn.png" 
duty_img = pygame.image.load(str(img_path)) 
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/reset_btn.png" 
reset_img = pygame.image.load(str(img_path)) 
# draw the click buttons for the action record and replay
stop_bt = screen.blit(btn_img, (BX, BY))
start_bt = screen.blit(start_img, (BX+50, BY))
duty_bt = screen.blit(duty_img, (BX+100, BY))
reset_bt = screen.blit(reset_img, (BX+150, BY))

# run the controls for reading keyboard and mouse and draw info screen with action choice push buttons
def main_run():
    global running
    pushFlag1 = False
    pushFlag2 = False 
    pushFlag3 = False 
    pushFlag4 = False 
    p_f_name = "ds_pickle.pcl"                                                            # name of the pickle file for this group 
    try:                                                                                  # power cycle state recovery from pickle file
        with open(p_f_name, 'rb') as f:
            p = pickle.load(f)                                                            # load the last saved state before power off from pickle file
    except:
        p=PowerSaveData()                                                                 # create a new power save object first time no pickle file
    (mx, my) = pygame.mouse.get_pos()
    GPIO.setmode(GPIO.BCM) 
    GPIO.setup(p1run_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)                              # input config
    GPIO.setup(p2run_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(esd_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(p1op_pin, GPIO.OUT)                                                        # output config
    GPIO.setup(p2op_pin, GPIO.OUT)
    GPIO.add_event_detect(esd_pin, GPIO.FALLING, callback=esd_callback, bouncetime=50)    # emergency stop interrupt handler activation
    act_txt = " "  
    t = threading.Thread(target = scheduler)                                              # io scheduler
    t.start()    
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

        # --------- mouse controls for button selection ---------
        mdown = pygame.mouse.get_pressed()
        (mx, my) = pygame.mouse.get_pos()
        # mdown[0]	Left-click        (draws image)
        # mdown[1]	Middle click      (wheel)
        # mdown[2]	Right-click       (draws flipped image)
        if mdown[LEFT_CLK]:
            pushFlag1 = False
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False           
        elif mdown[RIGHT_CLK]:
            pushFlag1 = False  
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False 
        if mdown[MID_CLK]:                                                                  # middle wheel down activates the screen button selection
            print("wheel pressed checking to see if it was over the button")
            if stop_bt.collidepoint(mx, my) and pushFlag1 == False: 
                stop_pump1()
                stop_pump2()
                pushFlag1 = True 
                act_txt = "stop pumps" 
                p.p_state = 2                
            elif start_bt.collidepoint(mx, my) and pushFlag2 == False: 
                if p.duty == 0:
                    start_pump1()
                else:
                    start_pump2()
                pushFlag2 = True  
                act_txt = "start duty pump" 
                p.p_state = 1                
            elif duty_bt.collidepoint(mx, my) and pushFlag3 == False: 
                change_duty(p)
                stop_pump1()
                stop_pump2()
                if p.p_state == 1:
                    if p.duty == 0:
                        start_pump1()
                    else:
                        start_pump2()
                pushFlag3 = True 
                act_txt = "change duty"                 
            elif reset_bt.collidepoint(mx, my) and pushFlag4 == False: 
                reset_pump1()
                reset_pump2()
                stop_pump1()
                stop_pump2()
                if p.duty == 0:
                    start_pump1()
                else:
                    start_pump2()
                pushFlag4 = True 
                act_txt = "reset"                                 
        else:
            pushFlag1 = False  
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False 

        if P1RUN:
            textimg1 = font.render(f"Pump ① RUNNING", True, pygame.Color("GREEN"))
        else:
            textimg1 = font.render(f"Pump ① STOPPED", True, pygame.Color("RED"))  
        if P1FAIL == 1:
            textimg2 = font.render(f"Pump ① FAILED TO START", True, pygame.Color("RED"))
        elif P1FAIL == 2:
            textimg2 = font.render(f"Pump ① FAILED TO STOP", True, pygame.Color("RED"))
        elif P1FAIL == 3:
            textimg2 = font.render(f"Pump ① ESD BUTTON PRESSED", True, pygame.Color("RED"))
        else:
            textimg2 = font.render(f"Pump ① OK", True, pygame.Color("GREEN")) 
        if P2RUN:
            textimg3 = font.render(f"Pump ② RUNNING", True, pygame.Color("GREEN"))
        else:
            textimg3 = font.render(f"Pump ② STOPPED", True, pygame.Color("RED"))  
        if P2FAIL == 1:
            textimg4 = font.render(f"Pump ② FAILED TO START", True, pygame.Color("RED"))
        elif P2FAIL == 2:
            textimg4 = font.render(f"Pump ② FAILED TO STOP", True, pygame.Color("RED"))
        elif P2FAIL == 3:
            textimg4 = font.render(f"Pump ② ESD BUTTON PRESSED", True, pygame.Color("RED"))
        else:
            textimg4 = font.render(f"Pump ② OK", True, pygame.Color("GREEN"))            
        textimg5 = font.render(f"Running pump : {RUNNING_PMP} duty : {p.duty+1} action: {act_txt}", True, pygame.Color("BLUE"))
        screen.blit(textimg1, (TPOSX, TPOSY))
        screen.blit(textimg2, (TPOSX, TPOSY+SPACER))
        screen.blit(textimg3, (TPOSX, TPOSY+(2*SPACER)))
        screen.blit(textimg4, (TPOSX, TPOSY+(3*SPACER)))
        screen.blit(textimg5, (TPOSX, TPOSY+(4*SPACER)))
        pygame.display.update()
        with open(p_f_name, 'wb') as f:                                   # save states for power cycle reset
            pickle.dump(p, f) 
        clock.tick(60)                                                    # refresh @ 60hz
    pygame.quit()
    GPIO.cleanup()

if __name__ == "__main__":
    main_run()