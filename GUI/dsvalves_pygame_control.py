#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Duty standby valve controller using pygame HMI and raspi i/o (by changing the scheduler reads data can come from other sources e.g. modbus, siemens etc)
#
# usage <prog> (time to open in 0.01 secs)
#
import sys
import pygame

# in this example we will use nose to check for the valve open up and if not raise the exception for the logic when it didnt open in time
# pip install nose 
#
from unittest import TestCase
from nose.tools import timed
import time
import threading
import RPi.GPIO as GPIO
import pickle
import os

# global data used by duty-standby valve logic
V1OUT=0
V1STATE=0
V1FAIL=0
V2OUT=0
V2STATE=0
V2FAIL=0
COUT1=0
COUT2=0
COUT3=0
COUT4=0
GT1=0
GT2=0
RFT=10                                               # anti noise debounce on the run signal
if len(sys.argv[0]) >= 1:
    TTS=int(sys.argv[1])                             # read time to open/close from command line argument
else:
    TTS=6000                                         # time to open/close in 0.01 seconds
DTY_VLV=0                                            # no valve running
RUN_CMD_INTERVAL = 0.05                              # timed scheduled i/o routine
# GPIO set-up
V1opn_pin = 13                                       # set these to how you wired it up physically
V2opn_pin = 6
V1cls_pin = 15
V2cls_pin = 18
v1op_pin = 19
v2op_pin = 36
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

def reset_valve1():
    global V1FAIL
    global COUT1
    global COUT3
    global GT1
    if not GPIO.input(esd_pin) == GPIO.LOW:           # dont allow reset if esd still present
        V1FAIL = 0
        COUT1 = 0
        COUT3 = 0
        GT1 = 0
        return 1
    else:
        return 0

def reset_valve2():
    global V2FAIL
    global COUT2
    global COUT4
    global GT2
    if not GPIO.input(esd_pin) == GPIO.LOW:           # dont allow reset if esd still present
        V2FAIL = 0
        COUT2 = 0
        COUT4 = 0
        GT2 = 0
        return 1
    else:
        return 0

# open the valve and either get running feedback (opened ok, or timeout fail)
def open_valve1_feedback():
    global V1OUT
    global COUT1
    global DTY_VLV
    V1OUT = 1
    while not V1STATE == 1 and COUT1 < TTS:
        time.sleep(0.01)
        COUT1 += 1
    if COUT1 >= TTS:
        V1OUT = 0
    else:
        DTY_VLV = 1

def open_valve2_feedback():
    global V2OUT
    global COUT2
    V2OUT = 1
    while not V2STATE == 1 and COUT2 < TTS:
        time.sleep(0.01)
        COUT2 += 1
    if COUT2 >= TTS:
        V2OUT = 0
    else:
        DTY_VLV = 2

def close_valve1_feedback():
    global V1OUT
    global COUT3
    V1OUT = 0
    while not V1STATE == 2 and COUT3 < TTS:
        time.sleep(0.01)
        COUT3 += 1
    if DTY_VLV == 1:
        DTY_VLV = 0

def close_valve2_feedback():
    global V2OUT
    global COUT4
    V2OUT = 0
    while not V2STATE == 2 and COUT4 < TTS:
        time.sleep(0.01)
        COUT4 += 1
    if DTY_VLV == 2:
        DTY_VLV = 0

# run the unit operation (valve open/close) or timeout failure
class CheckvalveStatus(TestCase):
    @timed((TTS/100)-0.1)
    def test_valve1_open(self):
        open_valve1_feedback()
    @timed((TTS/100)-0.1)
    def test_valve2_open(self):
        open_valve2_feedback()
    @timed((TTS/100)-0.1)
    def test_valve1_close(self):
        close_valve1_feedback()
    @timed((TTS/100)-0.1)
    def test_valve2_close(self):
        close_valve2_feedback()

def open_valve2():
    global V2FAIL
    pmps=CheckvalveStatus()
    try:
        pmps.test_valve2_open()
    except:
        if not V2STATE == 1:
            print("valve 2 failed to open in time")
            V2FAIL=1
            open_valve1()

def close_valve2():
    global V2FAIL
    pmps=CheckvalveStatus()
    try:
        pmps.test_valve2_close()
    except:
        if not V2STATE == 2:
            print("valve 2 failed to close in time")
            V2FAIL=2

def open_valve1():
    global V1FAIL
    pmps=CheckvalveStatus()
    try:
        pmps.test_valve1_open()
    except:
        if not V1STATE == 1:
            print("valve 1 failed to open in time")
            V1FAIL=1
            open_valve2()

def close_valve1():
    global V1FAIL
    pmps=CheckvalveStatus()
    try:
        pmps.test_valve1_close()
    except:
        if not V1STATE == 2:
            print("valve 1 failed to close in time")
            V1FAIL=2

# update globals periodicaly with live data and perform change over if failure of running valve inline  
def scheduler():
    global V1STATE, V2STATE, V1FAIL, V2FAIL, GT1, GT2
    t = threading.Timer(RUN_CMD_INTERVAL, scheduler)
    t.start()
    if GPIO.input(V1opn_pin) == GPIO.HIGH:
        V1STATE |= 1
    else:
        V1STATE &= ~1
    if GPIO.input(V1cls_pin) == GPIO.HIGH:
        V1STATE |= 2
    else:
        V1STATE &= ~2        
    if GPIO.input(V2opn_pin) == GPIO.HIGH:
        V2STATE |= 1
    else:
        V2STATE &= ~1
    if GPIO.input(V2cls_pin) == GPIO.HIGH:
        V2STATE |= 2
    else:
        V2STATE &= ~2
    GPIO.output(v1op_pin, V1OUT)
    GPIO.output(v2op_pin, V2OUT)
    # if a open valve closes try to change to the standby
    if DTY_VLV == 1 and not V1STATE == 1:
        GT1 += 1
        if GT1 >= RFT:
            V1FAIL = 1
            close_valve1()
            open_valve2()
    elif DTY_VLV == 2 and not V2STATE == 1:
        GT2 += 1
        if GT2 >= RFT:
            V2FAIL = 1
            close_valve2()
            open_valve1() 
    else:
        GT1 = 0
        GT2 = 0

def esd_callback(gpio_pin):
    close_valve1()
    close_valve2()
    V1FAIL=3
    V2FAIL=3

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
pygame.display.set_caption(" Control the valve ")
clock  = pygame.time.Clock()
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
CRAD = 25
BX=TPOSX
BY=HEIGHT-100                                                         # buttons @ bottom of the screen
pygame.font.init()
font = pygame.font.Font(None, 50)

# this is if you want a button to press on the screen to do something once centre mouse wheel clicked
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/close_btn.png" 
btn_img = pygame.image.load(str(img_path)) 
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/open_btn.png" 
open_img = pygame.image.load(str(img_path)) 
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/duty_btn.png" 
duty_img = pygame.image.load(str(img_path)) 
img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/reset_btn.png" 
reset_img = pygame.image.load(str(img_path)) 
# draw the click buttons for the action record and replay
close_bt = screen.blit(btn_img, (BX, BY))
open_bt = screen.blit(open_img, (BX+50, BY))
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
    GPIO.setup(V1opn_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)                              # input config
    GPIO.setup(V1cls_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(V2opn_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)                              
    GPIO.setup(V2cls_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(esd_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(v1op_pin, GPIO.OUT)                                                        # output config
    GPIO.setup(v2op_pin, GPIO.OUT)
    GPIO.add_event_detect(esd_pin, GPIO.FALLING, callback=esd_callback, bouncetime=50)    # emergency close interrupt handler activation
    act_txt = " "  
    t = threading.Thread(target = scheduler)                                              # io scheduler
    t.start()    
    while running:
        # draw the click buttons for the action record and replay
        close_bt = screen.blit(btn_img, (BX, BY))
        open_bt = screen.blit(open_img, (BX+50, BY))
        duty_bt = screen.blit(duty_img, (BX+100, BY))
        reset_bt = screen.blit(reset_img, (BX+150, BY))
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
            if close_bt.collidepoint(mx, my) and pushFlag1 == False: 
                close_valve1()
                close_valve2()
                pushFlag1 = True 
                act_txt = "close valves" 
                p.p_state = 2                
            elif open_bt.collidepoint(mx, my) and pushFlag2 == False: 
                if p.duty == 0:
                    open_valve1()
                else:
                    open_valve2()
                pushFlag2 = True  
                act_txt = "open duty valve" 
                p.p_state = 1                
            elif duty_bt.collidepoint(mx, my) and pushFlag3 == False: 
                change_duty(p)
                close_valve1()
                close_valve2()
                if p.p_state == 1:
                    if p.duty == 0:
                        open_valve1()
                    else:
                        open_valve2()
                pushFlag3 = True 
                act_txt = "change duty"                 
            elif reset_bt.collidepoint(mx, my) and pushFlag4 == False: 
                v=reset_valve1()
                if v == 1:
                    v=reset_valve2()
                if v == 1:
                    close_valve1()
                    close_valve2()
                    if p.duty == 0:
                        open_valve1()
                    else:
                        open_valve2()
                pushFlag4 = True 
                act_txt = "reset"                                 
        else:
            pushFlag1 = False  
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False 

        if V1STATE == 0:
            textimg1 = font.render(f"valve ① TRAVELLING", True, pygame.Color("YELLOW"))
        elif V1STATE == 1:
            textimg1 = font.render(f"valve ① OPEN", True, pygame.Color("GREEN"))
        elif V1STATE == 2:
            textimg1 = font.render(f"valve ① CLOSED", True, pygame.Color("RED"))
        else:
            textimg1 = font.render(f"valve ① UNKNOWN", True, pygame.Color("BLUE"))  
        if V1FAIL == 1:
            textimg2 = font.render(f"valve ① FAILED TO OPEN", True, pygame.Color("RED"))
        elif V1FAIL == 2:
            textimg2 = font.render(f"valve ① FAILED TO CLOSE", True, pygame.Color("RED"))
        elif V1FAIL == 3:
            textimg2 = font.render(f"valve ① ESD BUTTON PRESSED", True, pygame.Color("RED"))
        else:
            textimg2 = font.render(f"valve ① OK", True, pygame.Color("GREEN")) 
        if V2STATE == 0:
            textimg3 = font.render(f"valve ② TRAVELLING", True, pygame.Color("GREEN"))
        elif V2STATE == 1:
            textimg3 = font.render(f"valve ② OPEN", True, pygame.Color("GREEN"))
        elif V2STATE == 2:
            textimg3 = font.render(f"valve ② CLOSED", True, pygame.Color("RED"))
        else:
            textimg3 = font.render(f"valve ② UNKNOWN", True, pygame.Color("BLUE"))  
        if V2FAIL == 1:
            textimg4 = font.render(f"valve ② FAILED TO OPEN", True, pygame.Color("RED"))
        elif V2FAIL == 2:
            textimg4 = font.render(f"valve ② FAILED TO CLOSE", True, pygame.Color("RED"))
        elif V2FAIL == 3:
            textimg4 = font.render(f"valve ② ESD BUTTON PRESSED", True, pygame.Color("RED"))
        else:
            textimg4 = font.render(f"valve ② OK", True, pygame.Color("GREEN"))            
        textimg5 = font.render(f"Open valve : {DTY_VLV} duty : {p.duty+1} action: {act_txt}", True, pygame.Color("BLUE"))
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
