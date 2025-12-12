#!/usr/bin/python
#
# example of a simple tank with a discharge pump fill and empty operations
#
import signal
import time
import RPi.GPIO as GPIO
import pygame

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
pygame.display.set_caption(" Tank Sequence Control ")
clock  = pygame.time.Clock()
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
CRAD = 25
BX=TPOSX
BY=HEIGHT-100                                                         # buttons @ bottom of the screen
font = pygame.font.Font(None, 50)

inputs = [ 1, 0, 0, 1, 0, 1, 0 ]                                       # states v1 open v1 cls v2 open v2 cls p1 off tank low level tank high high lvel 
ipins = [ 17, 16, 11, 21, 22, 23, 26 ]
outputs = [ 1, 0, 0 ]                                                  # drives v1 open v2 close p1 off
opins = [ 32, 33, 31 ]                                                 # create a new power save object first time no pickle file
p_f_name = "seq_state.pcl" 
try:                                                                   # power cycle state recovery from pickle file
    with open(p_f_name, 'rb') as f:
        SEQ_STATE = pickle.load(f)                                     # load the last saved state before power off from pickle file   
except:
    SEQ_STATE=0
STATE_REACHED = 0
V_TRAVEL_T=10                                                          # valve travel time
ACTIVE = True
instate = 1                                                            # state for the inlet valve depending on the hi hi level in the tank

# get input i.o
def get_signals():
    global inputs
    inputs = []
    for i in range(0,len(ipins)):
        if GPIO.input(i) == GPIO.LOW:
            inputs.append(0)
        else:
            inputs.append(1) 

# set output i.o    
def put_signals():
    for i, o in enumerate(outputs):
        GPIO.output(ipins[i], o)

# time alarm triggered handler which checks if we reached the correct state after the time or if not flags a failure to the HMI
def handler(signum, frame):
    global STATE_REACHED
    get_signals()                                                      # update the i/o (no need for mutex as we refresh i.o here)
    print(f'handler (signum={signum})')

    if inputs[0] == instate and inputs[1] == 1 and inputs[2] == 0 and inputs[3] == 1 and inputs[4] == 0 and SEQ_STATE == 0 and START_PB == 1:
	    STATE_REACHED = 1
    elif inputs[0] == instate and inputs[1] == 1 and inputs[2] == 1 and inputs[3] == 0 and inputs[4] == 0 and SEQ_STATE == 1:
	    STATE_REACHED = 1
    elif inputs[0] == instate and inputs[1] == 1 and inputs[2] == 1 and inputs[3] == 0 and inputs[4] == 1 and SEQ_STATE == 2:
	    STATE_REACHED = 1
    elif inputs[1] == 0 and inputs[0] == instate and inputs[2] == 0 and inputs[3] == 1 and inputs[4] == 0 and STOP_PB == 1:
	    STATE_REACHED = 1
    else:
        STATE_REACHED = 2
     
# timed alarm handler
signal.signal(signal.SIGALRM, handler)

def sequence():
    global ACTIVE
    global instate
    global STATE_REACHED
    global SEQ_STATE
    global outputs
    GPIO.setmode(GPIO.BCM) 
    for i in range(0,len(ipins)):    
        GPIO.setup(i, GPIO.IN, pull_up_down=GPIO.PUD_UP)                              # input config
    for o in range(0,len(opins)): 
        GPIO.setup(o, GPIO.OUT)                                                       # output config

    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/stop_btn.png" 
    btn_img = pygame.image.load(str(img_path)) 
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/start_btn.png" 
    start_img = pygame.image.load(str(img_path)) 
    # draw the click buttons for the action record and replay
    stop_bt = screen.blit(btn_img, (BX, BY))
    start_bt = screen.blit(start_img, (BX+50, BY))

    # load sequence state sprites
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step0.png" 
    s0 = pygame.image.load(str(img_path))
    s0scale = pygame.transform.scale(s0, (50, 50))
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step1.png" 
    s1 = pygame.image.load(str(img_path))
    s1scale = pygame.transform.scale(s1, (50, 50))
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step2.png" 
    s2 = pygame.image.load(str(img_path))
    s2scale = pygame.transform.scale(s2, (50, 50))
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step3.png" 
    s3 = pygame.image.load(str(img_path))
    s3scale = pygame.transform.scale(s3, (50, 50))

    while ACTIVE == True:
        # --------- mouse controls for button selection ---------
        mdown = pygame.mouse.get_pressed()
        (mx, my) = pygame.mouse.get_pos()
        # mdown[0]	Left-click        
        # mdown[1]	Middle click      (wheel)
        # mdown[2]	Right-click       
        if mdown[LEFT_CLK]:
            pushFlag1 = False
            pushFlag2 = False      
        elif mdown[RIGHT_CLK]:
            pushFlag1 = False  
            pushFlag2 = False 
        if mdown[MID_CLK]:                                                                  # middle wheel down activates the screen button selection
            print("wheel pressed checking to see if it was over the button")
            if stop_bt.collidepoint(mx, my) and pushFlag1 == False: 
                if SEQ_STATE == 0:
                    START_PB = 1
                pushFlag1 = True                 
            elif start_bt.collidepoint(mx, my) and pushFlag2 == False: 
                STOP_PB = 1
                pushFlag2 = True  
        get_signals()                                   # update i.o
        if inputs[6] == 1:                              # high high lvel will force input closed
            instate = 0                                 # inlet is interlocked closed 
        else:
            instate = 1                                 # permit opening of the inlet
        if SEQ_STATE == 0:                              # wait for start push button
            if instate == 1:
                screen.blit(s0scale, (TPOSX, TPOSY))
            else:
                screen.blit(s1scale, (TPOSX, TPOSY))            
            if START_PB == 1:
                outputs = [ 0, 0, 0 ]                   # close inlet valve
                signal.alarm(V_TRAVEL_T)                # check valve limits after travel time
                if STATE_REACHED == 1:
                    SEQ_STATE = 1
                    START_PB = 0
            elif STOP_PB == 1:
                outputs = [ instate, 0, 0 ]             # open inlet valve
                signal.alarm(V_TRAVEL_T)
                if STATE_REACHED == 1:
                    STOP_PB = 0		
            else:
                outputs = [ instate, 0, 0 ]             # open inlet valve
                signal.alarm(V_TRAVEL_T)
        elif SEQ_STATE == 1:                            # open outlet valve
            outputs = [ 0, 1, 0 ]                      
            signal.alarm(V_TRAVEL_T)                    # check valve limits after travel time
            if STATE_REACHED == 1:
                SEQ_STATE = 2
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                   
                signal.alarm(V_TRAVEL_T)
                if STATE_REACHED == 1:
                    STOP_PB = 0	
                    SEQ_STATE = 0
            screen.blit(s1scale, (TPOSX, TPOSY))
        elif SEQ_STATE == 2:                           # start pump
            if inputs[SEQ_STATE+5] == 1:               # conductivity probe covered then start pump
                outputs = [ 0, 1, 1 ] 
            else:
                STATE_REACHED = 2			
            signal.alarm(V_TRAVEL_T)                   # check valve limits after travel time
            if STATE_REACHED == 1:
                SEQ_STATE = 3
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                   
                signal.alarm(V_TRAVEL_T)
                if STATE_REACHED == 1:
                    STOP_PB = 0	
                    SEQ_STATE = 0
            screen.blit(s2scale, (TPOSX, TPOSY))
        elif SEQ_STATE == 3:                           # wait for level to drop and return to filling
            if inputs[SEQ_STATE+5] == 0:               # conductivity probe un-covered then stop pump
                outputs = [ instate, 0, 0 ]            # open inlet valve
                signal.alarm(V_TRAVEL_T)
                if STATE_REACHED == 1:
                    SEQ_STATE = 0			
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                   
                signal.alarm(V_TRAVEL_T)
                if STATE_REACHED == 1:
                    STOP_PB = 0	
                    SEQ_STATE = 0
            screen.blit(s3scale, (TPOSX, TPOSY))
        put_signals()                                 # drive i.o
        if STATE_REACHED == 2:
            textimg1 = font.render("Sequence Error", True, pygame.Color("RED"))
        else:
            textimg1 = font.render("Sequence OK", True, pygame.Color("BLUE"))
        screen.blit(textimg1, (TPOSX, TPOSY+150))
        pygame.display.update()
        with open(p_f_name, 'wb') as f:                                   # save states for power cycle reset
            pickle.dump(SEQ_STATE, f) 
        clock.tick(60)                                                    # refresh @ 60hz
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                ACTIVE = False
    pygame.quit()
    GPIO.cleanup()

if __name__ == '__main__':
    sequence()

