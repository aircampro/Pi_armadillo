#!/usr/bin/python
#
# Example of a simple tank with a discharge pump fill and empty operations
# shows how to do a timed event without using a thread
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
RED = (255, 50, 0)
GREEN = (0, 255, 50)
BLUE = (0, 100, 255)
BLACK = (0, 0, 0)
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
signal_active = 0
stop_pump_on_to=True                                                   # if you timeout emptying then stop pump
EMPTY_TO=200.0
feed_flag = True                                                       # feed flag means continuosly feed until flag cleared by operator
signal.signal(signal.SIGALRM, handler)                                 # timed alarm handler
FAIL_STEP=0
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
    global signal_active
    global FAIL_STEP
    get_signals()                                                      # update the i/o (no need for mutex as we refresh i.o here)
    print(f'handler (signum={signum})')

    # confirmation of condition to exit of each step
    if inputs[0] == 0 and inputs[1] == 1 and inputs[2] == 0 and inputs[3] == 1 and inputs[4] == 0 and ((SEQ_STATE == 0 and START_PB == 1) or SEQ_STATE == 5):
	    STATE_REACHED = 1
    elif inputs[0] == 0 and inputs[1] == 1 and inputs[2] == 0 and inputs[3] == 1 and inputs[4] == 0 and (STOP_PB == 1 or SEQ_STATE == 0) and instate == 0:
	    STATE_REACHED = 1
    elif inputs[0] == 1 and inputs[1] == 0 and inputs[2] == 0 and inputs[3] == 1 and inputs[4] == 0 and (STOP_PB == 1 or SEQ_STATE == 0) and instate == 1:
	    STATE_REACHED = 1
    elif inputs[0] == 0 and inputs[1] == 1 and inputs[2] == 1 and inputs[3] == 0 and inputs[4] == 0 and (SEQ_STATE == 1 or SEQ_STATE == 4):
	    STATE_REACHED = 1
    elif inputs[0] == 0 and inputs[1] == 1 and inputs[2] == 0 and inputs[3] == 1 and inputs[4] == 0 and SEQ_STATE == 5:
	    STATE_REACHED = 1
    elif inputs[0] == 0 and inputs[1] == 1 and inputs[2] == 1 and inputs[3] == 0 and inputs[4] == 1 and SEQ_STATE == 2:
	    STATE_REACHED = 1
    elif inputs[0] == 1 and inputs[1] == 0 and inputs[2] == 1 and inputs[3] == 0 and inputs[4] == 1 and SEQ_STATE == 6 and inputs[5] == 1:
	    STATE_REACHED = 1
    elif inputs[0] == 1 and inputs[1] == 0 and inputs[2] == 1 and inputs[3] == 0 and inputs[4] == 0 and SEQ_STATE == 6 and inputs[5] == 0:
	    STATE_REACHED = 1
    elif SEQ_STATE == 3:
	    pass
    else:
        STATE_REACHED = 2
        FAIL_STEP = SEQ_STATE
    signal_active = 0

# set a new signal to check state if we are not already waiting then call the handler after that time
def set_signal(tim, caller):
    global signal_active
    if not signal_active == caller:
        signal.alarm(tim)
        signal_active = caller  

def sequence():
    global ACTIVE
    global instate
    global STATE_REACHED
    global SEQ_STATE
    global outputs
    global signal_active
    global feed_flag

    step6_flg = False
    GPIO.setmode(GPIO.BCM) 
    for i in range(0,len(ipins)):    
        GPIO.setup(i, GPIO.IN, pull_up_down=GPIO.PUD_UP)                              # input config
    for o in range(0,len(opins)): 
        GPIO.setup(o, GPIO.OUT)                                                       # output config

    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/stop_btn.png" 
    btn_img = pygame.image.load(str(img_path)) 
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/start_btn.png" 
    start_img = pygame.image.load(str(img_path)) 
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step_btn.png" 
    step_img = pygame.image.load(str(img_path)) 
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/feed_btn.png" 
    feed_img = pygame.image.load(str(img_path)) 
    # draw the click buttons for the action record and replay
    stop_bt = screen.blit(btn_img, (BX, BY))
    start_bt = screen.blit(start_img, (BX+50, BY))
    step_bt = screen.blit(start_img, (BX+100, BY))
    feed_bt = screen.blit(feed_img, (BX+150, BY))
    
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
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step6.png" 
    s6 = pygame.image.load(str(img_path))
    s6scale = pygame.transform.scale(s6, (50, 50))
    img_path = os.path.dirname(os.path.abspath("__file__")) + "/images" + "/step6_low.png" 
    s6l = pygame.image.load(str(img_path))
    s6lscale = pygame.transform.scale(s6l, (50, 50))

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
        elif mdown[MID_CLK]:                                                                  # middle wheel down activates the screen button selection
            print("wheel pressed checking to see if it was over the button")
            if stop_bt.collidepoint(mx, my) and pushFlag1 == False:                         # stop sequence 
                STOP_PB = 1
                pushFlag1 = True               
            elif start_bt.collidepoint(mx, my) and pushFlag2 == False:                      # start sequence 
                if SEQ_STATE == 0:
                    START_PB = 1
                    pushFlag2 = True  
            elif step_bt.collidepoint(mx, my):                                              # step sequence (can use on error)
                SEQ_STATE += 1
                SEQ_STATE = SEQ_STATE % 4
            elif feed_bt.collidepoint(mx, my):                                              # request constant feed if true
                feed_flag = not feed_flag                                                   # toggle the feed flag
                
        get_signals()                                   # update i.o
        # continously check the high tank level to control the inlet
        if inputs[6] == 1:                              # high high lvel will force input closed
            instate = 0                                 # inlet is interlocked closed 
        else:
            instate = 1                                 # permit opening of the inlet
        if SEQ_STATE == 0:                              # wait for start push button
            if instate == 1:
                screen.blit(s0scale, (TPOSX, TPOSY))
            else:
                screen.blit(s1scale, (TPOSX, TPOSY))  
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]             # open inlet valve
                set_signal(V_TRAVEL_T, 1)
                if STATE_REACHED == 1:
                    STOP_PB = 0	
	                pushFlag2 = False 
                    START_PB = 0
                    pushFlag1 = False    
                    STATE_REACHED = 0                    
            elif START_PB == 1:
                outputs = [ 0, 0, 0 ]                    # close inlet valve
                set_signal(V_TRAVEL_T, 2)                # check valve limits after travel time
                if STATE_REACHED == 1:
                    SEQ_STATE = 1
                    START_PB = 0
                    STATE_REACHED = 0
                    pushFlag1 = False
            else:
                outputs = [ instate, 0, 0 ]               # open inlet valve
                set_signal(V_TRAVEL_T, 3)
        elif SEQ_STATE == 1:                              # open outlet valve
            if STATE_REACHED == 1:
                SEQ_STATE = 2
                STATE_REACHED = 0
            if STOP_PB == 1:                              # we could just return to step 0 but show picture until back in that state
                outputs = [ instate, 0, 0 ]                   
                set_signal(V_TRAVEL_T, 1)
                if STATE_REACHED == 1:
                    STOP_PB = 0	
                    SEQ_STATE = 0
                    pushFlag2 = False 
                    STATE_REACHED = 0
            else:
                outputs = [ 0, 1, 0 ]                      
                set_signal(V_TRAVEL_T,4)                   # check valve limits after travel time            
            screen.blit(s1scale, (TPOSX, TPOSY))
        elif SEQ_STATE == 2:                               # start pump
            if inputs[5] == 1:                             # conductivity probe covered then start pump
                outputs = [ 0, 1, 1 ] 
            else:
                outputs = [ 0, 1, 0 ]
                if step6_flg == True:                      # returned from step6 is a valid condition
                    SEQ_STATE = 3  
                    step6_flg = False   
                else:                    
                    STATE_REACHED = 2			
            if STATE_REACHED == 1:
                SEQ_STATE = 3
                STATE_REACHED = 0
                start_tm = time.time()                     # log start time for timed operation
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                   
                set_signal(V_TRAVEL_T, 1)
                if STATE_REACHED == 1:
                    STOP_PB = 0	
                    SEQ_STATE = 0
                    STATE_REACHED = 0
                    pushFlag2 = False 
            else:
                set_signal(V_TRAVEL_T, 5)                  # check valve limits after travel time
            screen.blit(s2scale, (TPOSX, TPOSY))
        elif SEQ_STATE == 3:                               # wait for level to drop and return to filling		
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                
                SEQ_STATE = 0
            elif feed_flag == True:                        # request to open inlet and continously feed
                SEQ_STATE = 6
            elif inputs[5] == 0:                           # conductivity probe un-covered then stop pump
                outputs = [ instate, 1, 0 ]                
                SEQ_STATE = 4	
            elif (time.time() - start_tm) > EMPTY_TO:      # timeout in emptying tank
                STATE_REACHED = 3                          # set indicator
                if stop_pump_on_to == True:
                    outputs = [ instate, 1, 0 ]           
                    SEQ_STATE = 4	
                    STATE_REACHED = 0             
            screen.blit(s3scale, (TPOSX, TPOSY))
        elif SEQ_STATE == 4:                               # stop pump first	
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                
                SEQ_STATE = 0
            else:
                outputs = [ 0, 1, 0 ]  
                set_signal(V_TRAVEL_T, 6)
                if STATE_REACHED == 1:
                    SEQ_STATE = 5
                    STATE_REACHED = 0
            screen.blit(s3scale, (TPOSX, TPOSY))
        elif SEQ_STATE == 5:                               # close outlet	
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                
                SEQ_STATE = 0
            else:
                outputs = [ 0, 0, 0 ]  
                set_signal(V_TRAVEL_T, 7)
                if STATE_REACHED == 1:
                    SEQ_STATE = 0
                    STATE_REACHED = 0
            screen.blit(s2scale, (TPOSX, TPOSY)) 
        elif SEQ_STATE == 6:                               # open inlet and continuosly feed	
            if STOP_PB == 1:
                outputs = [ instate, 0, 0 ]                
                SEQ_STATE = 0
            elif feed_flag == False:                       # stop feeding and supply until empty
                SEQ_STATE = 2 
                step6_flg = True                
            else:
                outputs = [ 1, 1, inputs[5] ]  
                set_signal(V_TRAVEL_T, 8)
                if STATE_REACHED == 1:
                    STATE_REACHED = 0
            if inputs[5] == 1:
                screen.blit(s6scale, (TPOSX, TPOSY))
            else:
                screen.blit(s6lscale, (TPOSX, TPOSY))            
        put_signals()                                      # drive i.o
        if STATE_REACHED == 2:
            textimg1 = font.render(f"Sequence Error {FAIL_STEP}", True, pygame.Color("RED"))
        elif STATE_REACHED == 3:
            textimg1 = font.render("Timeout emptying tank", True, pygame.Color("RED"))
        elif STATE_REACHED == 1:
            textimg1 = font.render(f"Step {SEQ_STATE} complete", True, pygame.Color("RED"))
        elif STATE_REACHED == 0:
            textimg1 = font.render(f"Step {SEQ_STATE} waiting", True, pygame.Color("RED"))
        else:
            textimg1 = font.render("Sequence OK", True, pygame.Color("BLUE"))
        screen.blit(textimg1, (TPOSX, TPOSY+150))
        if inputs[5] == 0:
            textimg1 = font.render("Level Low", True, pygame.Color("RED"))
        else:
            textimg1 = font.render("Level OK", True, pygame.Color("BLUE"))
        screen.blit(textimg1, (TPOSX, TPOSY+250))
        if inputs[6] == 1:
            textimg1 = font.render("Level High", True, pygame.Color("RED"))
        else:
            textimg1 = font.render("Level OK", True, pygame.Color("BLUE"))
        screen.blit(textimg1, (TPOSX, TPOSY+350))
        if SEQ_STATE == 0 and START_PB == 0 and instate = 1:
            textimg1 = font.render("Filling", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 0 and START_PB == 0 and instate = 0:
            textimg1 = font.render("Filled to high level", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 0 and START_PB == 1:
            textimg1 = font.render("Closing inlet", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 1:
            textimg1 = font.render("Opening outlet", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 2:
            textimg1 = font.render("Starting pump", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 3:
            textimg1 = font.render("Waiting for low level", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 4:
            textimg1 = font.render("Stopping pump", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 5:
            textimg1 = font.render("Closing outlet", True, pygame.Color("YELLOW"))
        elif SEQ_STATE == 6:
            textimg1 = font.render("Continous Supply", True, pygame.Color("YELLOW"))
        else:
            textimg1 = font.render("Sequence Step Unknown", True, pygame.Color("BLUE"))
        screen.blit(textimg1, (TPOSX, TPOSY+450))
        if feed_flag == True:
            textimg1 = font.render("Continous Supply", True, pygame.Color("GREEN"))
        else:
            textimg1 = font.render("Tank Discharge", True, pygame.Color("GREEN"))
        screen.blit(textimg1, (TPOSX, TPOSY+500))
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
