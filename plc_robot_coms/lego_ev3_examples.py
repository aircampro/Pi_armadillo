#!/usr/bin/env python3
# example using LEGO  M i n d s t o r m s EV3 library
#
# The way to program EV3 in Python is to write a file downloaded from the ev3dev to the 
# microD card and boot the EV3 from the microSD card
#
import ev3dev.ev3 as ev3
import time

# the robot speaks
from ev3dev2.sound import Sound

# color the LED's
from ev3dev2.led import Leds

# LCD display
from ev3dev2.display import Display

def smile_face():
    lcd.clear( )
    lcd.point( False , 45 , 45 )
    lcd.point( False , 135 , 45 )
    lcd.circle( False , 45 , 45 , 25 , fill_color = None )
    lcd.circle( False , 135 , 45 , 25 , fill_color = None )
    lcd.line(False, 25, 75, 45, 100, width=1)
    lcd.line( False, 45 , 100 , 135 , 100 , width = 1 )
    lcd.line(False, 135, 100, 155, 75, width=1)
    lcd.update( )

def curve_face():
    lcd.clear( )
    lcd.point( False , 45 , 45 )
    lcd.point( False , 135 , 45 )
    lcd.circle( False , 45 , 45 , 25 , fill_color = None )
    lcd.circle( False , 135 , 45 , 25 , fill_color = None )
    lcd.line(False, 15, 85, 45, 100, width=1)
    lcd.line( False, 45 , 100 , 135 , 100 , width = 1 )
    lcd.line(False, 135, 100, 165, 85, width=1)
    lcd.update( )
    
def sad_face():
    lcd.clear( )
    lcd.point( False , 45 , 45 )
    lcd.point( False , 135 , 45 )
    lcd.circle( False , 45 , 45 , 25 , fill_color = None )
    lcd.circle( False , 135 , 45 , 25 , fill_color = None )
    lcd.line(False, 25, 120, 45, 100, width=1)
    lcd.line( False, 45 , 100 , 135 , 100 , width = 1 )
    lcd.line(False, 135, 100, 155, 120, width=1)
    lcd.update( )

def flat_face():
    lcd.clear( )
    lcd.point( False, 45 , 45 )
    lcd.point( False, 135 , 45 )
    lcd.circle( False, 45 , 45 , 25 , fill_color = None )
    lcd.circle( False, 135 , 45 , 25 , fill_color = None )
    lcd.line( False, 45 , 100 , 135 , 100 , width = 1 )
    lcd.update( )

# speak
sound = Sound()
sound.speak('This is a demostration of the ev3 python commands')
    
#start and stop on time
n = ev3.LargeMotor('outB')                  # use B motor
n.run_forever(speed_sp=100)
time.sleep(2)
n.stop()

# now select other motor
m = ev3.LargeMotor('outA')                 # use A motor
m.reset()

#define touch sensor
ts = ev3.TouchSensor('in1')
t0 = time.time()
m.run_forever(speed_sp=500)
stopped=False
while time.time()-t0 < 30:                  # iterate for time
    if ts.value() == 1:                     # stop if ts pressed
        m.stop()
        stopped = True
if not stopped:                             # if not stopped stop on timeout
    m.stop()
	
# define an ultrasonic sensor and LED'screen
# color of the LED on the EV3 main unit changes.
leds = Leds()
us = ev3.UltrasonicSensor('in2')
leds.set_color("LEFT", "GREEN")
leds.set_color("RIGHT", "GREEN")
led_state = "green"
# Display co-orinates are :- x = 0-177 & y = 0-127
lcd = Display()
smile_face()

# control both motors from ultrasonic, stop with touch screen
#
while ts.value() == 0:                           # not touching screen
    if us.value() > 300:                         # high ultrasonic 
        m.run_forever(speed_sp=500)  
        n.run_forever(speed_sp=500) 
        if not led_state = "green":
            leds.set_color("LEFT", "GREEN")
            leds.set_color("RIGHT", "GREEN")
            led_state = "green"
            smile_face()
    elif us.value() > 250:
        m.run_forever(speed_sp=250) 
        n.run_forever(speed_sp=250) 
        if not led_state = "yellow":
            led.set_color("LEFT", "YELLOW")
            led.set_color("RIGHT", "YELLOW")
            led_state = "yellow"
            curve_face()
    elif us.value() > 100:
        m.run_forever(speed_sp=100) 
        n.run_forever(speed_sp=100) 
        if not led_state = "amber":
            led.set_color("LEFT", "AMBER")
            led.set_color("RIGHT", "AMBER")
            led_state = "amber"
            flat_face()
    else:                                        # low ultrasonic 
        m.run_forever(speed_sp=-600) 
        n.run_forever(speed_sp=-600)
        if not led_state = "orange":
            led.set_color("LEFT", "ORANGE")
            led.set_color("RIGHT", "ORANGE")
            led_state = "orange"
            sad_face()
    time.sleep(0.3)
m.stop()                                         # stop on touch screen was pressed stop
n.stop()
leds.set_color("LEFT", "RED")
leds.set_color("RIGHT", "RED")

sound.speak('This is the first demostration has now completed')
sound.speak('Now we are driving a large rectangle with 4 small rectangles around the corners')

# draw 2 squares with the robot
import math
from ev3dev2.motor import MoveTank,OUTPUT_B,OUTPUT_C   # use the move Tank Object
rbt=MoveTank(OUTPUT_B,OUTPUT_C)

sp=20                                      # Speed (%)
rt=0.54                                    # Number of revolutions required for 90° turn
lena=400                                   # Distance to advance in large rectangle (mm) center square 200
lenb=100                                   # distance to go forward in a small rectangle (mm)
tire=math.pi*56                            # circumference of tire (mm)
rsa=lena/tire                              # number of revolutions required to move forward in a large rectangle
rsb=lenb/tire                              # number of revolutions required to advance a small rectangle
for i in range(4):
    rbt.on_for_rotations(sp,sp,rsa)        # forward movement (no of revs for lena) in large rectangle
    rbt.on_for_rotations(-sp,sp,rt)        # turn left
    for j in range(2):
        rbt.on_for_rotations(sp,sp,rsb)    # forward movement (no of revs for lenb) in small rectangle
        rbt.on_for_rotations(-sp,sp,rt)    # turn left
    #rbt.on_for_rotations(sp,sp,rsb*2)     # if you want 400 mm square
sound.speak('This is the second demostration has now completed')
sound.speak('We are driving on the black tracks using a color sensor it gets steadily faster and if goes off trys to get back on the line')

# here we are moving along a black line of track using a color sensor
#
from time import time
from ev3dev2.sensor.lego import ColorSensor              # color sensor to see black line
from ev3dev2.motor import MoveTank,OUTPUT_B,OUTPUT_C     # outputs to wheels
from ev3dev2.sensor.lego import TouchSensor              # touchsensor
from PIL import Image                                    # for putting images on the screen

def display_image( lcd, imgnm ):
    lcd.clear()
    img=Image.open('/usr/share/images/ev3dev/mono/eyes/'+imgnm)
    lcd.image.paste(img,(0,0))

# define the modes for this craft
SPEED_INCREASE=0
SPEED_DECREASE=1
STOP_MOVING=2 
SCREEN_RSET=-99
   
ts=TouchSensor()                                         # make touch sensor 
cs=ColorSensor()                                         # use a color sensor to detect the black line for movement
lcd=Display()
th=55                                                    # light intensity threshold
rbt=MoveTank(OUTPUT_B,OUTPUT_C)                          # make tank object

sp_step=0.01                                             # speed increase %
sp_inc_time=0.2                                          # time of delay between speed increases [secs]
sp_default=30                                            # default reset speed for this craft
sp=sp_default                                            # speed (%)
rbt.on(sp,sp)                                            # forward movement first on line so go straight
speed_state = SPEED_INCREASE                             # the speed will increase with time 
di=SCREEN_RSET                                           # initialise flag to stop wriitng to display for the mode in continuos spped mode
while True:
    while speed_state < STOP_MOVING:                             # state is not to stop
        while ts.value() == 0:                                   # do forever........ if the screen is not pressed
            if cs.reflected_light_intensity >= th:               # not seeing the black line ?
                ti=0.0                                           # off the line and came to the white part
                sp=sp_default                                    # reset speed
                online=False                                     # not on the line flag
                dir=1                                            # First thing is to turn left and search
                display_image(lcd, 'dizzy.png')
                while online==False:
                    if dir==1:                                   # try left turn to get back on line                
                        rbt.on(-sp,sp)                           # turn left and search for line
                    else:
                        rbt.on(sp,-sp)                           # turn right and search for line
                    ti += 0.1;                                   # increase search time
                    if ti > 1.2:                                 # ti has reached max time
                        ti=0.2                                   # reset ti to 0.2 seconds
                        rbt.on(-sp,-sp)                          # Backtrack a bit.
                    ct=time();                                   # current time
                    while time()-ct < ti:                        # Repeat within search time, delay for ti seconds unless black is seen again
                        if cs.reflected_light_intensity < th:    # it now sees the black line
                            rbt.on(sp,sp)                        # on line, straight ahead
                            online=True
                            break                                # break out of repetition
                        elif not ts.value() == 0:                # pressed touchscreen
                            online=True                          # exit the loop if the touch is pressed
                            break                                
                    dir = -dir                                   # Swap turn directions
            else:
                if speed_state == SPEED_INCREASE:
                    sp += sp_step                                # increase the speed slightly
                    if not di == SPEED_INCREASE:                 # so as not to keep writing to display
                        display_image(lcd, 'crazy_2.png')
                        di = SPEED_INCREASE
                elif speed_state == SPEED_DECREASE:
                    sp -= sp_step                                # decrease the speed slightly  
                    if not di == SPEED_DECREASE:                    
                        display_image(lcd, 'evil.png') 
                        di = SPEED_DECREASE                        
                rbt.on(sp,sp)                                    # on line, straight ahead
                ti = sp_inc_time                                 # do it every sp_inc_time [secs]
                ct=time();                                       # current time
                while time()-ct < ti:                            # Repeat within search time, delay for ti seconds unless black is seen again
                    if cs.reflected_light_intensity >= th:       # it looses the black line                     
                        break                                    # break out of repetition  
                    elif not ts.value() == 0:                    # pressed touchscreen
                        break                                    # restart vehicle
                                                    
        display_image(lcd, 'crazy_1.png')                        # now moving forward at set speed while touchscreen is kept pressed                                                    
        ts.wait_for_released()                                   # wait for touchscreen release it will run at the fixed speed now
        if (speed_state == SPEED_INCREASE):                      # we were increasing speed
            speed_state = SPEED_DECREASE
        else:
            speed_state = STOP_MOVING
            
    rbt.on(0,0)                                              # stops vehicle
    display_image(lcd, 'black_eye.png')
    
    while ts.value() == 0:                                   # do forever........ if the screen is not pressed   
        if not ts.value() == 0:                              # pressed touchscreen
            break                                            # restart vehicle

    ts.wait_for_released()                                   # wait for touchscreen release
    
    speed_state = SPEED_INCREASE                             # reset to default speed and increasing mode
    sp=sp_default
    di=SCREEN_RSET
