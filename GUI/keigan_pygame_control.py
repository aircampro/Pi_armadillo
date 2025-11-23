#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Control 2 keigan motors on 2 wheel drive craft from a pygame keyboard controller and screen GUI example with click button actions
#
# usage : <prog> address_left address_right (default cart speed) (default delta)
#
# ref:- https://github.com/keigan-motor/pykeigan_motor/blob/master/pykeigan/blecontroller.py
#
# BLEController(aadr) we will get.
# read_motor_measurement {'position': position, 'velocity': velocity, 'torque': torque, 'received_unix_time': time.time()}
# read_imu_measurement {'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z, 'temp': temp, 'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z, 'received_unix_time': time.time()}
#
import sys
import pygame
# import the 2 wheel drive object
import ../plc_robot_comms/keigan_motor_2wheeldrive

# define the screen size
WIDTH = 1270
HEIGHT = 800
# text (data) positions on the screen
TPOSX=200
TPOSY=100
TSPACER=50

LEFT_CLK=0
MID_CLK=1
RIGHT_CLK=2

# init screen and controls 
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(" Control the Cart ")
clock  = pygame.time.Clock()
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
CRAD = 25
BX=TPOSX
BY=HEIGHT-100                                                         # buttons @ bottom of the screen
font = pygame.font.Font(None, 50)

# e.g. to specify on windows msgothic  font = pygame.font.Font("C:/Windows/Fonts/msgothic.ttc", 50)

# load and show a png object on your screen this will move with the mouse, also make flipped image
img_path = Path(__file__).parent / "images" / "ur_image.png" 
inu = pygame.image.load(str(img_path))
inu_imgR = pygame.transform.scale(inu, (50, 50))
inu_imgL = pygame.transform.flip(inu_imgR, True, False)

# screen.blit(inu, (mx-100, my-100))

# init the 2 wheel cart object communication over ble (addresses as passed as command line)
aadr1=sys.argv[1]
aadr2=sys.argv[2]
cart = keigan_motor_2wheeldrive.TWD(aadr1, aadr2, mode="ble")
cart.enable()
if len(sys.argv[0]) == 3:
    CART_SPD = float(sys.argv[3])
    if len(sys.argv[0]) == 4:
        SPD_DELTA=float(sys.argv[4])    
else:
    CART_SPD=10
    SPD_DELTA=0.1

# initialise a player sprite to represent the cart movement on the screen (relative)
show_sprite=False
player = pygame.Rect(300, 220, 40, 40)
speed  = 5
player.x = speed   
player.y = speed
running = True

# this is if you want a button to press on the screen to do something once centre mouse wheel clicked
img_path = Path(__file__).parent / "images" / "stop_btn.png" 
btn_img = pygame.image.load(str(img_path)) 
# draw the click button to stop the cart
btn = screen.blit(btn_img, (BX, BY))

img_path = Path(__file__).parent / "images" / "rec_btn.png" 
rec_img = pygame.image.load(str(img_path)) 

img_path = Path(__file__).parent / "images" / "end_rec_btn.png" 
end_rec_img = pygame.image.load(str(img_path)) 

img_path = Path(__file__).parent / "images" / "play_btn.png" 
play_img = pygame.image.load(str(img_path)) 

img_path = Path(__file__).parent / "images" / "stop_play_btn.png" 
stop_play_img = pygame.image.load(str(img_path)) 

# draw the click buttons for the action record and replay
rec = screen.blit(rec_img, (BX+50, BY))
end_rec = screen.blit(end_rec_img, (BX+100, BY))
play = screen.blit(play_img, (BX+150, BY))
stp_play = screen.blit(stop_play_img, (BX+200, BY))

# run the controls for reading keyboard and mouse and draw info screen with action choice push buttons
def main_run():
    global CART_SPD
    global running
    pushFlag1 = False
    pushFlag2 = False 
    pushFlag3 = False 
    pushFlag4 = False 
    pushFlag5 = False
    right_latch = False
    left_latch = False
    (mx, my) = pygame.mouse.get_pos()
    act_txt = "free movement"    
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.MOUSEBUTTONDOWN:  
                print("mouse button down！")
                pygame.draw.circle(screen, pygame.Color("RED"),[mx, my], CRAD))
            elif e.type == pygame.MOUSEBUTTONUP:  
                print("mouse button up！")
                pygame.draw.circle(screen, pygame.Color("GREEN"),[mx, my], CRAD))

        # ----- keyboard controls -----
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT] and not left_latch:  
            player.x -= speed
            cart.pivot_turn(CART_SPD, -90)
            act_txt = "turn left" 
            right_latch = False
            left_latch = True
        elif keys[pygame.K_RIGHT] and not right_latch: 
            player.x += speed
            cart.pivot_turn(CART_SPD, 90)
            act_txt = "turn right" 
            right_latch = True
            left_latch = False
        elif keys[pygame.K_UP]:    
            player.y -= speed
            CART_SPD -= SPD_DELTA
            cart.run(CART_SPD)
            act_txt = "increase speed" 
            right_latch = False
            left_latch = False
        elif keys[pygame.K_DOWN]:  
            player.y += speed
            CART_SPD += SPD_DELTA
            cart.run(CART_SPD)
            act_txt = "decrease speed" 
            right_latch = False
            left_latch = False
        else:
            right_latch = False
            left_latch = False

        # --------- mouse controls ---------
        mdown = pygame.mouse.get_pressed()
        (mx, my) = pygame.mouse.get_pos()
        # mdown[0]	Left-click        (draws image)
        # mdown[1]	Middle click      (wheel)
        # mdown[2]	Right-click       (draws flipped image)
        if mdown[LEFT_CLK]:
            screen.blit(inu_imgR, (mx, my))
            pushFlag1 = False
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False 
            pushFlag5 = False             
        elif mdown[RIGHT_CLK]:
            screen.blit(inu_imgL, (mx, my))
            pushFlag1 = False  
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False 
            pushFlag5 = False 
        elif mdown[MID_CLK]:                                                                  # middle wheel down activates the screen button selection
            print("wheel pressed checking to see if it was the button")
            if btn.collidepoint(mx, my) and pushFlag1 == False: 
                print("stop button was pressed with wheel on mouse")
                cart.stop()
                pushFlag1 = True 
                act_txt = "stop"                
            elif rec.collidepoint(mx, my) and pushFlag2 == False and not act_txt == "replay recorded movement" : 
                print("record button was pressed with wheel on mouse")
                cart.start_recording_taskset(1)
                pushFlag2 = True  
                act_txt = "recording movement"  
            elif end_rec.collidepoint(mx, my) and pushFlag3 == False and act_txt == "recording movement": 
                print("end record button was pressed with wheel on mouse")
                cart.stop_recording_taskset(1)
                pushFlag3 = True 
                act_txt = "stop recording movement"                 
            elif play.collidepoint(mx, my) and pushFlag4 == False and not act_txt == "recording movement": 
                print("replay button was pressed with wheel on mouse")
                cart.start_doing_taskset(1)
                pushFlag4 = True 
                act_txt = "replay recorded movement"                 
            elif stp_play.collidepoint(mx, my) and pushFlag5 == False and act_txt == "replay recorded movement": 
                print("replay button was pressed with wheel on mouse")
                cart.stop_doing_taskset()
                pushFlag5 = True 
                act_txt = "end play of recorded movement"                 
        else:
            pushFlag1 = False  
            pushFlag2 = False 
            pushFlag3 = False 
            pushFlag4 = False 
            pushFlag5 = False 

        # get new cart data and show on display
        l_motor_meas = cart.left.read_motor_measurement()
        l_imu_meas = cart.left.read_imu_measurement()
        r_motor_meas = cart.right.read_motor_measurement()
        r_imu_meas = cart.right.read_imu_measurement()
        textimg1 = font.render(f"① {l_motor_meas}", True, pygame.Color("BLUE"))
        textimg2 = font.render(f"② {l_imu_meas}", True, pygame.Color("BLUE"))
        textimg3 = font.render(f"③ {r_motor_meas}", True, pygame.Color("GREEN"))
        textimg4 = font.render(f"④ {r_imu_meas}", True, pygame.Color("GREEN"))
        textimg5 = font.render(f"⑤ speed : {CART_SPD} action: {act_txt}", True, pygame.Color("RED"))
        #textimg6 = font.render(f"⑥ spare line... add if you have more data here ", True, pygame.Color("RED"))
        screen.blit(textimg1, (TPOSX, TPOSY))
        screen.blit(textimg2, (TPOSX, TPOSY+SPACER))
        screen.blit(textimg3, (TPOSX, TPOSY+(2*SPACER)))
        screen.blit(textimg4, (TPOSX, TPOSY+(3*SPACER)))
        screen.blit(textimg5, (TPOSX, TPOSY+(4*SPACER)))
        #screen.blit(textimg6, (TPOSX, TPOSY+(5*SPACER)))
        pygame.display.update()

        if show_sprite:                                                   # if you want to draw a sprite you will have to make a screen boundary from the text info for this to move in   
            screen.fill((30, 30, 30))
            pygame.draw.rect(screen, (0, 200, 255), player)               # draw the sprite showing speed and turn
            pygame.display.flip()
        clock.tick(60)                                                    # refresh @ 60hz
    cart.disable()
    pygame.quit()

if __name__ == "__main__":
    main_run()