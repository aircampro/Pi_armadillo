#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  An Example to Control a crazyflie drone using driver as shown here :- https://github.com/bitcraze/crazyflie-clients-python/tree/master
#
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

# max/min thrust values
CRAZY_MAX_THRUST = 4500
CRAZY_MIN_THRUST = 2500

# think this is in radians if not change to 360 degrees
import math
CRAZY_MAX_RPY = math.pi*2.0
CRAZY_MIN_RPY = -math.pi*2.0

# step sizes to up/dwn when key pressed
THRUST_STEP_SIZE = 1.0
RPY_STEP_SIZE = 0.1

msg = """
Control Your Crazyflie!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease thrust
a/d : increase/decrease roll

        u
   h         k
        n
u/n : increase/decrease pitch
h/k : increase/decrease yaw

1-9 change LED Color

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

# crazyflie libraries
import time

try:
    import zmq
except ImportError as e:
    raise Exception("ZMQ library probably not installed ({})".format(e))

# defualt messages in json
#

# control
cmdmess = {
    "version": 1,
    "ctrl": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "thrust": 30
    }
}

# LED's
zmess = {
    "version": 1,
    "rgbleds": [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]
}
    
def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def check_thrust(th):
    th = constrain(th, -CRAZY_MAX_THRUST, CRAZY_MAX_THRUST)
    return th

def check_rpy(val):
    val = constrain(val, -CRAZY_MAX_RPY, CRAZY_MAX_RPY)
    return val

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # connect to the crazyflie
    context = zmq.Context()
    sender = context.socket(zmq.PUSH)
    bind_addr = "tcp://127.0.0.1:{}".format(1024 + 188)
    sender.connect(bind_addr)

    # unlock thrust
    cmdmess["ctrl"]["thrust"] = 0.0
    sender.send_json(cmdmess)
    time.sleep(0.01)
                
    # unlock rpy
    cmdmess["ctrl"]["roll"] = 0.0
    cmdmess["ctrl"]["pitch"] = 0.0
    cmdmess["ctrl"]["yaw"] = 0.0
    sender.send_json(cmdmess)
    time.sleep(0.01)
                
    # control the pitch roll yaw and thrust from the keyboard keys
    #
    try:
        print(msg)
        while True:
            key = getKey()
            if key == 'w' :
                target_crazy_thrust = check_thrust(target_crazy_thrust + THRUST_STEP_SIZE)
                cmdmess["ctrl"]["thrust"] = target_crazy_thrust
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == 'x' :
                target_crazy_thrust = check_thrust(target_crazy_thrust - THRUST_STEP_SIZE)
                cmdmess["ctrl"]["thrust"] = target_crazy_thrust
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == 'a' :
                target_crazy_roll = check_rpy(target_crazy_roll + RPY_STEP_SIZE)
                cmdmess["ctrl"]["roll"] = target_crazy_roll
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == 'd' :
                target_crazy_roll = check_rpy(target_crazy_roll - RPY_STEP_SIZE)
                cmdmess["ctrl"]["roll"] = target_crazy_roll
                sender.send_json(cmdmess)
                time.sleep(0.01)
            if key == 'u' :
                target_crazy_pitch = check_rpy(target_crazy_pitch + RPY_STEP_SIZE)
                cmdmess["ctrl"]["pitch"] = target_crazy_pitch
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == 'n' :
                target_crazy_pitch = check_rpy(target_crazy_pitch - RPY_STEP_SIZE)
                cmdmess["ctrl"]["pitch"] = target_crazy_pitch
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == 'h' :
                target_crazy_yaw = check_rpy(target_crazy_yaw + RPY_STEP_SIZE)
                cmdmess["ctrl"]["yaw"] = target_crazy_yaw
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == 'k' :
                target_crazy_yaw = check_rpy(target_crazy_yaw - RPY_STEP_SIZE)
                cmdmess["ctrl"]["yaw"] = target_crazy_yaw
                sender.send_json(cmdmess)
                time.sleep(0.01)
            elif key == '1' :  
                for i in range(0, len(zmess['rgbleds'])):            
                    zmess["rgbleds"][i] = [128, 0, 128]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '2' :  
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [128, 0, 0]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '3' :       
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [255, 128, 0]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '4' :      
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [0, 100, 255]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '5' :  
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [0, 0, 128]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '6' :  
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [0, 128, 0]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '7' :   
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [255, 128, 255]
                    sender.send_json(zmess)
                    time.sleep(1.0)
            elif key == '8' :    
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [60, 10, 55]
                    sender.send_json(zmess)
                    time.sleep(0.01)
            elif key == '9' :   
                for i in range(0, len(zmess['rgbleds'])):                
                    zmess["rgbleds"][i] = [60, 200, 128]
                    sender.send_json(zmess)
                    time.sleep(0.1)
            elif key == ' ' or key == 's' :
               target_crazy_thrust = 0.0 
               target_crazy_pitch = 0.0 
               target_crazy_roll = 0.0 
               target_crazy_yaw = 0.0 
               cmdmess["ctrl"]["yaw"] = target_crazy_yaw
               cmdmess["ctrl"]["pitch"] = target_crazy_pitch  
               cmdmess["ctrl"]["roll"] = target_crazy_roll
               cmdmess["ctrl"]["thrust"] = target_crazy_thrust   
               sender.send_json(cmdmess)
               time.sleep(0.01)   
            elif key == 't' :
               target_crazy_yaw = 0.0 
               cmdmess["ctrl"]["yaw"] = target_crazy_yaw   
               sender.send_json(cmdmess)
               time.sleep(0.01)  
            elif key == 'g' :
               target_crazy_pitch = 0.0 
               cmdmess["ctrl"]["pitch"] = target_crazy_pitch    
               sender.send_json(cmdmess)
               time.sleep(0.01) 
            elif key == 'b' :
               target_crazy_roll = 0.0             
               cmdmess["ctrl"]["roll"] = target_crazy_roll  
               sender.send_json(cmdmess)
               time.sleep(0.01) 
            elif key == 'y':
               target_crazy_thrust = 0.0 
               cmdmess["ctrl"]["thrust"] = target_crazy_thrust    
               sender.send_json(cmdmess)
               time.sleep(0.01)                
            else:
                if (key == '\x03'):
                    break

    except:
        print(e)

    finally:
        cmdmess["ctrl"]["yaw"] = 0.0
        cmdmess["ctrl"]["pitch"] = 0.0  
        cmdmess["ctrl"]["roll"] = 0.0
        cmdmess["ctrl"]["thrust"] = 0.0    
        sender.send_json(cmdmess)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
