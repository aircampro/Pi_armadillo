#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==========================================================================================================================================================================
# Windows program to control radio-controlled cars with DS4 remote control
# Ds4Pub2Socket.py Ver.1.1
# fan4fun2rc 5 Oct. 2022
# modified 09/2024
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Because the remote control that comes with RC cars has on/off control for forward, backward, left turn and right turn and it's hard to handle,
# Substitute the inexpensive DS4 compatible remote control used with the PS4 game console as the ROS remote control.
# If you use it on Ubuntu, you can connect it by using ds4drv + joy_node + teleop_node.
# but this time, the environment we are using is Windows with WSL.
# Ubuntu18 running on WSL, so ds4drv is not available, so I used Windows blue tooth
# I am using a configuration that passes the switch information of the DS4 compatible device connected via WSL # to the ROS environment of Ubuntu18 via socket communication.
# I'm taking.
#
# [Specification].
# ・Connection between DS4 compatible device and Windows is made using Windows standard Blue tooth function.
# ・Switch change information from DS4 compatible devices recognized as HID devices using socket communication.
# Ds4Socket2Twist.py running on Ubuntu18 side to ROS speed indication topic /cmd_vel
# Convert and control radio-controlled cars.
# [Function Summary] #[Function Summary] #[Function Summary] #[Function Summary]
# 【Outline of functions
# 1. main processing function: main()
# 2.
# [Revision history] #
# ・V1.0-2022/10/03: Coding started
# ・V1.1-2022/10/05: send() for socket communication cannot be changed from protocol=3,
# Change to send_bytes() due to error in recv() in Python2 on receiving side.
#===================================================================================================================================================================================

#------------------------------------------------------------------------------
#　libraries
#------------------------------------------------------------------------------
import hid                                    # For HID device control
import copy                                   # make copy
from multiprocessing.connection import Client # Used as a server for socket communication
import pickle

#------------------------------------------------------------------------------
#　Definition of constants and variables
#------------------------------------------------------------------------------
VENDOR_ID = 1356                # DS4 vendorーID
PRODUCT_ID = 2508               # DS4 product-ID
pub_add = ('localhost', 6000)   # Specifies port number for interprocess communication for socket communication

# zero when in the deadzone i.e @center
#
def deadzones(values):
    deadzone = 0.14
    if math.sqrt( values[1] ** 2  + values[2] ** 2) < deadzone:
        values[2] = 0.0
        values[1] = 0.0
    return values
        
#==============================================================================
# ：main()
#------------------------------------------------------------------------------
#　・send command from DS4 to control the robot and the realsense camera
#==============================================================================
def main():
    try:
        #
        # DS4 connect
        #
        ds4 = hid.device()
        ds4.open(VENDOR_ID, PRODUCT_ID)
        preval = []

        #
        # Create inter-process communication with Ubuntu18 as a client process
        #
        with Client(pub_add, authkey=b'secret password') as pub_con:
            print("connected to razpai．．．")
            while True:
                val = ds4.read(16)          # Reads 16 bytes of data from DS4 compatible devices
                val = deadzones(val)        # deadzone is the stick in center
                if preval != val:           # value has changed
                    preval = copy.copy(val)
                    # print(val)
                    msg = pickle.dumps(val, protocol=2)
                    pub_con.send_bytes(msg)

    except Exception as e:
        print("exception as ： ", e)

    ds4.close()

#------------------------------------------------------------------------------
# main loop
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()