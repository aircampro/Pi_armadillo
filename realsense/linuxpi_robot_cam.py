#!/usr/bin/env python
# -*- coding: utf-8 -*-

#===========================================================================================================================================================================
# Ubuntu-side program to control radio-controlled car and Intel realsense camera (e.g Intel RealSense L515 LiDAR camera) with DS4 remote control
# Ds4Socket2Twist.py Ver.1.1
# fan4fun2rc 5 Oct. 2022
# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Because the remote control that comes with radio-controlled cars has on/off control for forward, backward, left turn, and right turn, and it is difficult to handle,
# Substitute the inexpensive DS4 compatible remote control used with the PS4 game console as the ROS remote control.
# When using with Ubuntu, ds4drv + joy_node + teleop_node
# but this time, the environment we are using is Windows with WSL.
# Ubuntu18 running on WSL, so ds4drv is not available, so I used Windows blue tooth
# I am using a configuration that passes the switch information of the DS4 compatible device connected via WSL # to the ROS environment of Ubuntu18 via socket communication.
# I'm taking.
#
# [Specification].
# ・The connection between DS4 compatible device and Windows is made using Windows standard Bluetooth function.
# ・DS4 compatible devices recognized as HID devices by Ds4Pub2Socket.py running on Windows side
# - This software running on Ubuntu18 side using socket communication to receive switch change information from
# Converting to ROS speed indication topic /cmd_vel to control radio-controlled cars.
# ・Press DS4's □ button (data[5].bit4) and control back and forth driving with the left stick up/down (data[2]).
# Left stick left/right (data[1]) while holding down DS4's □ button (data[5].bit4) controls left/right turning.
# ・Turn on the turbo function with the R2 button (data[6].bit3) on DS4 and control the turbo amount with the R2 stick (data[9])
#[function summary] #[function summary] #[function summary] #[function summary]
#[Function Summary].
# 1. function to analyze data from DS4 compatible devices and deliver speed indication topics: ds4cmd(dat)
# 2. Main processing function: main()
#[Revision history
# [Revision history]
# ・V1.0-2022/10/03: Coding started
# -V1.1-2022/10/05: send() for socket communication on Windows side cannot be changed from protocol=3,
# -V2.0 2024/09/06 integrate with Intel Realsense camera
# Change to recv_bytes() on receiving side to communicate with protocol=2
#======================================================================================================================================================================

#------------------------------------------------------------------------------
#　Declaration of the library to import
#------------------------------------------------------------------------------
import rospy                                    # Python ROS
from geometry_msgs.msg import Twist             # Format for ROS running instruction messages/cmd_vel
from multiprocessing.connection import Listener # Used as a client for socket communication
import pickle                                   # Specify the format for socket communication

###############################################
##      Open CV and Numpy integration        ##
###############################################
#
import pyrealsense2 as rs
import numpy as np
import cv2

import signal

#------------------------------------------------------------------------------
#　Definition of constants and variables to be used in the module
#------------------------------------------------------------------------------
sub_add = ('localhost', 6000)   # Specifies port number for interprocess communication for socket communication

bit0 = 0b00000001               # Definition of bit constants
bit1 = 0b00000010
bit2 = 0b00000100
bit3 = 0b00001000
bit4 = 0b00010000
bit5 = 0b00100000
bit6 = 0b01000000
bit7 = 0b10000000

# set global for realsense streaming
REALSENSE_STR_ACTV = 0

# clean-up when kill -SIGUSR1 received
#
def handler(signum):
    print(f'handler signal : (signum={signum})')
    ds4cmd.pipe.stop()  
    sub_con.close()
    listener.close()  
    cv2.destroyAllWindows()
  
# function to estimate distance from the frames
#
def est_distance(frames, profile=ds4cmd.profile):
    # align captured frame
    aligned_frames = est_distance.align.process(frames)
    depth_frame =  aligned_frames.get_depth_frame()
    color_frame =  aligned_frames.get_color_frame()

    color_intr = rs.video_stream_profile(profile.get_stream(rs.stream.color)).get_intrinsics()

    # Estimation of 3D coordinates
    I_d = est_distance.result_frame.get_distance(685,642)
    point_I = rs.rs2_deproject_pixel_to_point(color_intr , [685,642], I_d)
    # Estimation of 3D coordinates
    R_d = est_distance.result_frame.get_distance(685,290)
    point_R = rs.rs2_deproject_pixel_to_point(color_intr , [685,290], R_d)
    # Calculate estimated distance
    est_range = math.sqrt((Point_I[0]-Point_R[0])*(Point_I[0]-Point_R[0]) + (Point_I[1]-Point_R[1])*(Point_I[1]-Point_R[1]) +(Point_I[2]-Point_R[2])*(Point_I[2]-Point_R[2]))
    print(" estimated range ", est_range)
    return est_range
    
#==================================================================================================================================================================
# 1. function to analyze data from DS4 compatible devices and deliver speed indication topic: ds4cmd(dat)
# <argument dat: array of switch information sent from DS4 >
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------
# ・Receive data from DS4 as a 16-byte array dat[].
# ・If DS4's R2 button (dat[6].bit3) is pressed, turn on turbo function and
# Multiply the R2 stick (dat[9]) by the turbo coefficient (ds4cmd.turbo) to determine the translation speed and rotation speed
# - If DS4's □ button (dat[5].bit4) is pressed,
# Determine the translation speed by multiplying the left stick up/down (dat[2]) information by the translation speed coefficient, and
# determine the rotation speed by multiplying the left stick left/right (data[1]) information by the rotation speed coefficient and distribute it.
# ・The R2 stick is normally 0 and increases to a maximum of 255 when the stick is pushed (pulled).
# ・ Left stick up is normally 128 and decreases to a minimum of 0 when the stick is pushed (pushed) upward and moves forward.
# The lower left stick is normally 128, and when the stick is pushed (pulled) down, it increases up to 255 and moves backward.
# Left stick left is normally 128 and decreases to minimum 0 and turns left when the stick is pushed to the left (down).
# Left stick right is normally 128, and when the stick is pushed to the right (down), it increases to a maximum of 255 and turns right.
# - If DS4's x button (dat[5].bit5) is pressed,
#   start streaming the realsense camera
# - If DS4 ○ Circle button (dat[5].bit6) is pressed,
#   stop streaming the realsense camera
# - DS4 : △ Triangle (dat[5].bit7) is pressed,
#   configure 640x480
# - DS4 : L1 (dat[6].bit0) is pressed,
#   configure 1280x720
# - DS4 R1 (dat[6].bit1) is pressed,
#   save to file is on
# - Share Key (dat[6].bit4) is pressed,
#   enable/disable distance estimation
# - Options key (dat[6].bit5) is pressed,
#   toggle aspect ratios
#
#====================================================================================================================================================================
def ds4cmd(dat):
    #
    # If the turbo function is turned on, the translational and rotational speeds are multiplied by the turbo factor.
    #
    if dat[6] & bit3:
        ds4cmd.vel.linear.x = (128-dat[2])/128.0 * ds4cmd.max_x \
                            * (1 + dat[9]/255.0 * (ds4cmd.turbo-1))
        ds4cmd.vel.angular.z = (128-dat[1])/128.0 * ds4cmd.max_z \
                            * (1 + dat[9]/255.0 * (ds4cmd.turbo-1))
        #
        # Deliver calculated translational and rotational speeds as ROS topics
        #
        ds4cmd.pub.publish(ds4cmd.vel)
        # print(ds4cmd.vel)
        # print(“Turbo: %d, up/down: %d, left/right: %d” % (dat[9], dat[2], dat[1]))
        a=ds4cmd.state_str.split("_")
        if (a >= 2):
            ds4cmd.state_str = "T" + "_" + a[1]
        else:
            ds4cmd.state_str="T"
    #
    # If the turbo function is off and in normal mode, the translational and rotational speeds are multiplied by the maximum factor.
    #
    elif dat[5] & bit4:
        ds4cmd.vel.linear.x = (128-dat[2])/128.0 * ds4cmd.max_x
        ds4cmd.vel.angular.z = (128-dat[1])/128.0 * ds4cmd.max_z
        #
        # Deliver calculated translational and rotational speeds as ROS topics
        #
        ds4cmd.pub.publish(ds4cmd.vel)
        # print(ds4cmd.vel)
        # print(“Normal: %d, up/down: %d, left/right: %d” % (dat[9], dat[2], dat[1]))
        a=ds4cmd.state_str.split("_")
        if (a >= 2):
            ds4cmd.state_str = "N" + "_" + a[1]
        else:
            ds4cmd.state_str="N"
    #
    # start realsense streaming
    #
    elif dat[5] & bit5:
        ds4cmd.profile = ds4cmd.pipe.start(ds4cmd.conf)
        global REALSENSE_STR_ACTV
        REALSENSE_STR_ACTV = 1
    #
    # start realsense streaming
    #
    elif dat[5] & bit6:
        ds4cmd.pipe.stop()
        global REALSENSE_STR_ACTV
        REALSENSE_STR_ACTV = 0
    #
    # 640x480
    #
    elif dat[5] & bit7:
        ds4cmd.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        ds4cmd.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #
    # 1280x720
    #
    elif dat[6] & bit0:
        ds4cmd.conf.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        ds4cmd.conf.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)   
    #
    # save to file on
    #
    elif dat[6] & bit1 :
        if REALSENSE_STR_ACTV == 0:
            ds4cmd.conf.enable_record_to_file('export_filename.bag')
            ds4cmd.state_str=ds4cmd.state_str+"_F"
        else:
            a = ds4cmd.state_str.split("_")
            if (a < 2):            
                ds4cmd.state_str=ds4cmd.state_str+"_StopStream2SaveFile"        
    #
    # toggle distance estimation
    #
    elif dat[6] & bit4:
        if ds4cmd.d_est_actv == 0:
            ds4cmd.d_est_actv = 1
        else:
            ds4cmd.d_est_actv = 0
    #
    # toggle aspect ratios
    #
    elif dat[6] & bit5:
        if ds4cmd.as_select == 0:          # normal
            ds4cmd.as_select = 2           # portrait
        elif ds4cmd.as_select == 2:        # portrait
            ds4cmd.as_select = 0.5         # landscape
        else:
            ds4cmd.as_select = 0           # normal
            
#==============================================================================
# 2. main processing function: main()
#------------------------------------------------------------------------------
# ・Receive DS4 compatible machine change information sent by inter-process communication from Windows side,
# Converts it into ROS speed indication information and distributes it via topic/cmd_vel.
#==============================================================================
def main():
    # define a clean-up on kill -SIGUSR1 <pid>
    #
    signal.signal(signal.SIGUSR1, handler)
    #
    # Variable definition and initialization in a function
    #
    ds4cmd.vel = Twist()            # ROS Twist object
    ds4cmd.vel.linear.x = 0.0       # initialize translation speed, positive for forward, negative for backward
    ds4cmd.vel.angular.z = 0.0      # initialize rotation speed, positive for left turn, negative for right turn
    #--------------------------------------------------------------------------
    # Create and run ROS node [brdg_ds4].
    #--------------------------------------------------------------------------
    rospy.init_node('brdg_ds4')
    rospy.loginfo("ROS node [brdg_ds4] is started.")
    #
    # Set the maximum translational speed [m/s] and rotational speed [rad/s] and the coefficient at turbo with parameters
    #
    ds4cmd.max_x = rospy.get_param('~scale_linear', 1.0)    # m/sec
    ds4cmd.max_z = rospy.get_param('~scale_angular', 1.0)   # rad/sec
    ds4cmd.turbo = rospy.get_param('~scale_turbo', 1.5)     # Units are coefficients
    #
    # Define the creation of topics to be delivered
    #
    # drive the robot
    #
    ds4cmd.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # realsense camera
    # config (default)
    #
    ds4cmd.conf = rs.config()
    ds4cmd.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    ds4cmd.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # define the stream
    #
    ds4cmd.pipe = rs.pipeline()
    # --- set-up filters for distance estimation ---
    #
    # distance estimate eanbled=1 disabled=0
    ds4cmd.d_est_actv = 0
    # decimarion_filter
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 1)
    # spatial_filter
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 1)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    # hole_filling_filter
    hole_filling = rs.hole_filling_filter()
    # disparity
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)
    # filter
    filter_frame = decimate.process(depth_frame)
    filter_frame = depth_to_disparity.process(filter_frame)
    filter_frame = spatial.process(filter_frame)
    filter_frame = disparity_to_depth.process(filter_frame)
    filter_frame = hole_filling.process(filter_frame)
    est_distance.result_frame = filter_frame.as_depth_frame()
    # align
    align_to = rs.stream.color
    est_distance.align = rs.align(align_to)
    # screen state string either T=turbo or N=normal
    ds4cmd.state_str="N"
    # aspect ratio as selected
    ds4cmd.as_select=0
    # get remote control operation information from Windows side
    # Act as a server from Windows side to get remote control operation information.
    # 
    while True:
        listener = Listener(sub_add, authkey=b'secret password')
        print("Start the server process for DS4 remote control...")
        sub_con = listener.accept()
        print("Initiate a connection with the client...")
        while True:
            try:
                dat = sub_con.recv_bytes()
                # print("received data length of ：%d from windows DS4 controller " % len(dat))
                val = pickle.loads(dat)
                # print('RCV:' + str(val))
                ds4cmd(val)     # look at the data received and peform the actions 
            except Exception as e:
                print("Terminate connection with client： %s" % e)
                break
            try:
                if REALSENSE_STR_ACTV == 1:
                    # Wait for a coherent pair of frames: depth and color
                    frames = ds4cmd.pipe.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                    if not depth_frame or not color_frame:                             # show a blank screen with information
                        path = "blank_screen.jpeg"
                        images = cv2.imread(path)
                        cv2.putText(img,"No color or depth frame found",org=(200, 50),fontFace=cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(0, 255, 0),thickness=2, lineType=cv2.LINE_AA)
                    else:
                        # Convert images to numpy arrays
                        depth_image = np.asanyarray(depth_frame.get_data())
                        color_image = np.asanyarray(color_frame.get_data())

                        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                        # Stack both images horizontally
                        images = np.hstack((color_image, depth_colormap))
                        # if distance estimation is active then calcualte the LIDAR distance 
                        if ds4cmd.d_est_actv == 1:
                            dist_est = est_distance(frames, ds4cmd.profile)
                            text_msg = ds4cmd.state_str+" distance estimate : "+str(dist_est)
                        else:
                            text_msg = ds4cmd.state_str
                        cv.putText(images, text_msg, (0, 50), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 5, cv.LINE_AA)                        
                    # Show images
                    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('RealSense', images, aspect=ds4cmd.as_select)
                    cv2.waitKey(1)
                    if key == 27:
                        break
                else:
                    cv2.destroyAllWindows()
            except Exception as e:
                print("Exception ： %s" % e)                
            time.sleep(0.05)
        sub_con.close()
        print('Accept:closed...')
        listener.close()
        print('Linstener:closed...')
        if key == 27:
            break
    # break
    cv2.destroyAllWindows()

#------------------------------------------------------------------------------
# main loop
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()