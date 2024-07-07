#!/usr/bin/env python
# ref:- https://github.com/lethic/inspired_pro
# 
# The Inspired Pro Humanoid robot controls its leg movement via ROS commands
# and we have also connected a Sony IMX264LLR-C USB3
#
import roslib
roslib.load_manifest('inspired_pro')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

LEG_JOINTS = ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']

# example from https://www.argocorp.com/UVC_camera/Linux_TIScam_Python.html
import sys
import cv2
import numpy as np

import TIS
         
class Joint:
    def __init__(self, motor_name):
        self.name = motor_name
        self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.joint_names = ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']

    def add_point(self, angles, seconds):
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(seconds)
        self.goal.trajectory.points.append(point)

    def move_joint(self):
        print 'Points number: ', len(self.goal.trajectory.points)
        self.jta.send_goal_and_wait(self.goal)
        self.goal.trajectory.points[:] = []

def set_joint_value(joint_str, setval, val_list):
    for i, x in enumerate(LEG_JOINTS):
        if x == joint_str:
            val_list[i] = setval
            break
            
def main():
    # set up cameras
    Tis = TIS.TIS()

    #　DMK 33UX264 Serial: 16710581 、 640x480＠30 fps
    # Sony IMX264LLR-C USB3す
    Tis.openDevice("16710581", 640, 480, "30/1", TIS.SinkFormats.BGRA,True)

    #　DFK 33UX253 Serial: 45120952 、 1920x1080＠30 fps
    # 1/1.8" Sony CMOS Pregius IMX252 equipped with global shutter  
    # 2048×1536 @120fps 320 megapixel color or monochrome
    # Tis.open_device("45120952", 1920,1080, "30/1", TIS.SinkFormats.BGRA,True)

    # https://www.argocorp.com/cam/usb3/tis/common/pdf/trmdmk33ux264.en_US.pdf
    Tis.Set_Property("GainAuto","Off")                
    Tis.Set_Property("Gain", 9.6)
    Tis.Set_Property("ExposureAuto", "Off")           
    Tis.Set_Property("ExposureTime", 2977)
    Tis.Set_Property("BalanceWhiteAuto", "Off")       
    Tis.Set_Property("BalanceWhiteRed", 1.0)
    Tis.Set_Property("BalanceWhiteBlue", 1.0)
    Tis.Set_Property("BalanceWhiteGreen", 1.0)
    Tis.Set_Property("TriggerMode","Off")
    Tis.Set_Property("BlackLevel",270)
    Tis.Set_Property("GPOut",True)
    Tis.Set_Property("StrobeEnable","On")
    Tis.Set_Property("StrobePolarity","ActiveHigh")
    Tis.Set_Property("StrobeOperation","Exposure")
    Tis.Set_Property("StrobeDelay",10)
    Tis.Set_Property("Gamma",0.1)

    print("Gain Auto : %s " % Tis.Get_Property("GainAuto"))
    print("Gain : %d" % Tis.Get_Property("Gain"))
    print("Exposure Auto : %s " % Tis.Get_Property("ExposureAuto"))
    print("Exposure Time (us) : %d" % Tis.Get_Property("ExposureTime"))
    print("Whitebalance Auto : %s " % Tis.Get_Property("BalanceWhiteAuto"))
    print("Whitebalance Red : " ,Tis.Get_Property("BalanceWhiteRed"))
    print("Whitebalance Blue :  " ,Tis.Get_Property("BalanceWhiteBlue"))
    print("Whitebalance Green : " ,Tis.Get_Property("BalanceWhiteGreen"))
    print("BlackLevel : " ,Tis.Get_Property("BlackLevel"))
    print("Gamma : " ,Tis.Get_Property("Gamma"))

    Tis.Start_pipeline()  
    lastkey = 0
    cv2.namedWindow('Window') 
    kernel = np.ones((5, 5), np.uint8)  
    imagecounter=0
    
    leg = Joint('leg')                                    # connect to Leg on ROS
    flag = 1
    # ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']
    joint_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    POS_SIT = [0, 0.5, -0.5, 0, 0, -0.5, 0.5, 0, 0]                      # sit
    POS_STAND = [0, -0.5, 0.5, 0, 0, 0.5, -0.5, 0, 0]                    # stand
    POS_START = [0, -0.5, -0.1, 0, 0, -0.1, -0.5, 0, 0]                  # position to start FLIR pictures from

    #left shift
    M_L_S = [0.3, 0, 0, 0.3, 0.3, 0, 0, 0.3, 0]
    #right leg up
    M_R_U = [0, 0.2, 0, 0, 0, 0, 0, 0, 0]
    #right hip
    M_R_H = [-0.1, 0, 0, 0, 0, 0, 0, 0, 0]
    #right leg up again
    M_R_U2 = [0, 0.4, 0, 0, 0, 0, 0, 0, 0]
    #left shift again
    M_L_S2 = [0, 0, 0, 0, 0.1, 0, 0, 0.1, 0]
    #left thigh up
    M_L_U = [0, 0, 0, 0, 0, 0.4, 0, 0, 0]
    #right shank forward
    R_S_F = [0.1, 0, 0.4, 0, 0, 0, 0.3, 0, 0]
    #right leg straigt, left thigh up
    #R_L_S = [0, -0.6, -0.4, 0, 0, -0.4, -0.3, 0, 0]

    #ankles straight
    R_A_S = [-0.3, 0, 0, -0.3, -0.4, 0, 0.3, -0.4, 0]
    #here is[0, 0.1, 0.9, 0, 0, 0.9, 0.1, 0, 0]

    #shift right
    M_R_S = [-0.3, 0, 0, -0.3, -0.3, 0, 0, -0.3, 0]
    #right thigh forward
    R_T_F = [0, -1.0, 0, 0, 0, 0, 0, 0, 0]
    #left thigh up
    L_T_U = [0, 0, 0, 0, 0, -0.5, 0, 0, 0]
    L_H_U = [0, 0, 0, 0, 0.1, 0, 0, 0, 0]
    L_T_U2 = [0, 0, 0, 0, 0, -0.5, 0, 0, 0]
    #left shank forward
    L_S_F = [0, 0, 0, 0, 0, 0, -1.0, 0, 0]
    #right shank forward
    R_S_B = [0, 0, -1.0, 0, 0, 0, 0, 0, 0]
    #shift back
    S_F_B = [0.3, 0, 0, 0.3, 0.2, 0, 0, 0.3, 0]

    #right shift
    #F_R_S = [-0.3, 0, 0, -0.3, -0.3, 0, 0, -0.3, 0]
    #left leg up
    #F_L_U1 = [0, 0, 0, 0, 0, -0.7, -0.5, 0, 0]
    #F_L_U = [0, -0.5, 0.5, 0, 0, 0, 0, 0, 0]

    # initial position
    joint_pos = POS_STAND
    leg.add_point(joint_pos, 0.5)
    leg.move_joint()
    state = 1

    POS_TIME = 0.5

    while state >= 0:                                                    # until we exit with q key
        leg.jta.wait_for_result()
        if state == 1:                                                   # first trajectory of movements
            joint_pos = POS_STAND
            joint_pos = map(sum, zip(joint_pos, M_L_S))
            leg.add_point(joint_pos, POS_TIME)
            joint_pos = map(sum, zip(joint_pos, M_R_U))
            leg.add_point(joint_pos, POS_TIME *2)
            joint_pos = map(sum, zip(joint_pos, M_R_H))
            leg.add_point(joint_pos, POS_TIME *3)
            joint_pos = map(sum, zip(joint_pos, M_R_U2))
            leg.add_point(joint_pos, POS_TIME *4)
            joint_pos = map(sum, zip(joint_pos, M_L_S2))
            leg.add_point(joint_pos, POS_TIME *5)
            joint_pos = map(sum, zip(joint_pos, M_L_U))
            leg.add_point(joint_pos, POS_TIME *6)
            joint_pos = map(sum, zip(joint_pos, R_S_F))
            leg.add_point(joint_pos, POS_TIME *7)
            joint_pos = map(sum, zip(joint_pos, R_A_S))
            leg.add_point(joint_pos, POS_TIME *8)
            joint_pos = map(sum, zip(joint_pos, M_R_S))
            leg.add_point(joint_pos, POS_TIME * 9)
            joint_pos = map(sum, zip(joint_pos, R_T_F))
            leg.add_point(joint_pos, POS_TIME *10)
            joint_pos = map(sum, zip(joint_pos, L_T_U))
            leg.add_point(joint_pos, POS_TIME *11)
            joint_pos = map(sum, zip(joint_pos, L_H_U))
            leg.add_point(joint_pos, POS_TIME *12)
            joint_pos = map(sum, zip(joint_pos, L_T_U2))
            leg.add_point(joint_pos, POS_TIME *13)
            joint_pos = map(sum, zip(joint_pos, L_S_F))
            leg.add_point(joint_pos, POS_TIME *14)
            joint_pos = map(sum, zip(joint_pos, R_S_B))
            leg.add_point(joint_pos, POS_TIME *15)
            joint_pos = map(sum, zip(joint_pos, S_F_B))
            leg.add_point(joint_pos, POS_TIME *16)

            leg.move_joint()                                                   # do trajectory movement
            state = 2
            print('next state 0')
        elif state == 2:                                                       # second trajectory of movements
            joint_pos = POS_SIT
            val_list = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            set_joint_value('waist', 0.7, val_list)                            # waist
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME)
            set_joint_value('waist', 0.2, val_list)            
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *2)
            set_joint_value('waist', -0,7, val_list)            
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *3)
            set_joint_value('hip_r', -0,2, val_list)                             # left and right hips
            set_joint_value('hip_l', 0,2, val_list)             
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *4)
            joint_pos = POS_STAND
            set_joint_value('waist', 0.7, val_list)                            # waist
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *5)
            set_joint_value('waist', 0.2, val_list)            
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *6)
            set_joint_value('waist', -0,7, val_list)            
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *7)
            set_joint_value('hip_r', -0,2, val_list)                             # left and right hips
            set_joint_value('hip_l', 0,2, val_list)             
            joint_pos = map(sum, zip(joint_pos, val_list))
            leg.add_point(joint_pos, POS_TIME *8)
            
            leg.move_joint()                                                  # do motion
            state = 3
            print('finish state 0')
        elif state == 3:                                                      # enter the mode scanning with FLIR cameras rotating the waist at 3 stances
            val_list = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            angle_start= 0                                                    # starts in middle then rotates waist from limits to -limits
            angle_pos = 100                                                   # 100/100 = 1.0 max angle  
            a_step = 2                                                        # how much to move in each step

            start_positions = [POS_START, POS_STAND, POS_SIT]
            sel_position = 0
            print('Press Esc to stop')            
            while lastkey != 27:                                                     # until ESC comes from the keyboard
                for z in range(angle_start, (angle_pos+a_step), a_step):             # rotate waist and look at both camera outputs
                    joint_pos = start_positions[int(sel_position/2)]                 # defined start stance position for robot
                    set_joint_value('waist', z/100, val_list)                        # waist
                    joint_pos = map(sum, zip(joint_pos, val_list))
                    leg.add_point(joint_pos, POS_TIME)                               # add to the trajectory
                    leg.move_joint()                                                 # make the motion per trajectory
                    
                    if Tis.Snap_image(1) is True: 
                        image = Tis.Get_image() 
                        image = cv2.erode(image, kernel, iterations=5) 
                        cv2.imshow('Window', image) 

                        imagecounter += 1
                        filename = "./image{:04}.jpg".format(imagecounter)
                        cv2.imwrite(filename, image)
                       
                    lastkey = cv2.waitKey(10)
                        if lastkey == 27:                                             # operator hits ESC on keyboard we exit
                            break        

                if z >= angle_pos:                                                    # reached end change direction
                    angle_start = 100
                    angle_pos = -100                                                  # 100/100 = 1.0 max angle  
                    a_step = -2
                elif z <= angle_pos:                                                  # reached end change direction 
                    angle_start = -100
                    angle_pos = 100                                                   # 100/100 = 1.0 max angle  
                    a_step = 2
                sel_position = (sel_position + 1) %  6                                # increment to change stance posiiton
                
            state = 0                                                                 # stop & exit
            print('finish state 0')  
            cv2.destroyAllWindows() 
            Tis.Stop_pipeline()  
            joint_pos = POS_SIT                                                       # finally sit
            leg.add_point(joint_pos, 0.5)
            leg.move_joint()            
        else:
            state = 1
            print('finish state 1')

if __name__ == '__main__':
    rospy.init_node('inspired_leg_trajectory_controller')
    main()