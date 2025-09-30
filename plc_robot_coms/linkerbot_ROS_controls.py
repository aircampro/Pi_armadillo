#!/usr/bin/env python3
#
# LinkerBot Hand SDK Examples
#
# ref SDK :- https://github.com/linkerbotai/linker_hand_sdk/blob/main/examples/set_linker_hand_torque/scripts/set_linker_hand_torque.py
#
import rospy,json
from std_msgs.msg import String
import time
import rospy,json

from sensor_msgs.msg import JointState
import std_msgs.msg

import signal
import sys

# step variables for hand finger states
#
show_count=0
show_count_obj=0
show_step=0

joint_state = JointState() 
hand = { "joint1":255,   # curved thumb base
         "joint2":128,   # thumb
         "joint3":255,   # Finger root curvature  
         "joint4":255,   # curvature of the base of the middle finger
         "joint5":255,   # No name finger root curve
         "joint6":255,   # curvature of the base of the little finger
         "joint7":255,   # thumb rotation

class ColorMsg():
    def __init__(self,msg: str,color: str = '', timestamp: bool = True) -> None:
        self.msg = msg
        self.color = color
        self.timestamp = timestamp
        self.colorMsg(msg=self.msg, color=self.color, timestamp=self.timestamp)

    def colorMsg(self,msg: str, color: str = '', timestamp: bool = True):
        str = ""
        if timestamp:
            str += time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + "  "
        if color == "red":
            str += "\033[1;31;40m"
        elif color == "green":
            str += "\033[1;32;40m"
        elif color == "yellow":
            str += "\033[1;33;40m"
        elif color == "magenta":
            str += "\033[35m"
        elif color == "blue":
            str += "\033[36m"
        else:
            print(str + msg)
            return
        str += msg + "\033[0m"
        print(str)

# a set of movements related to a hand
#
def do_sequence():
    global show_count
    global show_count_obj
    global show_step
    global hand
    show_count=show_count+1
    if(show_count>=show_count_obj):
        show_count=0
        if(show_step==0):                # open hand pose
            show_step=show_step+1
            show_count_obj = 100
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==1):              #// Little finger and anonymous finger
            show_step=show_step+1
            show_count_obj = 10
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint5'] = 0
            hand['joint6'] = 0
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==2):              #// Top of thumb, little finger, and no name finger
            show_step=show_step+8
            show_count_obj = 30
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint7'] = 80
            return list(hand.values())
        elif(show_step==3):              #// A side-by-side slant
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==4):              #// one side
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==5):              #//  Two parts in progress
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==6):             #// Eating finger Japanese middle finger Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==7):             
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==8):             #// Eating finger Japanese middle finger Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==9): #// 收Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==10):            #// Middle Finger Japanese Food Finger Curved, Curved, Straight, and Straight Alternating
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==11):            #// Middle Finger Japanese Food Finger Curved, Curved, Straight, and Straight Alternating
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==12):             #// Middle Finger Japanese Food Finger Curved, Curved, Straight, and Straight Alternating
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==13):             #// Middle Finger Japanese Food Finger Curved, Curved, Straight, and Straight Alternating
            show_step=show_step+1
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==14):            #// bent thumb
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint7'] = 120
            return list(hand.values())
        elif(show_step==15):            #// thumb in the palm of your hand
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==16):            #// 4 fingers
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 10
            hand['joint4'] = 10
            hand['joint5'] = 10
            hand['joint6'] = 10
            return list(hand.values())
        elif(show_step==17):            #// open 4 fingers Japanese thumb
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==18): #// 1
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==19): #// 2
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==20): #// 3
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            return list(hand.values())
        elif(show_step==21): #// 40
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 110
            hand['joint7'] = 240
            return list(hand.values())
        elif(show_step==22):           #// thumbnail
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 10
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==23):          #// Reverse thumb finger palm heart
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            hand['joint2'] = 10
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==24):         #// The first time you arrive at the starting position
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 0
            hand['joint2'] = 240
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==25): #// 1
            show_step=show_step+4
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==26): #// 2
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==27): #// 3
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==28): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==29): #// 4 Pinky Finger
            show_step=show_step+4
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==30): #// 4 fingers bent
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==31): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==32): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==33): #// Pinky Finger
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 0
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==34):       #// Pinky Finger 
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 0
            return list(hand.values())
        elif(show_step==35):       #// Pinky Finger 
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 0
            return list(hand.values())
        elif(show_step==36):      #// Pinky Finger 
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 0
            return list(hand.values())
        elif(show_step==37):     #// bent thumb
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            return list(hand.values())
        elif(show_step==38): #// pinky finger
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 250
            hand['joint2'] = 230
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==39): 
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 250
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==40): #// Thumb finger up 666
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 10
            hand['joint2'] = 40
            hand['joint7'] = 60
            return list(hand.values())
        elif(show_step==41): #// left and right fingers
            show_step=show_step+5
            show_count_obj = 5
            hand['joint1'] = 50
            return list(hand.values())
        elif(show_step==42): #// left and right fingers
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==43): #// left and right fingers
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==44): #// left and right fingers
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==45): #// left and right fingers
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==46): #//  Expand
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==47): #// Thumb Japanese food fingering
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 120 #155
            hand['joint2'] = 130 #130
            hand['joint3'] = 155 #158
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 90
            return list(hand.values())
        elif(show_step==48): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint3'] = 250
            return list(hand.values())
        elif(show_step==49): #// Thumb and middle finger kneading
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 120
            hand['joint4'] = 140
            hand['joint7'] = 60
            return list(hand.values())
        elif(show_step==50): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==51): #// Thumb and name without name
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 120
            hand['joint2'] = 125
            hand['joint5'] = 145
            hand['joint7'] = 40
            return list(hand.values())
        elif(show_step==52): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==53): #// Thumb and little finger kneading
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 120
            hand['joint6'] = 135
            hand['joint7'] = 15
            return list(hand.values())
        elif(show_step==54): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        else:
            show_step=0

# control either or both hands
#
def control_hand(hnd):
    global show_step
    rospy.init_node('dong_test_sender', anonymous=True)
    if hnd == "right":
        pub = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=10)
    elif hnd == "left":
        pub = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)	
    elif hnd == "both":
        pub_l = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)
        pub_r = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=10)
    else:
        warn = ColorMsg() 
		warn.colorMsg(msg="you must pass either left or right pr both to choose the hand", color="red")
        return 
    rate = rospy.Rate(30)                                                                        # Setting frequency 30Hz
    joint_state.header = std_msgs.msg.Header()
    joint_state.header.seq=0
    joint_state.header.stamp = rospy.Time.now() 
    joint_state.header.frame_id = ''
    joint_state.name=list(hand.keys())
    joint_state.velocity = [0] * len(joint_state.position)  
    joint_state.effort = [0] * len(joint_state.position)  
    if hnd == "both":
        pub_l.publish(joint_state)	
        pub_r.publish(joint_state)	
    else:
        pub.publish(joint_state)

    while not rospy.is_shutdown():  
        if hnd == "both":
            position = do_sequence()                           # every other step on each hand (otherwise make 2 sequences for same action on both hands)
            if(position is not None):
                joint_state.position = position
            pub_l.publish(joint_state)
            position = do_sequence()                           # every other step on each hand (otherwise make 2 sequences for same action)
            if(position is not None):
                joint_state.position = position
            pub_r.publish(joint_state)
        else:
            position = do_sequence()
            if(position is not None):
                joint_state.position = position
            pub.publish(joint_state)
        if show_step==54:
            show_step=0
        rate.sleep()

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0) 

if __name__ == '__main__':
    '''
    L7:
    rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=right _speed:="[255,255,255,255,255,255,255]"

    other:
    rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=right _speed:="[100,100,100,100,100]"
    '''
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('get_linker_hand_speed', anonymous=True)
    hand_type = rospy.get_param("~hand_type",default="left")                        # get the hand type from ROS command (left or right)
    speed = rospy.get_param('~speed', default=[255,255,255,255,255])                # get the speed for each finger
    delta_s = 1                                                                     # how much to increase the speed by
    no_loops = 2                                                                    # number of repeat loops
    chosen_hand="left"                                                              # chosing left right or both to perform the sequence of operations
    cm = ColorMsg()                                                                 # color wrapper object to print

    pub = rospy.Publisher("/cb_hand_setting_cmd", String, queue_size=10)            # publisher to change the speed
    msg = String()  
    count = 0   
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        speed2 = []
        for s in speed:
            s += delta_s                                                             # add the delta to the speed
            speed2,append(s)
        dic = {
            "setting_cmd":"set_speed",
            "params":{
                "hand_type": hand_type,
                "speed":speed2
            }
        }
        msg.data = json.dumps(dic)
        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("changed speed to :%s",msg.data)
        cm.colorMsg(msg=msg.data, color="green")
        torque = [160, 200, 200, 200, 200]                                          # set fingers torque 
        dic = {
            "setting_cmd":"set_max_torque_limits",
            "params":{
                "hand_type": hand_type,
                "torque":torque
            }
        }
        msg.data = json.dumps(dic)
        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("changed torque to :%s",msg.data)
        cm.colorMsg(msg=msg.data, color="yellow")
        count += 1
        if count > no_loops:
            break

    try:
        control_hand(chosen_hand)
    except KeyboardInterrupt:
        cm.colorMsg(msg="Caught KeyboardInterrupt, exiting gracefully.", color="green")
    except rospy.ROSInterruptException:
        cm.colorMsg(msg="Caught KeyboardInterrupt, exiting gracefully.", color="green")
    finally:
         print("Cleaning up...")
