#!/usr/bin/env python3
#
# LinkerBot Hand SDK Examples
#
# ref SDK :- https://github.com/linkerbotai/linker_hand_sdk/blob/main/examples/set_linker_hand_torque/scripts/set_linker_hand_torque.py
#
import rospy,json
from std_msgs.msg import String
import time

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
        else:
            print(str + msg)
            return
        str += msg + "\033[0m"
        print(str)

if __name__ == '__main__':
    '''
    L7:
    rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=right _speed:="[255,255,255,255,255,255,255]"

    other:
    rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=right _speed:="[100,100,100,100,100]"
    '''
    rospy.init_node('get_linker_hand_speed', anonymous=True)
    hand_type = rospy.get_param("~hand_type",default="left")                        # get the hand type from ROS command (left or right)
    speed = rospy.get_param('~speed', default=[255,255,255,255,255])                # get the speed for each finger
    delta_s = 1                                                                     # how much to increase the speed by
    no_loops = 2                                                                    # number of repeat loops
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
        cm.colorMsg(msg=msg.data, color="green")
        count += 1
        if count > no_loops:
            break