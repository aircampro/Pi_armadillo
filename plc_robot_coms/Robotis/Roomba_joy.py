#!/usr/bin/env python3
#
# This is a ROS controlled robot which moves according to command by the PS4 controller
# ref:- https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/avoidance.py
#
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

# scalar for the distance
DIST_MAX=5.0
VELO_MAX=0.6
YAW_MAX=360.0
YRATE_MAX=-0.5

# simple controller for the roomba which feedsback its odometry
#
class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for odometry
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        # Subscriber for joystick
        joy_sub = rospy.Subscriber('/joy', Joy, self.control)
		
        self.x = None
        self.y = None
        self.yaw = None
        while self.x is None:
            rospy.sleep(0.1)

    # control from the joystick
    def control(self, msg):
        # left sticks control robot twist                                    
        L_horizontal = msg.axes[0]                                     # L stick horizontal (left=+1, right=-1)
        L_vertical = msg.axes[1]                                       # L stick vertical (up=+1, down=-1)）
        R_horizontal = msg.axes[2]                                     # R stick horizontal (left=+1, right=-1)
        R_vertical = msg.axes[5]                                       # R stick vertical (up=+1, down=-1)
        if msg.buttons[0] == 1 :                                       # press square on ps4
            self.go_straight(L_horizontal*DIST_MAX, L_vertical*VELO_MAX)	
        elif msg.buttons[1] == 1 :                                     # press cross on ps4
            self.turn_right(R_horizontal*YAW_MAX, R_vertical*YRATE_MAX)
        elif msg.buttons[2] == 1 :                                     # press circle on ps4
            self.turn_left(R_horizontal*YAW_MAX, R_vertical*YRATE_MAX)
			
    # read from the odometry
    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = self.get_yaw_from_quaternion(data.pose.pose.orientation)

    # go straight a distance
    def go_straight(self, dis, velocity=0.3):
        vel = Twist()
        x0 = self.x
        y0 = self.y
        while(np.sqrt((self.x-x0)**2+(self.y-y0)**2)<dis):
            vel.linear.x = velocity
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    # turn right by yaw angle
    def turn_right(self, yaw, yawrate=-0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    # turn right by yaw angle
    def turn_left(self, yaw, yawrate=0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]

if __name__=='__main__':
    simple_controller = SimpleController()
    try:
	    rospy.spin()
		# to command without joystick in a program do like this ...
		#
        #simple_controller.go_straight(1.0)
        #simple_controller.turn_left(90)
        #simple_controller.turn_right(90)
    except rospy.ROSInitException:
        pass
