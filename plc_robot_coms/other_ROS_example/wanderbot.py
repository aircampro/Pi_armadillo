#! /usr/bin/env python3

# example of using rospy for object detection
#

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
from tf.transformations import euler_from_quaternion                                   #recv odom messgaes as orientation.x.y.z.w (quat)
from tf.transformations import quaternion_from_euler                                   #recv pose messages as orientation.x.y.z                          

g_range_ahead = 1                                                                      #initial value
g_setpoint_dist = 0.8                                                                  #obstcle dist
g_odom_x = 0
g_odom_y = 0
g_odom_z = 0
g_odom_theta = 0

# Callback for laser scanner
def scan_callback(msg):
    """the read sensor data to a global variable so that the entire program can handle the information"""
    global g_range_ahead
    g_range_ahead = msg.ranges[len(msg.ranges)//2]  #Store frontal distance in global variable

# Callback for odometry
def odom_callback(msg):
    """read the odometry"""
    global g_odom_x
    global g_odom_y
    global g_odom_z
    global g_odom_theta
    # get positions of the pose returned
    g_odom_x = msg.pose.pose.position.x 
    g_odom_y = msg.pose.pose.position.y 
    g_odom_z = msg.pose.pose.position.z 
    # orientation returned is a quaternion
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    e = euler_from_quaternion(q)                       # get theta
    g_odom_theta = e[2]                                # z direction 
    rospy.loginfo("rcv Odomery: x=%s y=%s z=%s theta=%s", g_odom_x, g_odom_y, g_odom_z, g_odom_theta)

# Callback for local position
local_pos = PoseStamped()
def pose_callback(msg):
    # Need global declaration in order to assign global variable
    global local_pos
    local_pos = msg
    local_op = get_odom_quat_from_pose(local_pos)
    rospy.loginfo("rcv Pose Quaternion: x=%s y=%s z=%s w=%s", local_op[0], local_op[1], local_op[2], local_op[3])
    rospy.loginfo("rcv Pose Position: x=%s y=%s z=%s", local_pos.position.x, local_pos.position.y, local_pos.position.z)
    
# pose returned from a pose subscriber is position and oreientation x.y.z
def get_odom_quat_from_pose(pose):
    odom_quat = quaternion_from_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z)
    return odom_quat
    
rospy.init_node('wanderbot')                                                           #initialize node
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)                          #subscribe to sensor data
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)                        #Prepare to deliver mobile topics
odom_subscriber = rospy.Subscriber('odom', Odometry, odom_callback)                    #subscribe to odometry data for recording
local_pos_sub = rospy.Subscriber("pose", PoseStamped, callback=pose_callback)          #subscribe to pose data for recording
    
state_change_time = rospy.Time.now()                                                   #Record current time
driving_forward = True                                                                 #Assume it is possible to drive straight ahead
rate = rospy.Rate(10)                                                                  #10Hz
time.sleep(0.1)                                                                        # delay to allow update of data on start-up
rate.sleep()                                                                           #wait 10Hz (=0.1 second)
num_cons_twist=0                                                                       #number of consecutive twists
max_twist = 4                                                                          #maximum twists angular z 4 before trying a y movement

# Loop until #ctrl key is pressed
while not rospy.is_shutdown():

    #Allowed to go straight
    if driving_forward:
        #Conditions 1 and 2 are satisfied?
        if (g_range_ahead < g_setpoint_dist or rospy.Time.now() > state_change_time):
            driving_forward = False                                                   # not possible to drive straight ahead
            state_change_time = rospy.Time.now() + rospy.Duration(5)                  # The current time is 5duration ahead in rospy's reference → means that the rotation continues for 5duration

    #not possible to drive straight ahead
    else:
        #Ended turning?
        if rospy.Time.now() > state_change_time:
            driving_forward = True                                                    #Try go straight
            state_change_time = rospy.Time.now() + rospy.Duration(30)                 #The time 30 durations ahead is the current time based on rospy's criteria → means that the car will continue going straight for 30 durations


    twist = Twist()                                                                   #Generate a Twist instance each time to initialize the whole with 0 This means that you only need to change the element you want to move below

    #straight forward possible → speed 1 in the straight line direction
    if driving_forward:
        twist.linear.x = -1

    #Not possible to go straight → change direction → speed of rotation (around z-axis) 
    else:
        twist.angular.z = 1
        num_cons_twist += 1
        if (num_cons_twist > max_twist):                                              #if we did 4 then try one movement in y direction
            twist.linear.y = -1                                                       #try a movement as y
            num_cons_twist = 0
            
    cmd_vel_pub.publish(twist)                                                         #distribute moving topic

    rate.sleep()                                                                       #wait 10Hz (=0.1 second)