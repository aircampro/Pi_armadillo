#!/usr/bin/env python
# move roomba to the goal as sent over a tcp socket
# 
from __future__ import print_function
 
import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
import socket
import select
 
rospy.init_node('talker')
pub=rospy.Publisher('chatter',String, queue_size=10)
rate = rospy.Rate(10)
 
host = 'localhost'
port = 12000
backlog = 10
bufsize = 4096

class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
readfds = set([server_sock])
try:
    server_sock.bind((host, port))
    server_sock.listen(backlog)
 
    while True:
        xready, wready, rready = select.select(readfds, [], [])
        for sock in rready:
            if sock is server_sock:
                address, conn = server_sock.accept()
                readfds.add(conn)
            else:
                msg = sock.recv(bufsize)
                if len(msg) == 0:
                    sock.close()
                    readfds.remove(sock)
                else:
                    msg = msg.decode('utf-8')
                    print(msg)
                    target_array = msg.split(',')                    # sent data x,y,z as the target split it into each part
                    #sock.send(msg)
                    # Set self position when you receive something
                    pose_target_goal = Pose(int(target_array[0]), int(target_array[1]), int(target_array[2]))
                    pose_start_1 = Pose(5, 2, 0)
                    if not rospy.is_shutdown():
                        move_to_pose(pose_start_1.x, pose_start_1.y, pose_start_1.theta, pose_target_goal.x, pose_target_goal.y, pose_target_goal.theta)
 
finally:
    for sock in readfds:
        sock.close()