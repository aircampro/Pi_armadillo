#!/usr/bin/env python
#
# It uses a ROS bridge to aquire the image frame and paramters and calculates the distance from object automatically brakes and publishes twist when radar sensor inside distance it turns away
#
# this ensures correct openCV bridge to ros
#
#$ sudo apt install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
#$ mkdir -p cv_bridge_ws/src && cd cv_bridge_ws
#$ git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
#$ apt-cache show ros-melodic-cv-bridge | grep Version
#$ cd src/vision_opencv/
#$ git checkout 1.13.0
#$ cd ../../
#$ catkin config -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
#$ catkin build
#$ source devel/setup.bash --extend
#$ cd ../catkin_ws
#$ catkin build
#$ source devel/setup.bash
#
# $ sudo pip install -U trollius rosdep rosinstall-generator wstool rosinstall defusedxml netifaces
# $ sudo pip install --upgrade setuptools; sudo pip install -U rospkg
#
# sudo pip uninstall enum34; pip install empy
#
# Change the 5 to 7 lines of the ~/colcon_ws/src/<your project name>/package.xml proj=robo_picar_controls
# <exec_depends>rclpy</exec_depends>
# <exec_depends>std_msgs</exec_depends>
# <exec_depends>geometry_msgs</exec_depends>
# <exec_depends>sensor_msgs</exec_depends>
# <exec_depends>sensor_msgs</exec_depends>
#
# add to entry points in setup.python
#
# ‘camnode = robo_picar_controls.camnode:main’,
#
# further ref:- https://demura.net/robot/ros2/20748.html
#
import rospkg
version = rospkg.RosStack().get_stack_version('ros')
_ROS = int(version)                                                     # set to the version of ROS being used 1 or 2 
if _ROS == 1:
    import rospy
elif _ROS == 2:
    import rclpy
    from rclpy.node import Node
    import sys
    from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import wiringpi as wp
import math

class CarController(object):

    FORWARD_PIN = 6
    BACKWARD_PIN = 5
    STEER_PIN = 18
    STEER_MIN = 65
    STEER_MAX = 89

    def __init__(self):
        # throttle
        wp.wiringPiSetupGpio()
        wp.pinMode(CarController.FORWARD_PIN,1)
        wp.softPwmCreate(CarController.FORWARD_PIN,0,100)
        wp.pinMode(CarController.BACKWARD_PIN,1)
        wp.softPwmCreate(CarController.BACKWARD_PIN,0,100)
        # steering
        wp.wiringPiSetupGpio()
        wp.pinMode(CarController.STEER_PIN,2) # Hardware PWM
        wp.pwmSetMode(0)
        wp.pwmSetRange(1024)
        wp.pwmSetClock(375)
    
    def throttle(self,x):
        """
        x: -1.0 to 1.0
        """
        x = int(max(-100,min(100,100*x)))
        if 0<=x:
            wp.softPwmWrite(CarController.FORWARD_PIN,abs(x))
            wp.softPwmWrite(CarController.BACKWARD_PIN,0)
        if x<0:
            wp.softPwmWrite(CarController.FORWARD_PIN,0)
            wp.softPwmWrite(CarController.BACKWARD_PIN,abs(x))

    def steer(self,x):
        """
        x: -1.0 to 1.0
        """
        a = 0.5 * (CarController.STEER_MAX - CarController.STEER_MIN)
        b = 0.5 * (CarController.STEER_MAX + CarController.STEER_MIN)
        x = int(max(CarController.STEER_MIN,min(CarController.STEER_MAX, a*x+b)))
        wp.pwmWrite(CarController.STEER_PIN,x)

    def brake(self):
        wp.softPwmWrite(CarController.FORWARD_PIN,100)
        wp.softPwmWrite(CarController.BACKWARD_PIN,100)

class NaiveTTC(object):    
    """
    Ratio of TimeToContact Estimation Algorithm for Distortion-free Camera
    """
    def __init__(self,initial_bg,framerate=90,m0=5,r0=0.6):
        self.m0 = m0
        self.bg = initial_bg
        self.height,self.width = initial_bg.shape[:2]
        self.ex, self.ey = np.meshgrid(np.linspace(-self.width/2,self.width/2,self.width),np.linspace(-self.height/2,self.height/2,self.height))
        self.ex, self.ey = self.ex*0.5/framerate, self.ey*0.5/framerate                                                   # The correction factor of dx, dy, dt is multiplied first.く

        mask = (self.ex**2+self.ey**2)**0.5<r0*self.height/2
        self.ex *= mask
        self.ey *= mask

    def update(self,img):
        img = cv2.GaussianBlur(img,(5,5),2.0).astype(np.float32)

        dt = cv2.subtract(img,self.bg)
        dx = cv2.Sobel(img,cv2.CV_32F,1,0,ksize=1)                                                                         # sobel edge detection
        dy = cv2.Sobel(img,cv2.CV_32F,0,1,ksize=1)

        self.bg = img                                                                                                      # next background
        idx0 = np.abs(dt)>self.m0                                                                                          # remove noise

        ttc = (-(dx*self.ex+dy*self.ey)/dt)[idx0]                                                                          # calc time-to-contact
        mttc = np.mean(ttc)
        if math.isnan(mttc):
            mttc = 0.0
        return mttc

# choose the control depending on the version of ROS
if _ROS == 1:
    from ROS1_Car_CamNode import CamNode
elif _ROS == 2:
    from ROS2_Car_CamNode import CamNode

def main():
    if _ROS == 1:
        rospy.init_node('robo_picar_controls', anonymous=True)                                # Create a ROS node
        node = CamNode()                                                                      # Start the program
        node.run()
    elif _ROS == 2:
        rclpy.init(args=args)
        translate = CamNode()
        rclpy.spin(translate)
        rclpy.shutdown()
if __name__ == '__main__':
    main()
