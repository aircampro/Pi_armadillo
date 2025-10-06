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
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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

class Node():
    def __init__(self):
        self._publish_rate = rospy.get_param('~publish_rate', 100)                                                         # Get ROS parameters
        self.thresh = rospy.get_param('/roslaunch/threshold')
        self.fps = rospy.get_param('/roslaunch/fps')
        self.dist_spt = rospy.get_param('/roslaunch/distance_setpoint')

        self._result_pub = rospy.Publisher('~filtered_image', Image, queue_size=1)                                          # Create ROS topics
        self._camera_input = rospy.Subscriber('~input', Image, self.imageCallback, queue_size=1)
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self.cmd_vel_pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)

        self.min_range = None
        self._last_msg = None                                                                                                # Create multitreading locks
        self._msg_lock = threading.Lock()

    def imageCallback(self, msg):
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def callback_scan(self, data):
        fov = np.deg2rad(60)
        min_range = data.range_max
        min_idx = -1
        angle = data.angle_min
        for idx, r in enumerate(data.ranges):
            angle += data.angle_increment
            if -fov<angle<fov:
                if r<min_range:
                    min_range = r
                    min_idx = idx
        if min_idx < len(data.ranges)/2.0:
            self.direction = "RIGHT"
        else:
            self.direction = "LEFT"
        self.min_range = min_range

    def run(self):
        WIDTH = 64
        HEIGHT = 48
        if self.fps <= 0:
            FRAMERATE = 30
        else:
            FRAMERATE = self.fps        
        KEEP_BRAKE = 0.2*FRAMERATE                           # # Number of steps to continue backward output
        if self.thresh <= 0:
            THRESHOLD = 0.45                                 # Brake when the estimated TTC falls below this value
        else:
            THRESHOLD = self.thresh         
        R = 0.4                                              # Estimated TTC update ratio
        if self.dist_spt <= 0: 
            SPT = 0.4
        else:
            SPT = self.dist_spt        
        rate = rospy.Rate(self._publish_rate)                # Rate of the main loop
        cv_bridge = CvBridge()                               # ROS camera interface
        ctrl = CarController()                               # interface to PWM controlled motors on the car
        ctrl.steer(0)                                        # ahead
        count_loop = 0
        vel = Twist()       
        while not rospy.is_shutdown():                       # while ROS running grab the frames from the camera

            if self._msg_lock.acquire(False):                # If there is no lock on the message (not being written to in the moment)
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue                                                      # create twist message for ROS control when laser scanner is inside range                   
            if msg is not None:                                                       # If the message is not empty
                np_image = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')                       # Convert the message to an OpenCV object
                img_resize = cv2.resize(np_image, (256, 256))
                img = img_resize[:HEIGHT].astype(np.float32)                          # I420->Y
                if count_loop == 0:
                    ettc = NaiveTTC(img, FRAMERATE, m0=5,r0=0.60)
                    ctrl.throttle(1.0)
                    brake = 0
                    mttc = 0.50		
                    count_loop += 1
                elif count_loop >= 1:
                    ttc = ettc.update(img)
                    if ttc>0:
                        mttc = min(1.0,(1-R)*mttc+R*ttc)
                    print("{0:+03.6f}".format(mttc))
                    if brake==0 and mttc<THRESHOLD:
                        brake = 1
                    elif brake>0:
                        if brake<KEEP_BRAKE:
                            ctrl.throttle(-1.0)                                          # slow down
                            vel.linear.x = 0.05
                            brake += 1
                        else:
                            break
                image_msg = cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                self._result_pub.publish(image_msg)

            if self.min_range is not None:
                if self.min_range >= SPT:
                    if not(brake<KEEP_BRAKE and brake>0):                             # no brake applied from the video feed
                        vel.linear.x = 0.2                                            # go forward x                              
                    ctrl.steer(0.0)                                                   # steering of piCar is ahead
                    vel.angular.z = 0.0
                else:
                    vel.linear.x = 0.0
                    if self.direction == "RIGHT":
                        vel.angular.z = 0.5
                        ctrl.steer(-0.1)                                              # turn right
                    elif self.direction == "LEFT":
                        vel.angular.z = -0.5
                        ctrl.steer(0.1)                                               # turn left
            self.cmd_vel_pub.publish(vel)                                             # publish the twist to ROS                  
        ctrl.brake()                                                                      # stop                   
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)                                                     # publish the twist to ROS         
def main():
    rospy.init_node('robo_picar_controls', anonymous=True)                                # Create a ROS node
    node = Node()                                                                         # Start the program
    node.run()

if __name__ == '__main__':
    main()
