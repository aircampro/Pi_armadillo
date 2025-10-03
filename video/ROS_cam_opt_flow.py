#!/usr/bin/env python
#
# This has been expanded from the follwoing open source project
# ref :- https://robotec.ua/open_project.html
#
# It uses a ROS bridge to aquire the image frame and then runs various openCV on the input frames
#
# This ensures correct openCV bridge to ros is used.
#
# $ sudo apt install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
# $ mkdir -p cv_bridge_ws/src && cd cv_bridge_ws
# $ git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
# $ apt-cache show ros-melodic-cv-bridge | grep Version
# $ cd src/vision_opencv/
# $ git checkout 1.13.0
# $ cd ../../
# $ catkin config -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
# $ catkin build
# $ source devel/setup.bash --extend
# $ cd ../catkin_ws
# $ catkin build
# $ source devel/setup.bash
#
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import argparse
import pyflow
from geometry_msgs.msg import Twist

class Node():
    def __init__(self):
        self._publish_rate = rospy.get_param('~publish_rate', 100)                                        # Get ROS parameters
        lower_color_boundary = rospy.get_param('/roslaunch/lower_color_boundary')
        upper_color_boundary = rospy.get_param('/roslaunch/upper_color_boundary')
        self._box_color = rospy.get_param('/roslaunch/box_color')
        self._toVisualize = rospy.get_param('~visualization')     
        self._result_pub = rospy.Publisher('~filtered_image', Image, queue_size=1)                        # Create ROS topics
        self._visual_pub = rospy.Publisher('~visualization', Image, queue_size=1)
        self._camera_input = rospy.Subscriber('~input', Image, self.imageCallback, queue_size=1)
        self._last_msg = None                                                                             # Create multitreading locks for frame grabber 
        self._msg_lock = threading.Lock()
        self._lower = np.array(lower_color_boundary, np.uint8)                                            # Set range for the color
        self._upper = np.array(upper_color_boundary, np.uint8)
        self._vel = Twist()                                                                               # twist from color
        self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)                                   # ROS topic for twist (robot movement)

    def __createMask(hsvFrame) :
        # Obtain masks for colored objects
        mask = cv2.inRange(hsvFrame, self._lower, self._upper)
        kernal = np.ones((5, 5), "uint8")
        mask = cv2.dilate(mask, kernal)
        return mask

    def __drawBoxes(np_image, contours) : 
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            thresh = 500                                                                      # size of area before you draw box
            if (area > thresh):                                                               # Draw only big boxes
                x, y, w, h = cv2.boundingRect(contour)                                        # Get the coordinates of the box
                # Draw the box
                np_image = cv2.rectangle(np_image, (x, y),  
                    (x + w, y + h),
                    self._box_color, 2)
        return np_image

    def __detect_colors(np_image) : 
        hsvFrame = cv2.cvtColor(np_image, cv2.COLOR_BGR2HSV)
        mask = __createMask(hsvFrame)                                                          # Create the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)           # Create contours for the object
        return contours 

    def get_colored_area(self, cv2_image, lower, upper):
        hsv_image = cv2.cv2tColor(cv2_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(cv2_image, cv2_image, mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return (area, extracted_image)

    def imageCallback(self, msg):
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def run(self, vopt=0):                                                              # grab ROS frame and process according the argument passed as vopt
        rate = rospy.Rate(self._publish_rate)                                           # Rate of the main loop (frame rate)
        cv_bridge = CvBridge()                  #
        no_of_loop = 0
        while not rospy.is_shutdown():                                                  # while rospy is running grab frame and process it
            if self._msg_lock.acquire(False):                                           # If there is no lock on the message (not being written to in the moment)
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue
                    
            if msg is not None:                                                          # If the message frame is not empty
                try:
                    np_image = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')                      # Convert the message to an OpenCV object
                except cv2BridgeError, e:
                    print e
                if vopt == 0:                                                            # detect contours
                    contours = __detect_colors(np_image)                                 # Get the contours of the objects
                    if (len(contours) != 0) :                                            # IF there are detected objects
                        self._result_pub.publish(msg)                                    # Send the message
                        if (self._toVisualize) :                                         # If the option of visualization is enabled
                            np_image = __drawBoxes(np_image, mask)                       # Draw the boxes over the original image
                            try:
                                redrawn_image = cv_bridge.cv2_to_imgmsg(np_image, 'bgr8')    # Send the redrawn image
                                self._visual_pub.publish(redrawn_image)         
                            except cv2BridgeError, e:
                                print e                                
                elif vopt == 1:                                                          # optical flow method No.1
                    if no_of_loop == 0:                                                  # first image
                        im = np_image
                        no_of_loop += 1
                    elif no_of_loop >= 1:                                                # second image
                        pimg = im
                        im = np_image
                        im1 = pimg.astype(float) / 255.
                        im2 = im.astype(float) / 255.    
                        alpha = 0.012                                                     # Flow Options:
                        ratio = 0.75
                        minWidth = 20
                        nOuterFPIterations = 7
                        nInnerFPIterations = 1
                        nSORIterations = 30
                        colType = 0                                                       # 0 or default:RGB, 1:GRAY (but pass gray image with shape (h,w,1))
                        u, v, im2W = pyflow.coarse2fine_flow(im1, im2, alpha, ratio, minWidth, nOuterFPIterations, nInnerFPIterations, nSORIterations, colType)
                        flow = np.concatenate((u[..., None], v[..., None]), axis=2)
                        hsv = np.zeros(im1.shape, dtype=np.uint8)
                        hsv[:, :, 0] = 255
                        hsv[:, :, 1] = 255
                        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
                        hsv[..., 0] = ang * 180 / np.pi / 2
                        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
                        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                        try:
                            redrawn_image1 = cv_bridge.cv2_to_imgmsg(rgb, 'bgr8')    
                            self._result_pub.publish(redrawn_image1) 
                            redrawn_image2 = cv_bridge.cv2_to_imgmsg(im2W[:, :, ::-1] * 255, 'bgr8')    
                            self._visual_pub.publish(redrawn_image2) 
                        except cv2BridgeError, e:
                            print e                             
                elif vopt == 2:                                                          # optical flow method No.2
                    if no_of_loop == 0:                                                  # first image
                        pimg = np_image
                        feature_params = dict(maxCorners = 300, qualityLevel = 0.2, minDistance = 2, blockSize = 7)                                   # Parameters for Shi-Tomasi corner detection
                        lk_params = dict(winSize = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))     # Parameters for Lucas-Kanade optical flow
                        color = (0, 255, 0)                                                                                                           # Variable for color to draw optical flow track
                        first_frame = pimg                                               # ret = a boolean return value from getting the frame, first_frame = the first frame in the entire video sequence
                        prev_gray = cv2.cv2tColor(first_frame, cv2.COLOR_BGR2GRAY)       # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
                        # Finds the strongest corners in the first frame by Shi-Tomasi method - we will track the optical flow for these corners
                        # https://docs.opencv2.org/3.0-beta/modules/imgproc/doc/feature_detection.html#goodfeaturestotrack
                        prev = cv2.goodFeaturesToTrack(prev_gray, mask = None, **feature_params)
                        mask = np.zeros_like(first_frame)                                # Creates an image filled with zero intensities with the same dimensions as the frame - for later drawing purposes 
                        no_of_loop += 1
                    elif no_of_loop >= 1:                                                # next images
                        im = np_image
                        frame = im
                        gray = cv2.cv2tColor(frame, cv2.COLOR_BGR2GRAY)                  # Converts each frame to grayscale - we previously only converted the first frame to grayscale
                        # Calculates sparse optical flow by Lucas-Kanade method
                        # https://docs.opencv2.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#calcopticalflowpyrlk
                        prev = cv2.goodFeaturesToTrack(prev_gray, mask = None, **feature_params)
                        next, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev, None, **lk_params)
                        good_old = prev[status == 1].astype(int)                         # Selects good feature points for previous position
                        good_new = next[status == 1].astype(int)                         # Selects good feature points for next position
                        for i, (new, old) in enumerate(zip(good_new, good_old)):         # Draws the optical flow tracks
                            a, b = new.ravel()                                           # Returns a contiguous flattened array as (x, y) coordinates for new point
                            c, d = old.ravel()                                           # Returns a contiguous flattened array as (x, y) coordinates for old point
                            mask = cv2.line(mask, (a, b), (c, d), color, 2)              # Draws line between new and old position with green color and 2 thickness
                            frame = cv2.circle(frame, (a, b), 3, color, -1)              # Draws filled circle (thickness of -1) at new position with green color and radius of 3
                        output = cv2.add(frame, mask)                                    # Overlays the optical flow tracks on the original frame
                        prev_gray = gray.copy()                                          # Updates previous frame
                        prev = good_new.reshape(-1, 1, 2)                                # Updates previous good feature points
                        try:                        
                            self._result_pub.publish(msg) 
                            redrawn_image2 = cv_bridge.cv2_to_imgmsg(output, 'bgr8')    
                            self._visual_pub.publish(redrawn_image2) 
                        except cv2BridgeError, e:
                            print e  
                elif vopt == 3:                                                          # optical flow method No.3
                    if no_of_loop == 0:                                                  # first image
                        pimg = np_image
                        movie_output_nm = 'masked_frame_movie.mp4'                       # name for your output movie (masked frame)
                        feature_params = dict( maxCorners = 100, qualityLevel = 0.3, minDistance = 7, blockSize = 7 )                                    # Shi-Tomasi
                        lk_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))      # Lucas-Kanade 
                        color = np.random.randint(0, 255, (100, 3))
                        frame = pimg
                        gray_prev = cv2.cv2tColor(frame, cv2.COLOR_BGR2GRAY)
                        feature_prev = cv2.goodFeaturesToTrack(gray_prev, mask = None, **feature_params)
                        mask = np.zeros_like(frame)
                        # ref :- https://rikoubou.hatenablog.com/entry/2019/01/15/174751
                        frame_rate = rate 
                        size = (640, 480)                                                                                    
                        fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')                                                    
                        masked_frame_writer = cv2.VideoWriter(movie_output_nm, fmt, frame_rate, size)               
                        no_of_loop += 1
                    elif no_of_loop >= 1:                                                # next images
                        im = np_image
                        frame = im
                        if feature_prev is None:
                            print("feature error")
                            break
                        i = 1
                        gray_next = cv2.cv2tColor(frame, cv2.COLOR_BGR2GRAY)
                        feature_next, status, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray_next, feature_prev, None, **lk_params)
                        if feature_next is None:
                            print("feature extraction error")
                            break
                        good_prev = feature_prev[status == 1]
                        good_next = feature_next[status == 1]
                        for i, (next_point, prev_point) in enumerate(zip(good_next, good_prev)):
                            prev_x, prev_y = prev_point.ravel()
                            next_x, next_y = next_point.ravel()
                            mask = cv2.line(mask, (int(next_x), int(next_y)), (int(prev_x), int(prev_y)), color[i].tolist(), 2)
                            frame = cv2.circle(frame, (int(next_x), int(next_y)), 5, color[i].tolist(), -1)
                        img = cv2.add(frame, mask)
                        masked_frame_writer.write(mask) 
                        gray_prev = gray_next.copy()
                        feature_prev = good_next.reshape(-1, 1, 2)  
                        try:
                            self._result_pub.publish(msg) 
                            redrawn_image2 = cv_bridge.cv2_to_imgmsg(img, 'bgr8')    
                            self._visual_pub.publish(redrawn_image2) 
                        except cv2BridgeError, e:
                            print e 
                elif vopt == 5:                                                                                    # color checker with movement  
                    close_theshold = 1000                                                                          # color threshold set-point       
                    blue_area, blue_image = self.get_colored_area(np_image, np.array([50,100,100]), np.array([150,255,255])) # blue area & image
                    red_area, red_image = self.get_colored_area(np_image, np.array([150,100,150]), np.array([180,255,255]))  # red area & image
                    try:
                        self._result_pub.publish(self._bridge.cv2_to_imgmsg(blue_image, 'bgr8'))                    # publish blue image
                        self._visual_pub.publish(self._bridge.cv2_to_imgmsg(red_image, 'bgr8'))                     # publish red image
                    except cv2BridgeError, e:
                        print e   
                    if blue_area > close_theshold:                                                                  # close to blue
                        self._vel.linear.x = 0.5                                                                    # linear velocity forward in camera direction
                        self._vel_pub.publish(self._vel)                                                            # publish the twist message
                    elif red_area > close_theshold:                                                                 # close to red (blue prefernce - swap if red preference)
                        self._vel.linear.x = -0.5                                                                   # move backwards direction
                        self._vel_pub.publish(self._vel)                                                            # publish twist message to robot                        
        if vopt == 3:                            
            masked_frame_writer.release()

def main(vopt):
    rospy.init_node('robotec_image_filtering', anonymous=True)                                # Create a ROS node
    node = Node()                                                                             # Start the program
    node.run(vopt)                                                                            # run the frame grabber with the video processing option passed to this script

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='parser')
    parser.add_argument('video_option', type=str, help='option to process video 0-5')             # parse the command line arg which determines the video processing option
    args = parser.parse_args()	
    vopt = args.video_option
    main(vopt)                                                                                # run the frame grabber with the video processing option
