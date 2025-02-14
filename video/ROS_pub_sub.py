#!/usr/bin/env python3
#
# ros image processing example
#
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import numpy as np
import cv2
import matplotlib.pyplot as plt
%matplotlib inline

# bounding box class
#
class bBox:
    def __init__(self,a,b,c,d):
        self.center_x = a
        self.center_y = b
        self.size_x = c
        self.size_y = d 

# perfrom gaus blur
#		
def gaussian_blur(img, ksize, sigma):
    if ksize <= 0:
        if img.dtype == 'uint8':
            D = 3
        else:
            D = 4
        new_ksize = round(sigma * D * 2 + 1)
        if new_ksize % 2 == 0:
            new_ksize = new_ksize + 1
    else:
        new_ksize = ksize
    print("ksize", new_ksize)

    kernel = cv2.getGaussianKernel(ksize=new_ksize, sigma=sigma, ktype=cv2.CV_32F)
    dst = cv2.sepFilter2D(img, ddepth=-1, kernelX=kernel, kernelY=kernel)
    
    return dst

# function for processing the image
#	
def process_image(im, blur_algo=0):
    # gaussian blur set-up
    k = 3
    sigma = 0
    #k = 0
    #sigma = 5
    #cv::GaussianBlur
    if blur_algo == 0:
        blur_image = cv2.GaussianBlur(im, (k, k), sigma, cv2.BORDER_REPLICATE)
    else:
        blur_image = gaussian_blur(im, ksize=k, sigma=sigma)
    img_hsv = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV_FULL)
    #(blur_image - blur_image2).mean()  # 9.298937479654947

    #cv::cvtColor(blur_image, hsv_image, CV_BGR2HSV)
    hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
    img_hsv = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV_FULL)
    # hsv range filter
    # img_h, img_s, img_v = cv2.split(img_hsv);
    # cv::inRange(hsv_image, min_range, max_range, hsv_range_image);
    # hsv_range_image = cv2.inRange(img_hsv, np.array([159, 127,0]), np.array([177, 255, 255]))
    hsv_range_image = cv2.inRange(img_hsv, np.array([0, 180,0]), np.array([255, 0, 255]))
    # opening
    # cv::dilate(hsv_range_image, mid_mask, cv::Mat(), cv::Point(-1,-1), 2);
    # cv::erode(mid_mask, opened_mask, cv::Mat(), cv::Point(-1,-1), 2);
    kernel = np.ones((3,3),np.uint8)
    dilation_th = cv2.dilate(hsv_range_image,kernel)
    kernel5 =  np.ones((5,5),np.uint8)
    dilation5_th = cv2.dilate(hsv_range_image,kernel5)
    frame_dilation = cv2.hconcat([dilation_th,dilation5_th])
    erosion_th = cv2.erode(frame_dilation,kernel)
    erosion5_th = cv2.erode(frame_dilation,kernel5)
    frame_erosion = cv2.hconcat([erosion_th,erosion5_th])
    # get contours
    # cv::findContours(opened_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    # contours, hierachy = cv2.findContours(frame_erosion, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierachy = cv2.findContours(frame_erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out_frame = frame_erosion
    
    # show contours
    fig, ax = plt.subplots(2, 5, sharex=True, sharey=True, figsize=(15,15))
    ax = ax.flatten()
    for i in range(len(contours)):
        img = cv2.drawContours(frame_erosion, contours, i, (0,255,0), 2)                                           
        ax[i].imshow(img)
        ax[i].set_title('contour{}'.format(i))
    ax[0].set_xticks([])
    ax[0].set_yticks([])
    plt.tight_layout()
    plt.show()
    
    # get convex
    approxes = []
    for c in contours:
        if cv2.contourArea(c) > 100:
            hull = cv2.convexHull(c)
            approxes.append(hull) 
    # draw on the outframe 
    for a in approxes:
        cv2.polylines(out_frame, a, true, cv::Scalar(255, 0, 0), 2, cv::LINE_AA)            
    # get bbox
    bboxes = []
    for a in approxes:
        rect = cv2.boundingRect(a)
        bboxes.append(rect)
    # rotation rect
    rects = []
    for a in approxes:
        rect = cv2.minAreaRect(a)
        rects.append(rect)
    # output detections result
    output = []
    for r in bboxes:
        detect = bBox((r.x + r.width / 2), (r.y + r.height / 2), r.width, r.height)
        output.detections.append(detect)
    return output, out_frame
	
class cvBridgeDemo:
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/head_camera/image_raw", Image, self.image_callback, queue_size=1) 
        self.image_pub = rospy.Publisher("/cmatch/image_raw", Image, queue_size=1)

    def image_callback(self, ros_image):
        try:
            input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        outp, output_image = process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "mono8"))
        
        cv2.imshow(self.node_name, output_image)   
        cv2.waitKey(1)
                                 
    def cleanup(self):
        cv2.destroyAllWindows()
    
if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()