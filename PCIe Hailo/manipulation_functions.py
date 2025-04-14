#
# openCV functions for manipulation of camera frames and images
#
import cv2 as cv
from enum import Enum
import numpy as np
import time

# the size of the ROI determines our distance from the object in sight
# therefore the reaction of the craft
# too far - ignore
# far - slow down to anticipate the reaction
# in range react to color of object
# past - ignore object and ramp speed back-up to previous
#
class SizeOfROI(Enum):
    FAR_AWAY = 10
    INSIGHT = 50
    REACT = 900

# what the craft does in relation to the seen object
class craftReaction(Enum):
    IGNORE = 0
    SLOW_DOWN = 1
    READ_REACT = 2
    SPEED_UP = 3
         
# photos folder and npz file location for use with picture undistort if needed
#
folder_path = "/photo"
calb_folder_path = "/20231211_disparity_calc/20231211_calibration_image"

# global counters
g_counter = 0
g_icounter = 0

# load the camera paramters file
npz = np.load(f"{calb_folder_path}/camera_parameter.npz")           # Read camera parameters
ret = npz["arr_0"]
mtx = npz["arr_1"]                                                  # camera matrix
dist = npz["arr_2"]                                                 # Distortion coefficients
rvecs = npz["arr_3"]                                                # Rotation matrix
tvecs = npz["arr_4"]                                                # Translation vector

# undistort the image using the npz file
def undistortion(img, mtx, dist, h, w) :                         
    h, w = img.shape                                             # Get the number of pixels in the image's width and height.
    # Get the optimal camera matrix.
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist,  # Camera matrix and distortion coefficients.
                                                     (w,h),
                                                     0,          # Setting this argument to 1 will preserve all pixels of the original image.
                                                     (w,h))
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)       # Remove image distortion.
    return dst

# gray scale image
def grayscale( filenm ):
    img1 = cv.imread(f"{folder_path}/{filenm}", cv.IMREAD_GRAYSCALE) 
    return img1

# extract region of interest from the frame
def img_roi(img1, x1, x2, y1, y2):
    img1_roi = img1[y1:y2, x1:x2]
    return img1_roi

# calculate the roi area to decide how to react to the traffic light
def calc_roi_area( x1, x2, y1, y2 ):
    return (x2-x1)*(y2-y1)

# calculate the craft mode based of the roi of the object (our relative distance from it)
def get_mode_from_roi(roi_area):
    if roi_area <= SizeOfROI.FAR_AWAY.value:
        mode = craftReaction.IGNORE.value
    elif roi_area > SizeOfROI.FAR_AWAY.value and roi_area <= SizeOfROI.INSIGHT.value:
        mode = craftReaction.SLOW_DOWN.value 
    elif roi_area > SizeOfROI.INSIGHT.value and roi_area <= SizeOfROI.REACT.value:
        mode = craftReaction.READ_REACT.value                 
    elif roi_area > SizeOfROI.REACT.value:
        mode = craftReaction.SPEED_UP.value  
    return mode

# extract the colors from the bgr frame range 0-255
def extract_colors(img1):
    result = img1.copy()
    imgH, imgW = result.shape[:2]
    blue = result[imgH-10, imgW-10, 0]
    green = result[imgH-10, imgW-10, 1]
    red = result[imgH-10, imgW-10, 2]
    return blue, green, red

# bgr to rgb
def bgr_rgb(material) : 
    return cv.cvtColor(material, cv.COLOR_BGR2RGB) 

# Set all pixel values ​​less than v=100 to 0.
def set_zero( img, v=100 ):
    return img[img < v] = 0

# Set all pixel values ​​greater than or equal to v=200 to 255.
def set_max( img, v=200 ):
    return img[img >= v] = 255

def overlap(base_img_path: str, over_img_path: str) -> None:
    baseImg = cv.imread(base_img_path)
    overImg = cv.imread(over_img_path, cv.IMREAD_UNCHANGED)

    width, height = baseImg.shape[:2]
    baseImg[0:width, 0:height] = (
        baseImg[0:width, 0:height] * (1 - overImg[:, :, 3:] / 255)
         + overImg[:, :, :3] * (overImg[:, :, 3:] / 255)
    )
    cv.imwrite(over_img_path, baseImg)

# image homography
def homography(img):
    rows, cols = img.shape[:2]
    src_pts = np.float32([[0, 0], [cols-1, 0], [0, rows-1], [cols-1, rows-1]])
    dst_pts = np.float32([[100, 100], [cols-101, 100], [100, rows-101], [cols-101, rows-101]])
    matrix = cv.getPerspectiveTransform(src_pts, dst_pts)
    dst = cv.warpPerspective(img, matrix, (cols, rows))
    return dst

# panorama from 2 images
def panorama2images(frame1, frame2):
    stitcher = cv.Stitcher.create()
    images = [frame1, frame2]
    status, panorama = stitcher.stitch(images)
    if status == cv.Stitcher_OK:
        return panorama
    else:
        return -1

# separate background and foreground
# first create mog to use
def createMog():
    fgbg = cv.createBackgroundSubtractorMOG2()
    return fgbg

# process each frame 
def process_fgbg(frame, fgbg):
    fgmask = fgbg.apply(frame)
    return fgmask

# functions to write out the frame to a file
def write_indexed_jpg( frm ):
    global g_icounter
    cv.imwrite(f"{folder_path}/{g_icounter}.jpg", frm) 
    g_icounter += 1 

def write_indexed_jpg_2png(i):
    global g_counter
    img = cv.imread(f"{folder_path}/IMG_0{i}.jpg") 
    cv.imwrite(f"{folder_path}/{g_counter}.png", img) 
    g_counter += 1 

def write_indexed_png_2jpg(i):
    global g_counter
    img = cv.imread(f"{folder_path}/IMG_0{i}.png") 
    cv.imwrite(f"{folder_path}/{g_counter}.jpg", img) 
    g_counter += 1 

# traffic light controller object makes descisions on the observation (how far & what color)
class TrafficLightController(object):

    def __init__(self, init_speed, mode=craftReaction.IGNORE.value, quite_close=10, ramp_speed_settings=[1,5,10,20], ud=10.0):
        self.mode = mode
        self.pmode = mode
        self.start_react = 0
        self.ramp_time = 0
        self.yellow = 0
        self.red = 0
        self.green = 0
        self.close_time = quite_close
        self.ramp_down_time = ramp_speed_settings
        self.init_speed = init_speed
        self.speed = init_speed
        self.last_ch = 0
        self.ch_time = 0
        self.update_time = ud

    # look at the roi for the object to infer the mode of action
    def check_distance(self, roi):
        self.pmode = self.mode
        self.mode = get_mode_from_roi(roi)

    # from the mode of action determine the ramp speed
    def check_action(self, frm):
        if self.mode == craftReaction.READ_REACT.value and not self.pmode == craftReaction.READ_REACT.value:
            self.start_react = time.time()
            self.yellow = 0                                                      
            self.red = 0 
            self.green = 0 
        if self.mode == craftReaction.READ_REACT.value:
            b, g, r = extract_colors(frm)
            if g >= 80 and r >= 80 and b <= 50:                                     #and self.yellow == 0: uncomment if you want yellow first seen
                ts = time.time - self.start_react
                if ts >= self.close_time:                                           # we are quite close
                    self.ramp_time = self.ramp_down_time[1]                         # stop a bit faster
                else:
                    self.ramp_time = self.ramp_down_time[2]                         # stop slowly
                self.yellow = 1
            elif g <= 50 and r >= 80 and b <= 50 and self.red == 0:                 # red first seen
                ts = time.time - self.start_react
                if ts >= self.close_time:                                           # we are quite close
                    self.ramp_time = self.ramp_down_time[0]                         # ramp quickly down
                else:
                    self.ramp_time = self.ramp_down_time[1]                         # stop slowly as yellow
                self.red = 1
            elif g >= 80 and r <= 50 and b <= 50 and self.red == 1:                 # green after red seen
                self.ramp_time = self.ramp_down_time[3]                             # speed back-up
                self.red = 0
                self.yellow = 0
                self.green = 1
                self.mode == craftReaction.SPEED_UP.value
            elif g >= 80 and r <= 50 and b <= 50 and not self.green == 1:           # green seen without a red
                self.ramp_time = self.ramp_down_time[3]                             # only slow down a little
                self.red = 0
                self.yellow = 0
        elif self.mode == craftReaction.SLOW_DOWN.value:     
                self.ramp_time = self.ramp_down_time[3]                             # slow down a little 
        elif self.mode == craftReaction.SPEED_UP.value:     
                self.ramp_time = self.ramp_down_time[3]                            
        else:
            self.ramp_time = 0

    # ramp up/down the craft speed
    def change_speed(self):
        if self.ramp_time > 0:
            self.last_ch = self.ch_time
            self.ch_time = time.time()
            if (self.ch_time - self.last_ch) > self.update_time:
                delta = self.speed / (self.ramp_time * 10.0)
                if self.mode == craftReaction.SPEED_UP.value:
                    if self.speed < self.init_speed:
                        self.speed += delta
                elif self.mode == craftReaction.IGNORE.value:
                    self.speed = self.init_speed
                else:
                    if self.speed > 0.0:
                        self.speed -= delta 
        else:
            self.speed = self.init_speed                                               

    # from the frame set the speed of the craft depending on light distance and color
    def observe_and_react(self, frm, x1, y1, x2, y2):
        ra = calc_roi_area(x1, x2, y1, y2)
        self.check_distance(ra)
        roi_frm = img_roi(frm, x1, x2, y1, y2)
        self.check_action(roi_frm)     
        self.change_speed()

    def remote_spd_spt(self, sp):
        self.init_speed = sp