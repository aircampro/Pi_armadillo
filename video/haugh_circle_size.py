#!/usr/bin/python
#
# Example : hough circles to estimate circle radius from video or still frame
#
import cv2
import numpy as np
import sys
import subprocess
import sys
import copy

RESIZE = False                                     # set True if you want to re-size the input frame to the below settings
frameWidth = 640
frameHeight = 480

# list of camera parameters openCV enum & v3l2 description as a pair
#
CAM_PARAM = [ (cv2.CAP_PROP_BRIGHTNESS,'brightness'),
 (cv2.CAP_PROP_CONTRAST,'contrast'),
 (cv2.CAP_PROP_SATURATION,'saturation'),
 (cv2.CAP_PROP_HUE,'hue'),
 (cv2.CAP_PROP_GAIN,'gain'),
 (cv2.CAP_PROP_EXPOSURE,'exposure_absolute'),
 (cv2.CAP_PROP_AUTO_EXPOSURE,'exposure_auto'),
 (cv2.CAP_PROP_WB_TEMPERATURE,'white_balance_temperature'),
 (cv2.CAP_PROP_AUTO_WB,'white_balance_temperature_auto'),]

# function to get camera parameters
#
def get_cam_param():
    try:
        subprocess.run(['v4l2-ctl', '–L'], check=True, timeout=3)
    except subprocess.TimeoutExpired:
        print('The process timed out!')
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
    return result.stdout, result.returncode

# function to parse cam params
#
def parse_v4l2_param(res, lookfor="brightness"):
    fnd=False
    for r in res.split(":"):
        if fnd == True:
            l2 = r.split("=")
            min_value = l2[1].split(" ")[0]
            max_value = l2[2].split(" ")[0]
            print("max = ",max_value, "min = ",min_value)	
            fnd = False
        if not r.find(lookfor) == -1:
            fnd = True
    return min_value, max_value

# dummy callback function for the trackbar
#
def empty(x):
    pass

# callback function for recaslling the original camera settings by pressing mouse center button
# or left mouse button to set default params for the hough transdorm
#
def set_original_value(event, x, y, flags, vs):
    if event == cv2.EVENT_MBUTTONDOWN:
        for ii, line in enumerate(CAM_PARAM):                                                                                                     
            cv2.setTrackbarPos(line[1], "Parameters", vs[ii]) 
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.setTrackbarPos('k_size_set', 'Parameters', 3)                         
        cv2.setTrackbarPos('canny_1st', 'Parameters', 80)                       
        cv2.setTrackbarPos('canny_2nd', 'Parameters', 120)                       
        cv2.setTrackbarPos('minDist_set', 'Parameters', 150)
        cv2.setTrackbarPos('param1_set', 'Parameters', 100)                    
        cv2.setTrackbarPos('param2_set', 'Parameters', 30)                   
        cv2.setTrackbarPos('minRadius_set', 'Parameters', 250)                 
        cv2.setTrackbarPos('maxRadius_set', 'Parameters', 500)      
        
if __name__ == "__main__":

    # allow passing of file name as the first arguemnt to the program
    if len(sys.argv) > 1:
        FRAME_IN = str(sys.argv[1])
        if len(sys.argv) > 2:                        # 2nd param is video name or usb port if not 0
            try:
                PRT = int(sys.argv[2])               # port
            except:
                PRT = str(sys.argv[2])               # video name          
        else:
            PRT = 0        
    else:
        FRAME_IN="./my_picture_name.png"

    # create a window to allow change of parameters so you cab get the best result
    cv2.namedWindow('Parameters')
    cv2.resizeWindow('Parameters', width=1000, height=400)
    cv2.createTrackbar('k_size_set', 'Parameters', 1, 10, empty)                         #3
    cv2.createTrackbar('canny_1st', 'Parameters', 90, 500, empty)                        #80
    cv2.createTrackbar('canny_2nd', 'Parameters', 60, 500, empty)                        #120
    cv2.createTrackbar('minDist_set', 'Parameters', 100, 200, empty)
    cv2.createTrackbar('param1_set', 'Parameters', 100, 300, empty)                      #100
    cv2.createTrackbar('param2_set', 'Parameters', 30, 300, empty)                       #30
    cv2.createTrackbar('minRadius_set', 'Parameters',510, 1000, empty)                   #250
    cv2.createTrackbar('maxRadius_set', 'Parameters', 0, 1000, empty )                   #500                                                  
    #inner (minR, maxR) = (172,278)
    #outer (minR, maxR) = (490,570)
    if not FRAME_IN == "video_feed":
        img_src = cv2.imread(FRAME_IN, 1)
    else:
        cap = cv2.VideoCapture(PRT)                                                     # capture incoming frame from camera
        if not cap.isOpened():                                                          # fail then exit
            sys.exit()
        lastvals = []
        res, ret = get_cam_param()                                                       # get paramters from camera using video for linux  
        for line in CAM_PARAM:                                                           # for each listed camera parameter                                                        
            minv, maxv = parse_v4l2_param(res, line[1])                                  # get max and min value
            act_val = cap.get(line[0])                                                   # get actual value
            cv2.createTrackbar(line[1], 'Parameters', minv, maxv, empty)                 # create a trackbar for each one  
            cv2.setTrackbarPos(line[1], "Parameters", act_val)                           # set trackbar to current for bumpless operation        
            lastvals.append(act_val)                                                     # initialize list with each current value
    saved_start = copy.deepcopy(lastvals)                                                # save set-up for re-call
    cv2.setMouseCallback('Parameters', set_original_value, saved_start)                  # set callback for center mouse press to rest the camera to original
    # Circle Detection Process
    # 1. Color -> GrayScale
    # 2. Gaussian Blur :: parameter -> kernel
    # 2.5. Canny Edge Detection
    # 3. Hough Transform
    #
    while True :
       
        if FRAME_IN == "video_feed":
            ret, img_src = cap.read()
            for i, line in enumerate(CAM_PARAM):                                         #create a trackbar for each one
                valu = cv2.getTrackbarPos(line[1], "Parameters")
                if not (lastvals[i] == valu):                                            # look for change of slider
                    cap.set(line[0], valu)                                               # change made then set camera   
                    lastvals[i] = valu                                                   # store changed value

        if not (FRAME_IN == "video_feed") or ret:
            if RESIZE == True:
                img_src = cv2.resize(img_src, (frameWidth, frameHeight))        

            # make a copy of the input image
            img_dst = img_src.copy()  

            # 1. Color -> GrayScale    
            img_gray = cv2.cvtColor(img_src, cv2.COLOR_BGR2GRAY)

            #2. Gaussian blur
            kernel = cv2.getTrackbarPos("k_size_set", "Parameters")
            kernel = (kernel * 2) + 1
            img_blur = cv2.GaussianBlur(img_gray, (kernel, kernel), None)

            #2.5.Canny Edge Detection
            thres1_val = cv2.getTrackbarPos('canny_1st', 'Parameters')
            thres2_val = cv2.getTrackbarPos('canny_2nd', 'Parameters')
            img_edge = cv2.Canny(img_blur, threshold1=thres1_val, threshold2=thres2_val)

            # 3. Hough Transform   
            circles = cv2.HoughCircles(img_edge, cv2.HOUGH_GRADIENT,
                                       dp=1,
                                       minDist=cv2.getTrackbarPos('minDist_set', 'Parameters'),
                                       param1=cv2.getTrackbarPos('param1_set', 'Parameters'),
                                       param2=cv2.getTrackbarPos('param2_set', 'Parameters'),
                                       minRadius=cv2.getTrackbarPos('minRadius_set', 'Parameters'),
                                       maxRadius=cv2.getTrackbarPos('maxRadius_set', 'Parameters'),
                                       )
   
            try:
                circles = np.uint16(np.around(circles))
                arr = []
                for circle in circles[0, :]:
                    # print the circle radius
                    cv2.circle(img_dst, (circle[0], circle[1]), circle[2], (0, 105, 205), 5)
                    print(' radius ')
                    print(circle[2])
                    arr.append(float(circle[2]))                       
                    # print the circle center
                    cv2.circle(img_dst, (circle[0], circle[1]), 2, (0, 0, 255), 3)
                    print(' center ')
                    print(circle[0], circle[1]) 
                print("\033[32m ---------------------------------- \033[0m")                    
                print("avg radius ",sum(arr)/len(arr))
                print("std dev ",np.std(arr))	
                print("max radius ",np.max(arr))
                print("min radius  ",np.min(arr))	                
                # 4. Plotting
                cv2.imshow('result', img_dst)
            except:
                pass

        # q will exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # clean-up
    if FRAME_IN == "video_feed":
        cap.release()
    cv2.destroyAllWindows()

