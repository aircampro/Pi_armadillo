#!/usr/bin/python
#
# Example : hough circles to estimate circle radius
#
import cv2
import numpy as np
import sys

RESIZE = False                                     # set True if you want to re-size the input frame
frameWidth = 640
frameHeight = 480

def empty(x):
    pass

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
        cap = cv2.VideoCapture(PRT)

    # Circle Detection Process
    # 1. Color -> GrayScale
    # 2. Gaussian Blur :: parameter -> kernel
    # 2.5. Canny Edge Detection
    # 3. Hough Transform
    #
    while True :

        if FRAME_IN == "video_feed":
            ret, img_src = cap.read()
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
                cv2.circle(img_dst, (circle[0], circle[1]), circle[2], (0, 165, 255), 5)
                print('radius')
                print(circle[2])
                arr.append(float(circle[2]))			
                # print the circle center
                cv2.circle(img_dst, (circle[0], circle[1]), 2, (0, 0, 255), 3)
                print('center')
                print(circle[0], circle[1])  
            print("avg radius ",sum(arr)/len(arr))
            print("std dev ",np.std(arr))			
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

