#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================================================================
#
# Tello drone moving to oolor of object selected
#
# ============================================================================================================================
import tello        
import time         
import cv2                                        # OpenCV
import numpy as np

LOW_COLOR1 = np.array([0, 50, 50])                # color selection range hardcoded
HIGH_COLOR1 = np.array([6, 255, 255])             
LOW_COLOR2 = np.array([174, 50, 50])
HIGH_COLOR2 = np.array([180, 255, 255])

W_S=480
H_S=360

# main program
def main():
    # Connect to the tello drone
    drone = tello.Tello('', 8889, command_timeout=.01)  

    current_time = time.time()  # get current time
    pre_time = current_time     # set pretime to current

    time.sleep(0.5)         

    # create a named cv window
    cv2.namedWindow("OpenCV Window")

    # dummy call back fro trackbar 
    def nothing(x):
        pass
    
    # define two trackbars for the 2 hsv ranges used in color selection
    cv2.createTrackbar("H_min", "OpenCV Window", 0, 179, nothing)
    cv2.createTrackbar("H_max", "OpenCV Window", 9, 179, nothing)       
    cv2.createTrackbar("S_min", "OpenCV Window", 128, 255, nothing)
    cv2.createTrackbar("S_max", "OpenCV Window", 255, 255, nothing)
    cv2.createTrackbar("V_min", "OpenCV Window", 128, 255, nothing)
    cv2.createTrackbar("V_max", "OpenCV Window", 255, 255, nothing)

    cv2.createTrackbar("H2_min", "OpenCV Window", 0, 179, nothing)
    cv2.createTrackbar("H2_max", "OpenCV Window", 9, 179, nothing)       
    cv2.createTrackbar("S2_min", "OpenCV Window", 128, 255, nothing)
    cv2.createTrackbar("S2_max", "OpenCV Window", 255, 255, nothing)
    cv2.createTrackbar("V2_min", "OpenCV Window", 128, 255, nothing)
    cv2.createTrackbar("V2_max", "OpenCV Window", 255, 255, nothing)

    cv2.createTrackbar("P_CONTROL", "OpenCV Window", 0, 10, nothing)
    
    # define flags for drone movement and object color selection    
    flag = 0
    use_trackbar = 0
    
    try:
        while True:

            frame = drone.read()                                                               # read the tello drone frame from the onboard camera
            if frame is None or frame.size == 0:                                               # continue if we didnt get a frame
                continue 

            img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)                                       # convert RGB to BGR
            if resize == 1:
                img = cv2.resize(img, dsize=(W_S,H_S) )                                        # resize if asked to
            img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)                                     # RGB => YUV(YCbCr)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))                         # Create a clahe object
            img_yuv[:,:,0] = clahe.apply(img_yuv[:,:,0])                                       # Histogram equalization for luminance only
            img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)                                     # YUV => RGB
    
            img_blur = cv2.blur(img, (15, 15))                                                 # Apply a smoothing filter     
            hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)                                    # BGR to HSV

            # define 2 trackbars for the color ranges
            h_min = cv2.getTrackbarPos("H_min", "OpenCV Window")
            h_max = cv2.getTrackbarPos("H_max", "OpenCV Window")
            s_min = cv2.getTrackbarPos("S_min", "OpenCV Window")
            s_max = cv2.getTrackbarPos("S_max", "OpenCV Window")
            v_min = cv2.getTrackbarPos("V_min", "OpenCV Window")
            v_max = cv2.getTrackbarPos("V_max", "OpenCV Window")

            h2_min = cv2.getTrackbarPos("H2_min", "OpenCV Window")
            h2_max = cv2.getTrackbarPos("H2_max", "OpenCV Window")
            s2_min = cv2.getTrackbarPos("S2_min", "OpenCV Window")
            s2_max = cv2.getTrackbarPos("S2_max", "OpenCV Window")
            v2_min = cv2.getTrackbarPos("V2_min", "OpenCV Window")
            v2_max = cv2.getTrackbarPos("V2_max", "OpenCV Window")

            pb = cv2.getTrackbarPos("P_CONTROL", "OpenCV Window")
            if pb == 0:
                pb = 1.0
                
            # Use the inRange function to specify the range for binarization
            if use_trackbar == 1:
                bin_img1 = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))          # look for the range of color
                bin_img2 = cv2.inRange(hsv, (h2_min, s2_min, v2_min), (h2_max, s2_max, v2_max))
            else:            
                bin_img1 = cv2.inRange(hsv, LOW_COLOR1, HIGH_COLOR1) 
                bin_img2 = cv2.inRange(hsv, LOW_COLOR2, HIGH_COLOR2)

            # Mask the original image with bitwise_and -> Only the colors in the masked areas remain                
            mask = bin_img1 + bin_img2                                                             # combine the 2 color images
            masked_img = cv2.bitwise_and(img_blur, img_blur, mask= mask)                           # create mask 

            out_img = masked_img

            # Perform labeling processing with area and center of gravity calculation            
            num_labels, label_img, stats, centroids = cv2.connectedComponentsWithStats(mask)      

            # The largest label is unnecessary as it is black and covers the entire screen. Delete data
            num_labels = num_labels - 1
            stats = np.delete(stats, 0, 0)
            centroids = np.delete(centroids, 0, 0)

            if num_labels >= 1:  
                # Get the index with the largest area
                max_index = np.argmax(stats[:, 4])                                                
                # Get the x, y, w, h, area s, and center of gravity mx, my of the label with the largest area
                x = stats[max_index][0]
                y = stats[max_index][1]
                w = stats[max_index][2]
                h = stats[max_index][3]
                s = stats[max_index][4]
                mx = int(centroids[max_index][0])                                                                # X coordinate of center of gravity
                my = int(centroids[max_index][1])                                                                # Y coordinate of the center of gravity
                cv2.rectangle(out_img, (x, y), (x+w, y+h), (255, 0, 255))                                        # Draw a bounding box around the label
                cv2.putText(out_img, "%d,%d"%(mx, my), (x-15, y+h+15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0)) 
                cv2.putText(out_img, "%d"%(s), (x, y+h+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0))                       

                if flag == 1:
                    a = b = c = d = 0
                    # P control formula (Kp gain is read from the trackbar)
                    dx = pb * (240 - mx)                                                                        # Difference from the screen center

                    # Set dead zone in turning direction
                    d = 0.0 if abs(dx) < 50.0 else dx                                                            # Set to zero if less than ±50

                    # Software limiter for rotation direction (not to exceed ±100)
                    d = -d
                    d =  100 if d >  100.0 else d
                    d = -100 if d < -100.0 else d

                    print('dx=%f'%(dx) )
                    drone.send_command('rc %s %s %s %s'%(int(a), int(b), int(c), int(d)) )
            else:
                print("Target not found!!")
                
            cv2.imwrite("out_img.jpg", out_img)             
            cv2.imshow('OpenCV Window', out_img)  

            # read keyboard
            key = cv2.waitKey(1)
            if key == 27:                   # ESC to exit
                break
            elif key == ord('t'):
                drone.takeoff()            
            elif key == ord('l'):
                drone.land()                
            elif key == ord('w'):
                drone.move_forward(0.3)     
            elif key == ord('s'):
                drone.move_backward(0.3)    
            elif key == ord('a'):
                drone.move_left(0.3)        
            elif key == ord('d'):
                drone.move_right(0.3)       
            elif key == ord('q'):
                drone.rotate_ccw(20)        
            elif key == ord('e'):
                drone.rotate_cw(20)         
            elif key == ord('r'):
                drone.move_up(0.3)          
            elif key == ord('f'):
                drone.move_down(0.3)        
            elif key == ord('1'):
                flag = 1                    # Tracking mode ON
            elif key == ord('2'):
                flag = 0                    # Tracking mode OFF
            elif key == ord('t'):
                use_trackbar = 1           # use trackbar for color selction
            elif key == ord('c'):
                use_trackbar = 0           # use program defined hsv range of colors
                
            # (Z) Send 'command' every 5 seconds and check if alive
            current_time = time.time()         
            if current_time - pre_time > 5.0 :  # Has more than 5 seconds passed 
                drone.send_command('command')   
                pre_time = current_time         # Update last time

    except( KeyboardInterrupt, SystemExit):     # Ctrl+c
        print( "\033[31m SIGINT interrupt \033[0m" )

    drone.send_command('streamoff')
    # delete tello instance
    del drone

# main program
if __name__ == "__main__":      
    main()                      