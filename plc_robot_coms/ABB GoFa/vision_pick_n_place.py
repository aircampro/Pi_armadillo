#!/usr/bin/env python
#
# an openCV application to look at the camera image and send to the pick and place ABB GoFa cobot
#
import cv2
import numpy as np
import socket
import time
import math

# Define the port on which you want to connect to the ABB GoFa Cobot
port = 1025
goFa_IP = '192.168.98.94'

# Create a socket object 
s = socket.socket() 
# connect to the server on local computer 
#s.connect(('127.0.0.1', port))
print("connecting...")
try:
    s.connect((goFa_IP, port)) 
except:
    while True:
        if (s.connect((goFa_IP, port)) != True ):
            break
        print("trying to connect")
print("connected...")
		
s.sendall('Server connected'.encode())

# function to look at the objects from the camera using OpenCV functions
#
def do_contour_analysis(frame, c):
    M = cv2.moments(c)
    # Calculate centroid coordinates from moment
    if (M["m00"]==0): M["m00"]=1
    x = int(M["m10"]/M["m00"])
    y = int(M['m01']/M['m00'])
    print('Centroid coordinates (x,y): ({}, {})'.format(x, y))
    newContor = cv2.convexHull(c)
    cv2.circle(frame,(x,y),7,(0,255,0),-1)
    cv2.putText(frame,'{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
    cv2.drawContours(frame, [newContor], 0, color, 3)
    coorX = ''
    coorY = ''
    if x < 100: coorX = '0'+str(x)
    else: coorX = str(x)
    if y < 100: coorY = '0'+str(y)
    else: coorY = str(y)
    coordXY = coorX+coorY
    # send the data to the GoFa via the gloabl socket we opened to it
    s.sendall(coordXY.encode())
    time.sleep(1)
    print("Coordinates: " + coordXY)
    # now lets do some more vision to see its results on various objects
    # Output area
    area = M['m00']
    print('Area: ', area)
    # Calculate rotation angle from moment
    angle = math.degrees(0.5 * math.atan(2 * M['m11'] / (M['m20'] - M['m02'])))
    print('Rotation angle: {} degrees'.format(angle))
    # Semi-major and semi-minor axes of the ellipse: The semi-major and semi-minor axes of the ellipse are determined from the moments of the 0th and 2nd order.
    x_direction_radius = math.sqrt(4.0 * M['m20']/ M['m00'])
    y_direction_radius = math.sqrt(4.0 * M['m02']/ M['m00'])
    print('Direction Radius (x,y): ({}, {})'.format(x_direction_radius, y_direction_radius))
    perimeter = cv2.arcLength(c, True)
    print('The perimeter (or length) surrounding the area : ({})'.format(perimeter))
    hull = cv2.convexHull(c)
    print('Convex Hull : ({})'.format(hull))
    # circle function show it in a diiferent color
    (x,y),radius = cv2.minEnclosingCircle(c)
    center = (int(x),int(y))
    radius = int(radius)
    print('Center (x,y) Radius: ({}, {})'.format(center, radius))
    frame = cv2.circle(frame,center,radius,(255,0,255),2)
    # straight line fir to object
    rows,cols = frame.shape[:2]
    [vx,vy,x,y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    print('Line fit: ({}, {})'.format(lefty, righty))
    frame = cv2.line(frame,(cols-1,righty),(0,lefty),(90,105,100),2)
            
def look_at_object(mask, color):
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area > 3000:
            do_contour_analysis(frame, c)
			
cap = cv2.VideoCapture(0)
# define the colors and masks for the objects
BlueLow = np.array([100,100,20],np.uint8)
BlueHigh = np.array([125,255,255],np.uint8)
YelloLow = np.array([15,100,20],np.uint8)
YelloHigh = np.array([45,255,255],np.uint8)
redLow1 = np.array([0,100,20],np.uint8)
redHigh1 = np.array([5,255,255],np.uint8)
redLow2 = np.array([175,100,20],np.uint8)
redHigh2 = np.array([179,255,255],np.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX

# set the program actions default to perform both analysis on the image
gs = 1
colr = 2
action_word = gs | colr

while True:
    ret,frame = cap.read()

    if ret == True:
        if (action_word and gs) == gs:
            # use grayscale image
            img = cv2.cvtColor(frame, cv2.COLOR_GRAYSCALE)
            ret, thresh = cv2.threshold(img, 127, 255, 0)
            if ret == True:
                do_contour_analysis(img, thresh)
            # show the frames of analysis 
            cv2.imshow('grayscale image',img)
        if (action_word and colr) == colr:
            # use masked images of color
            frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            maskBlue = cv2.inRange(frameHSV,BlueLow,BlueHigh)
            maskYello = cv2.inRange(frameHSV,YelloLow,YelloHigh)
            maskRed1 = cv2.inRange(frameHSV,redLow1,redHigh1)
            maskRed2 = cv2.inRange(frameHSV,redLow2,redHigh2)
            maskRed = cv2.add(maskRed1,maskRed2)
            look_at_object(maskBlue,(255,0,0))
            look_at_object(maskYello,(0,255,255))
            look_at_object(maskRed,(0,0,255))
            # show the frames of analysis
            cv2.imshow('masked color frame',frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('x'):                      # exit
            break
        elif key & 0xFF == ord('b'):                    # toggle on off grayscale analysis
            action_word ^= gs
        elif key & 0xFF == ord('c'):                    # toggle on/off color analysis
            action_word ^= colr
            
# exit and close
s.close()
cap.release()
cv2.destroyAllWindows()