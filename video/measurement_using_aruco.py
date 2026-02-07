#!/usr/bin/env python
#
# Either print the 4 aruco markers or measure the rectangle dimensions and object inside the markers
# 
# <prog> [ print = print markers, measure = measure the rectangle ] width height aspect-ratio
#
import cv2
import numpy as np
import sys
rs = "imutil"                                                                                                  # we will use imutil to re-size image if needed
if rs == "imutil":
    import imutils

args = sys.argv
if len(args) > 1:
    print_opt=not str(args[1]).find("print") == -1
else:
    print_opt=True

if print_opt == True:                                                                                        # if you specify print it makes your 4 markers to print else measure
    aruco = cv2.aruco
    p_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker =  [0] * 4 
    for i in range(len(marker)):
        marker[i] = aruco.drawMarker(p_dict, i, 75) # 75x75 px
        cv2.imwrite(f'marker{i}.png', marker[i])
        print(f"made marker{i}.png")
    sys.exit(1)		
else: 
    blur = 11 
    if len(args) > 4:                                                                                        # read distance from markers and aspect ratio from command line
        x_dis, y_dis = (int(args[2]),int(args[3]))                                                           # AR marker actual size 150mm apart in a square
        size= int(args[4])                                                                                   # Display image size = actual size × size between AR markers aspect ratio
    else:
        x_dis, y_dis = (150,150)                                                                             # AR marker actual size 150mm apart in a square
        size= 3.33333                                                                                        # Display image size = actual size × size between AR markers [wxh 500x500]
    th = 130                                                                                                 # The threshold value of the initial value
    width, height = (x_dis*size,y_dis*size)                                                                  # Image size after deformation
    x_ratio = width/x_dis
    y_ratio = height/y_dis
			
    cap = cv2.VideoCapture(0)                                                                                # capture from usb cam 0
    if not cap.isOpened():
        print("Could not open video device")
        sys.exit(-1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)                                                                 # set camera to desired width and height
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    aruco = cv2.aruco

    def nothing(x):
        pass

    cv2.namedWindow('binary')
    cv2.createTrackbar('threshold','binary', th , 256, nothing)                                             # create a trackbar you can change the ammount by 

    while True:
        try:
            ret, img = cap.read()                                                                           # read frame from the camera
            if ret == None:
                continue
            w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)                                                           # get width and height from camera
            h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            if w != width and h != height:
                if rs == "imutil":
                    img = imutils.resize(img, width=width, height=height)                                       # try resize of image
                else:
                    img = cv2.resize(img, None, fx=width/w, fy=height/h, interpolation=cv2.INTER_AREA)
            p_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)                                       # ArUco The argument selects a dictionary that has a marker with a 4x4 fill pattern inside the square that can be used up to 50 times
            corners, ids, rejectedImgPoints = aruco.detectMarkers(img, p_dict)                              # extract corner co-ord for each marker and the id
                                       
            # Store the coordinates of the displayed images in m in clockwise order from top left
            m = np.empty((4,2))                                                                             # [x,y] array of 4 points
            corners2 = [np.empty((1,4,2))]*4
            for i,c in zip(ids.ravel(), corners):
                corners2[i] = c.copy()
            m[0] = corners2[0][0][2]                                                                        # Lower right of marker 0
            m[1] = corners2[1][0][3]                                                                        # Lower right of marker 1
            m[2] = corners2[2][0][0]                                                                        # Lower right of marker 2
            m[3] = corners2[3][0][1]                                                                        # Lower right of marker 3

            width, height = img.shape[:2]                                                                   # Image size read back
            x_ratio = width/x_dis
            y_ratio = height/y_dis

            marker_coordinates = np.float32(m)
            true_coordinates   = np.float32([[0,0],[width,0],[width,height],[0,height]]) 
            mat = cv2.getPerspectiveTransform(marker_coordinates,true_coordinates)                          # Adjust the image size to any size
            img_trans = cv2.warpPerspective(img,mat,(width, height))
            tmp = img_trans.copy()

            # grayscale
            tmp = cv2.cvtColor(tmp, cv2.COLOR_BGR2GRAY)
            cv2.imshow('gray',tmp)

            # blur
            tmp = cv2.GaussianBlur(tmp, (blur, blur), 0)
            cv2.imshow('blur',tmp)

            # Binarization process
            gamma = cv2.getTrackbarPos('threshold','binary')
            th = cv2.getTrackbarPos('threshold','binary')
            _,tmp = cv2.threshold(tmp,th,255,cv2.THRESH_BINARY_INV)
            cv2.imshow('binary',tmp)

            # ========== look for objects(s) inside of the markers =============
            # blob detection
			t= tmp.copy()
            n, img_label, data, center = cv2.connectedComponentsWithStats(t)

            # Organize detection results
            detected_obj = list()                                                                  # Where to store detection results
            tr_x = lambda x : x * x_dis / width                                                    # X-axis portrait coordinates → portrait coordinates 
            tr_y = lambda y : y * y_dis / height                                                   # Y-axis
            img_trans_marked = img_trans.copy()
            for i in range(1,n):
                x, y, w, h, size = data[i]
                if size < 400 :                                                                    # The area is 400px, not full, ignore it
                    continue
                detected_obj.append( dict( x = tr_x(x), y = tr_y(y), w = tr_x(w), h = tr_y(h), cx = tr_x(center[i][0]), cy = tr_y(center[i][1])))  
                cv2.rectangle(img_trans_marked, (x,y), (x+w,y+h),(0,255,0),2)
                cv2.circle(img_trans_marked, (int(center[i][0]),int(center[i][1])),5,(0,0,255),-1)
            # display
            cv2.imshow('object inside markers',img_trans_marked)
            for i, obj in enumerate(detected_obj,1) :
                print(f'Detection object {i} of the center position X={obj["cx"]:>3.0f}mm Y={obj["cy"]:>3.0f}mm ')

            # ========== measure rectangle inside of the markers =============	
            # Contour detection for calculating size of rectangle inside markers
            contours, hierarchy = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            rect = cv2.minAreaRect(contours[0])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            n, img_label, data, center = cv2.connectedComponentsWithStats(tmp)

            x, y = box[0]
            center =int(center[0][0]),int(center[0][1])
            angle = rect[2]
            scale = 1.0
            mat = cv2.getRotationMatrix2D(center, angle , scale)
            img_trans = cv2.warpAffine(img_trans, mat , (width,height))                               #Affine transformation

            color_lower = np.array([0, 0, 0])                                                         # Lower limit of colors to extract (BGR format)
            color_upper = np.array([0, 0, 0])                                                         # Upper limit of colors to extract (BGR format)
            img_mask = cv2.inRange(img_trans, color_lower, color_upper)                               # Create a mask image from a range
            img_trans = cv2.bitwise_not(img_trans, img_trans, mask=img_mask)                          # Calculation of original image and mask image (white background)
            img_trans_mesure = img_trans.copy()
            img_trans = cv2.cvtColor(img_trans, cv2.COLOR_BGR2GRAY)

            # Blurring
            img_trans = cv2.GaussianBlur(img_trans, (blur, blur), 0)

            # Binarization process
            gamma = cv2.getTrackbarPos('threshold','binary')
            th = cv2.getTrackbarPos('threshold','binary')
            _,img_trans = cv2.threshold(img_trans,th,255,cv2.THRESH_BINARY_INV)
            contours, hierarchy = cv2.findContours(img_trans, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            x, y, w, h = cv2.boundingRect(contours[0])
            img_trans_mesure = cv2.rectangle(img_trans_mesure, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
            cv2.putText(img_trans_mesure, "width={:.1f}mm".format(w/x_ratio),(int(0), int(30)), cv2.FONT_HERSHEY_SIMPLEX,0.65, (0, 0, 255), 2)
            cv2.putText(img_trans_mesure, "height={:.1f}mm".format(h/y_ratio),(int(0), int(50)), cv2.FONT_HERSHEY_SIMPLEX,0.65, (0, 0, 255), 2)
       
            cv2.imshow('raw',img)
            cv2.imshow('image',img_trans_mesure)
            print(w/x_ratio,h/y_ratio)
     
            key = cv2.waitKey(1)
            if key == ord("q"):
                break

        # Ignore errors when AR markers are hidden
        except ValueError:
            print("ValueError")
        except IndexError:
            print("IndexError")
        except AttributeError:
            print("AttributeError")

    cap.release()
    cv2.destroyAllWindows()
