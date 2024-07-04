#!/usr/bin/python3
# use ricoh theta to compare a picture with your reference image
# this could be a defect on a product we are looking for or the spacial environment we are within.
#
from ricoh_theta import Theta

import cv2
import sys
import numpy as np

# UPDATE ME!! This should be the SSID for the Ricoh Theta that is attempting to be queried.
SSID = 'THETAYN14103427'

# matching picture threshold
match_thresh=3000

if __name__ == "__main__":

    # read the refrence image to check against
    img1 = cv2.imread(sys.argv[1])

    # now initialise the camera interface set client_mode=True to use client mode network settings
    camera = Theta(theta_ssid=SSID, client_mode=False, show_state_at_init=False)
    camera.showState()
    camera.takePicture()
    camera.downloadLastImage()

    gen = camera.yieldLivePreview()
    jpg = next(gen)
    with open('ricoh_theta_live_preview.jpg', 'wb') as f:
        f.write(jpg)
    gen.close()

    # read the picture taken for comparision
    img2 = cv2.imread('ricoh_theta_live_preview.jpg')

    # check that we have a ht and wdth for the image of reference
    try:
        W = img1.shape[1]
        H = img1.shape[0]
    except:
        import sys
        print("Error:", sys.exc_info()[0])
        print(sys.exc_info()[1])
        import traceback
        print(traceback.format_tb(sys.exc_info()[2]))
        
    # re-size camera photo if needed to that of the reference image
    if (not (W == img2[1]) or not (H == img2[0])): 
        img2 = cv2.resize(img2, (W, H))
        
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY).astype(np.float32)

    # phase correct
    (x, y), r = cv2.phaseCorrelate(gray1, gray2)
    print(x, y, r)

    if x < 0:
        x11, x12 = abs(int(x)), W
        x21, x22 = 0, W-abs(int(x))
    else:
        x21, x22 = abs(int(x)), W
        x11, x12 = 0, W-abs(int(x))
 
    if y < 0:
        y11, y12 = abs(int(y)), H
        y21, y22 = 0, H-abs(int(y))
    else:
        y21, y22 = abs(int(y)), H
        y11, y12 = 0, H-abs(int(y))
 
    print(W, x12-x11, x22-x21)
    cv2.imwrite("out1.jpg", img1[y11:y12, x11:x12])
    cv2.imwrite("out2.jpg", img2[y21:y22, x21:x22])    

    diff_pic = cv2.absdiff(img1[y11:y12, x11:x12], img2[y21:y22, x21:x22])
    cv2.imwrite("diffs.jpg", diff_pic)   

    # check the diff picture and see how matched (should be black when both are matching)
    cc=0   
    for i in diff_pic:
        for e in i:
            for rgb in e:
                if rgb > 0:
                    cc = cc + 1
    if (cc > match_thresh) :
        print("defects found")
    else:
        print("no defects")    