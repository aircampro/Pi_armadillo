#!/usr/bin/python
#
# analysis of particles in water
# pip install opencv-python
# pip install scipy
#
import cv2
import numpy as np
import scipy.ndimage as ndimage
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import sys

def Gaussian(x, *params):
    amp = params[0]
    wid = params[1]
    ctr = params[2]
    y = amp * np.exp( -((x - ctr)/wid)**2)
    return y

if __name__ == "__main__":

    # allow passing of file name as the first arguemnt to the program
    # or pass a second arg after the file_name to take the picture using a RoboDK camera
    #
    if len(sys.argv) == 2:
        img_raw = str(sys.argv[1])     
    elif len(sys.argv) == 3:    
        from robodk import robolink                                                        # RoboDK API
        from robodk import robomath                                                        # Robot toolbox
        RDK = robolink.Robolink()  
        cam_item = RDK.Item(“Camera1”)                                                     # connect to camera name shown in RoboDK console
        if not cam_item.Valid():
            print(‘No camera found by RoboDK SDK ..’)
            sys.exit()  
        snapshot_file=str(sys.argv[1])
        ret=RDK.Cam2D_Snapshot(snapshot_file, cam_item)                                    # take a picture
        img_raw = cv2.imread(snapshot_file, 1)        
    else: 
        img_raw = cv2.imread('800px-ColloidCrystal_10xBrightField_GlassInWater.jpg', 1)
    img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)

    # Before binaryization, blur with Gaussian Blur to remove noise.
    # Also, the particle size after binarization is a little smaller than the original image, so I expanded it a little with the morphology transform.
    # (It is better to adjust this with the value of Threshold)
    # Also, the size of the particles is quite small, so we enlarge the image by 3 times ahead of time.
    # (This time, the image is originally beautiful, so I hardly use filters)

    # get width and height from image
    h, w = img.shape

    # magnify by 3
    mag = 3
    img = cv2.resize(img, (w*mag, h*mag))

    # apply gaussian blur
    img_blur = cv2.GaussianBlur(img,(5,5),0)

    # apply otsu
    ret,th = cv2.threshold(img_blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    # dilate
    kernel = np.ones((3,3),np.uint8)
    th = cv2.dilate(th,kernel,iterations = 1)

    # save image
    cv2.imwrite('thresholds.png', th)

    # The above binarization seems to be working quite well, but some of the particles have become donut-shaped.
    # To fill this, we perform the Fill Holes treatment.
    # The Fill Holes handling is not implemented in OpenCV (probably), so we use the one implemented in a module called ndimage in scipy. It's easy to use.

    # Fill Holes
    th_fill = ndimage.binary_fill_holes(th).astype(int) * 255
    cv2.imwrite('thresholds_fill.png', th_fill)

    # Now that the binary is now complete, we will use OpenCV to detect the boundary.
    # For more information on boundary detection, see the tutorial.

    # find contours
    __, cnt, __ = cv2.findContours(th_fill.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_raw = cv2.resize(img_raw, (w*mag, h*mag))
    img_cnt = cv2.drawContours(img_raw, cnt, -1, (0,255,255), 1)
    cv2.imwrite('cnt.png', img_cnt)
    # measure
    # Area
    # Circularity
    # Equivalent Diameter
    # Circularity is a value between 0~1, and 1 indicates that it is a perfect circle.
    Areas = []
    Circularities = []
    Eq_diameters = []

    for i in cnt:
        # area (px*px)
        area = cv2.contourArea(i)
        Areas.append(area)
        #circularity
        arc = cv2.arcLength(i, True)
        circularity = 4 * np.pi * area / (arc * arc)
        Circularities.append(circularity)
        # diameter
        eq_diameter = np.sqrt(4*area/np.pi)
        Eq_diameters.append(eq_diameter)
    print("mean area ",np.mean(Areas)," SD ",np.std(Areas))
    print("mean Circularities ",np.mean(Circularities)," SD ",np.std(Circularities))
    print("mean diameters ",np.mean(Eq_diameters)," SD ",np.std(Eq_diameters))
    fig = plt.figure(figsize=(8,6))
    plt.subplot(2,2,1)
    plt.title("Areas (px^2)")
    plt.hist(Areas, bins=25, range=(0,150), rwidth=0.7)
    plt.subplot(2,2,2)
    plt.title("Circularity")
    plt.hist(Circularities, bins=25, range=(0.5,1), rwidth=0.7)
    plt.subplot(2,2,3)
    plt.title("Equal Diameters (px)")
    plt.hist(Eq_diameters, bins=25, range=(3.0, 15.0), rwidth=0.7)

    hist = np.histogram(Eq_diameters, bins=30, range=(3.0, 15.0))
    diff = np.diff(hist[1])
    hist_center = hist[1][:-1] + diff

    guess = [5000, 4, 9]
    params, cov = curve_fit(Gaussian, hist_center, hist[0], p0=guess)

    fmhm = 2 * np.sqrt(2*np.log(2)) * params[1]
    print("Amplitude = "+str(params[0]))
    print("FMHM = "+str(fmhm))
    print("Mean = "+str(params[2]))

    plt.title("Equal Diameters (px)")
    plt.bar(hist_center, hist[0], width=0.3)
    x = np.linspace(5,15, 1000)
    plt.plot(x, Gaussian(x, *params), c="red",lw=2, ls="--")

