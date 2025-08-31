#!/usr/bin/python
#
# adjust the color within a picture
#
import numpy as np
import cv2
import copy
import sys

def color_adjustment(img, color):
    img = img.astype(np.float)
    # Normalize by maximum and minimum values
    for n, bgr in enumerate(['Blue', 'Green', 'Red']):
        # Prevention of zero discount
        # When min==max, set all to zero
        if color[bgr][0] == color[bgr][1]:
            img[:,:,n] = 0
        else:
            img[:,:,n] = (img[:,:,n] - color[bgr][0])/(color[bgr][1]-color[bgr][0])
    img = np.clip(img, 0, 1) * 255
    return img.astype(np.uint8)
def nothing(x):
    pass
def create_trackbar(default_values):
    # Track bar settings
    # Enter the initial value of BGR in order
    for n, color in enumerate(bgr):
        cv2.createTrackbar('{} (min)'.format(color), 'image', default_values['color'][color][0], 255, nothing)
        cv2.createTrackbar('{} (max)'.format(color), 'image', default_values['color'][color][1], 255, nothing)
def get_trackbar_value(values):
    # Get the current track bar value
    for n, color in enumerate(bgr):
        values['color'][color][0]= cv2.getTrackbarPos("{} (min)".format(color), "image")
        values['color'][color][1]= cv2.getTrackbarPos("{} (max)".format(color), "image")
    return values
def get_default_value(img):
    # Get the maximum and minimum values of the original image
    default_values = {'color':{}}
    for n, color in enumerate(bgr):
        default_values['color'][color] = []
        default_values['color'][color] = [img[:,:,n].min(), img[:,:,n].max()]
    return default_values
def set_default_value(event, x, y, flags, param):
    # Revert to default values 
    default_color = param['color']
    if event == cv2.EVENT_MBUTTONDOWN:
        for n, color in enumerate(bgr):
            cv2.setTrackbarPos("{} (min)".format(color), "image", default_color[color][0])
            cv2.setTrackbarPos("{} (max)".format(color), "image", default_color[color][1])
# ===============
# Initial screen settings
# ===============

if __name__ == "__main__":

    bgr = ['Blue', 'Green', 'Red']
    # Image Loading
    if len(sys.argv) == 2:
        img_raw = str(sys.argv[1])   
    else:
        img_raw = 'IMG004.JPG'
    img = cv2.imread(img_raw)
    h1, w1, _ = img.shape
    # Define the size of the reduced image to be 500pixel in height
    h2 = 500
    resize_rate = h2/h1
    w2 = int(w1 * resize_rate)
    # Image Reduction
    disp_img = cv2.resize(img, (w2, h2))
    disp_img_ = copy.deepcopy(disp_img) # Keep the original image to revert the drawing
    # Set default values
    default_values = get_default_value(disp_img)
    # Image Display
    cv2.namedWindow('image') # Shrink screen
    create_trackbar(default_values)
    values = {'color' : {'Red':[0,255], 'Green':[0,255], 'Blue':[0,255]}}
    while(1):
        # Press the middle button to revert to the default value
        cv2.setMouseCallback("image", set_default_value, default_values)
        # Get the value of the trackbar and convert it to an image
        values = get_trackbar_value(values)
        disp_img = color_adjustment(disp_img_, values['color'])
        disp_img = cv2.hconcat((disp_img_, disp_img))
        cv2.imshow('image', disp_img)
        if cv2.waitKey(20) & 0xFF == 27:
            break  
    cv2.destroyAllWindows()