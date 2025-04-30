# ============================================================
# image manipulation functions
# ============================================================

# ============================================================
# import packages
# ============================================================
import scipy
from scipy import ndimage
from scipy import misc
import cv2
umport numpy as np

# color definition
RED   = 1
GREEN = 2
BLUE  = 3

# ============================================================
# functions
# ============================================================

# draw a rectangle of the target color 
#
def find_rect_of_target_color(image, color_type):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]

    # red detection
    if color_type == RED:
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[((h < 20) | (h > 200)) & (s > 128)] = 255

    # blue detection
    if color_type == BLUE:
        lower_blue = np.array([130, 50, 50])
        upper_blue = np.array([200, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # green detection
    if color_type == GREEN:
        lower_green = np.array([75, 50, 50])
        upper_green = np.array([110, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

    neiborhood = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)
    mask = cv2.dilate(mask, neiborhood, iterations=2)

    mask = cv2.erode(mask, neiborhood, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        approx = cv2.convexHull(contour)
        rect = cv2.boundingRect(approx)
        rects.append(np.array(rect))
    return rects

# find red green blue colors and draw rectangle round it
# returns marked image and word if color found R=1 B=2 G=4
def find_colors(frame):
    ret = 0
    # red
    rects = find_rect_of_target_color(frame, RED)
    if len(rects) > 0:
        rect = max(rects, key=(lambda x: x[2] * x[3]))
        if rect[3] > 10:                                                                                  # if red circle is one
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
        #for rect in rects:
        #  cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
        ret |= 1
        
    # green
    rects = find_rect_of_target_color(frame, GREEN)
    if len(rects) > 0:
        rect = max(rects, key=(lambda x: x[2] * x[3]))
        if rect[3] > 10:
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 0), thickness=2)
        #for rect in rects:      
        #  cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 0), thickness=2)
        ret |= 2
        
    # blue
    rects = find_rect_of_target_color(frame, BLUE)
    if len(rects) > 0:
        rect = max(rects, key=(lambda x: x[2] * x[3]))
        if rect[3] > 10:
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 0, 0), thickness=2)
        #for rect in rects:
        #  cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 0, 0), thickness=2)
        ret |= 4
        
    return frame, ret

# sharpen the image
#
def sharpness(file_or_frame):
    if len(file_or_frame.split(".")) > 1:           # its a . file
        n=cv2.imread(file_or_frame)
    elif len(file_or_frame.split(".")) == 1:        # its a frame
        n=file_or_frame
    else:
        print("you must pass a file or frame")
        return -1
    nn = n.astype(float)
    blurred_f = ndimage.gaussian_filter(nn, 3)
    filter_blurred_f = ndimage.gaussian_filter(blurred_f, 1)
    alpha = 30
    sharpened = blurred_f + alpha * (blurred_f - filter_blurred_f)

# label shapes in image
def label_image(frame):
    im = ndimage.gaussian_filter(frame, sigma=l/(4.*n))
    mask = im > im.mean()
    label_im, nb_labels = ndimage.label(mask)
    sizes = ndimage.sum(mask, label_im, range(nb_labels + 1))
    mean_vals = ndimage.sum(im, label_im, range(1, nb_labels + 1))
    return label_im, nb_labels, sizes, mean_vals

# get roi from above analysis
def label_image_roi(label_im): 	
    mask_size = sizes < 1000
    remove_pixel = mask_size[label_im]
    label_im[remove_pixel] = 0
    labels = np.unique(label_im)
    label_im = np.searchsorted(labels, label_im)
    slice_x, slice_y = ndimage.find_objects(label_im==4)[0]
    roi = im[slice_x, slice_y]
    return roi     

# write the frame to disk
def write_frame(frame, filenam='/mnt/h/test.jpg'):
    cv2.imwrite(filenam, frame)

# perform sobel edge analysis
def sobel(frame):
   sx = ndimage.sobel(frame, axis=0, mode='constant')
   sy = ndimage.sobel(frame, axis=1, mode='constant')
   return sob = np.hypot(sx, sy)
   
# crop the size of the picture as an even square
def region_crop(frame, mag=4):
    lx, ly = frame.shape
    # Cropping
    return frame[lx / mag: - lx / mag, ly / mag: - ly / mag]
	
# up <-> down flip
def up_down_flip(frame):
    return np.flipud(frame)
	
# rotation
def rotation(frame, angle=45, reshap=True):
    return rotate_face_noreshape = ndimage.rotate(face, angle, reshape=reshap)

# return the mean value of the frame
def take_mean_frame_value(_f):
    return _f.mean()


