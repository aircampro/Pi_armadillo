#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# Library to convert xyY color space to RGB and CT to RGB
#
class RGB:                      
    def __init__(self,r=0,g=0,b=0):
        self.r = r
        self.g = g
        self.b = b
	
def XYToRgb(Level, currentX, currentY):
    # convert xyY color space to RGB

    # https://www.easyrgb.com/en/math.php
    # https://en.wikipedia.org/wiki/SRGB
    # refer https://en.wikipedia.org/wiki/CIE_1931_color_space#CIE_xy_chromaticity_diagram_and_the_CIE_xyY_color_space

    # The currentX/currentY attribute contains the current value of the normalized chromaticity value of x/y.
    # The value of x/y shall be related to the currentX/currentY attribute by the relationship
    # x = currentX/65536
    # y = currentY/65536
    # z = 1-x-y
    rgb = RGB()
    x = currentX / 65535.0
    y = currentY / 65535.0
    z = 1.0 - x - y

    # Calculate XYZ values

    # Y - given brightness in 0 - 1 range
    Y = Level / 254.0
    X = (Y / y) * x
    Z = (Y / y) * z

    # X, Y and Z input refer to a D65/2° standard illuminant.
    # sR, sG and sB (standard RGB) output range = 0 ÷ 255
    # convert XYZ to RGB - CIE XYZ to sRGB
    r = (X * 3.2410) - (Y * 1.5374) - (Z * 0.4986)
    g = -(X * 0.9692) + (Y * 1.8760) + (Z * 0.0416)
    b = (X * 0.0556) - (Y * 0.2040) + (Z * 1.0570)

    # apply gamma 2.2 correction
    if (r <= 0.00304):
        r = 12.92 * r 
    else:
        r = (1.055) * pow(r, (1.0 / 2.4)) - 0.055
    if (g <= 0.00304):
        g = 12.92 * g 
    else:
        g = (1.055) * pow(g, (1.0 / 2.4)) - 0.055
    if (b <= 0.00304):
        b = 12.92 * b 
    else:
        b = (1.055) * pow(b, (1.0 / 2.4)) - 0.055

    # Round off
    r = min(max(r,0),1)
    g = min(max(g,0),1)
    b = min(max(b,0),1)

    # these rgb values are in  the range of 0 to 1, convert to limit of HW specific LED
    rgb.r = (r * 255)
    rgb.g = (g * 255)
    rgb.b = (b * 255)

    return rgb


def CTToRgb(ct_ctMireds):
    rgb = RGB()

    # Algorithm credits to Tanner Helland: https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
    #
    # Convert Mireds to centiKelvins. k = 1,000,000/mired
    ctCentiKelvin = 10000.0 / ct_ctMireds

    # Red
    if (ctCentiKelvin <= 66) :
        r = 255
    else:
        r = 329.698727446 * pow(ctCentiKelvin - 60, -0.1332047592)

    # Green
    if (ctCentiKelvin <= 66) :
        g = 99.4708025861 * log(ctCentiKelvin) - 161.1195681661
    else:
        g = 288.1221695283 * pow(ctCentiKelvin - 60, -0.0755148492)

    # Blue
    if (ctCentiKelvin >= 66) :
        b = 255
    else:
        if (ctCentiKelvin <= 19) :
            b = 0
        else:
            b = 138.5177312231 * log(ctCentiKelvin - 10) - 305.0447927307
    }
    rgb.r = min(max(r,0),255)
    rgb.g = min(max(g,0),255)
    rgb.b = min(max(b,0),255)

    return rgb

# resize the image to the screen size 
#    
import os
from Xlib import display
import cv2

# Set the DISPLAY environment variable if not already set
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'

def get_screen_resolution():
    screen = display.Display().screen()
    width = screen.width_in_pixels
    height = screen.height_in_pixels
    return width, height

def fit_image_to_screen(img: cv2.typing.MatLike, ratio: float=0.75) -> cv2.typing.MatLike:
    """
    Resizes an image to fit the screen resolution.

    Args:
        img (MatLike): The image to resize.
        ratio (float): The ratio of the screen resolution to use for resizing the image.

    Returns:
        MatLike: The resized image.
    """

    screen_width, screen_height = get_screen_resolution()
    img_width = int(ratio * screen_width)
    img_height = int(ratio * screen_height)
    return cv2.resize(img, [img_width, img_height])