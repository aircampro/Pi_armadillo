#!/usr/bin/env python
#
# example of use from this library to look at multispectral images https://github.com/spectralpython/spectral/tree/master
#
from spectral import *

img = open_image('92AV3C.lan')

view = imshow(img, (29, 19, 9))

gt = open_image('92AV3GT.GIS').read_band(0)

view = imshow(classes=gt)

view = imshow(img, (30, 20, 10), classes=gt)

view.set_display_mode('overlay')

view.class_alpha = 0.5
ave_rgb('rgb.jpg', img, [29, 19, 9])

# Functions for handling AVIRIS image files.
#
import spectral.io.aviris as aviris
img.bands = aviris.read_aviris_bands('92AV3C.spc')
view_cube(img, bands=[29, 19, 9])

# spectral data chart from lan and gis files 
#
data = open_image('92AV3C.lan').load()
gt = open_image('92AV3GT. GIS').read_band(0)

pc = principal_components(data)

xdata = pc.transform(data)

w = view_nd(xdata[:,:,:15], classes=gt)