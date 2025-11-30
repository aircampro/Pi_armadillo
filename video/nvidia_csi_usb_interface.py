#!/usr/bin/python
#
# https://github.com/NVIDIA-AI-IOT/jetcam
#
# NVIDIA camera interface for either CSI or USB cameras
#
# git clone https://github.com/NVIDIA-AI-IOT/jetcam
# cd jetcam
# sudo python3 setup.py install
#
import sys
import cv2
from .camera import Camera                                           # from jetcam library on github
import atexit
import numpy as np
import threading
import traitlets

class CSICamera(Camera):
    capture_device = traitlets.Integer(default_value=0)
    capture_fps = traitlets.Integer(default_value=30)
    capture_width = traitlets.Integer(default_value=640)
    capture_height = traitlets.Integer(default_value=480)
    def __init__(self, *args, **kwargs):
        super(CSICamera, self).__init__(*args, **kwargs)
        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)
            re, image = self.cap.read()
            if not re:
                raise RuntimeError('Could not read image from camera.')
        except:
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')
        atexit.register(self.cap.release)

    def __del__(self):
        self.cap.release()

    def _gst_str(self):
        return 'nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                self.capture_device, self.capture_width, self.capture_height, self.capture_fps, self.width, self.height)
    def _read(self):
        re, image = self.cap.read()
        if re:
            return image
        else:
            raise RuntimeError('Could not read image from camera')

class USBCamera(Camera):

    capture_fps = traitlets.Integer(default_value=30)
    capture_width = traitlets.Integer(default_value=640)
    capture_height = traitlets.Integer(default_value=480)   
    capture_device = traitlets.Integer(default_value=0)
    def __init__(self, *args, **kwargs):
        super(USBCamera, self).__init__(*args, **kwargs)
        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)
            re , image = self.cap.read()  
            if not re:
                raise RuntimeError('Could not read image from camera.')    
        except:
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')
        atexit.register(self.cap.release)

    def __del__(self):
        self.cap.release()

    def _gst_str(self):
        return 'v4l2src device=/dev/video{} ! video/x-raw, width=(int){}, height=(int){}, framerate=(fraction){}/1 ! videoconvert !  video/x-raw, format=(string)BGR ! appsink'.format(self.capture_device, self.capture_width, self.capture_height, self.capture_fps)
    def _read(self):
        re, image = self.cap.read()
        if re:
            image_resized = cv2.resize(image,(int(self.width),int(self.height)))
            return image_resized
        else:
            raise RuntimeError('Could not read image from camera')

if __name__ == "__main__":
    if len(sys.argv[0]) >= 1:
        if sys.argv[1] == "csi":
            camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
        elif sys.argv[1] == "usb":
            camera = USBCamera(capture_device=1)
    else:
        camera = USBCamera(capture_device=1)    

    while True:
        image = camera.read()
        cv2.imshow('nvidia image', image)	
        # q will exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    del camera
    cv2.destroyAllWindows()

