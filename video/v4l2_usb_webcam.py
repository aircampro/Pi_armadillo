#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# ====================== V4L2 camera interface class ============================= 
#
# A library for accessing USB devices without kernel-mode drivers.
# pyusb https://github.com/pyusb/pyusb
# A library for accessing USB devices in Python. Depends on libusb.
# libusb https://libusb.info/
# A library for accessing USB devices without kernel-mode drivers.
# usbdk https://github.com/daynix/UsbDk
# As the name USB Development Kit suggests, it seems to be a library for 
# communicating directly with USB devices. It can be used as a backend for libusb.
# USBPcap https://desowin.org/usbpcap/
#
# ================================================================================
# 
import numpy as np
import cv2
import yaml
from yaml.loader import SafeLoader
import subprocess
import glob
import sys
import time
import usb.core 
import usb.backend.libusb1
from ctypes import c_void_p, c_int
import argparse

print('cv2.__version__ =', cv2.__version__)
print('sys.version =', sys.version)
print(sys.platform)

def get_calibration_directory(camera_name, image_width, image_height):
    calibration_directory = './calibration_images/' + camera_name + '/' + str(image_width) + 'x' + str(image_height) + '/'
    return calibration_directory

def get_video_devices():
    
    command = 'v4l2-ctl --list-devices'
    lines = subprocess.getoutput(command).split('\n')
    lines = [l.strip() for l in lines if l != '']
    cameras = [l for l in lines if not ('/dev/' in l)]
    devices = [l for l in lines if '/dev/' in l]

    all_camera_devices = {}
    camera_devices = []
    current_camera = None
    for line in lines:
        if line in cameras:
            if (current_camera is not None) and camera_devices:
                all_camera_devices[current_camera] = camera_devices
                camera_devices = []
            current_camera = line
        elif line in devices:
            camera_devices.append(line)
    if (current_camera is not None) and camera_devices:
        all_camera_devices[current_camera] = camera_devices

    return all_camera_devices

# USB Webcam interface class
#
class V4L2UsbWebcam:                      
    def __init__(self,
                 camera_name='Arducam OV9782 USB Camera',
                 fps=30,
                 image_width=640,
                 image_height=480,
                 use_calibration=False,
                 use_second_camera=False,
                 show_images=False,
                 be=False,
                 cam_calib_path=None):
        
        self.show_images = show_images
        self.use_calibration = use_calibration
        
        camera_devices = get_video_devices()
        self.camera_name = camera_name
		
		# examples of camera names
        #self.camera_name = 'Arducam OV9782 USB Camera'
        #self.camera_name = 'Mi USB Webcam HD'
		#self.camera_name = 'Logitech C930e'
		
        first_camera_device = None
        second_camera_device = None
        self.camera_device = None
        self.backe = be
        if self.backe == True:
            self.backend = usb.backend.libusb1.get_backend(find_library=lambda x: "libusb-1.0.dll")
            self.backend.lib.libusb_set_option.argtypes = [c_void_p, c_int]
            self.backend.lib.libusb_set_debug(backend.ctx, 5)
            self.backend.lib.libusb_set_option(backend.ctx, 1)
        for k,v in camera_devices.items():
            if self.camera_name in k:
                if first_camera_device is None:
                    first_camera_device = v[0]
                else: 
                    second_camera_device = v[1]

        if use_second_camera:     
            self.camera_device = second_camera_device
        else:
            self.camera_device = first_camera_device
    
        assert (self.camera_device is not None), ('ERROR: Webcam did not find the specified camera, self.camera_name = \"' + str(self.camera_name) + '\" Do you have v4l2-ctl installed? Run \"v4l2-ctl --list-devices\" to check your devices and if v4l2-ctl is installed.') 

        self.use_logitech_c930 = 'C930e' in self.camera_name
		self.use_itl612 = 'ITL612' in self.camera_name
		self.use_sony_alpha = 'Sony' in self.camera_name
        self.first_frame = True

        camera_calibration = {}
        self.color_camera_info = {}
        
        if use_calibration:
            # calibration_directory = get_calibration_directory(self.camera_name, image_width, image_height)
            file_name_pattern = cam_calib_path
            file_names = glob.glob(file_name_pattern)
            file_names.sort()
            if len(file_names) > 0:
                file_name = file_names[-1]
                with open(file_name) as f:
                    camera_calibration = yaml.load(f, Loader=SafeLoader)
            else:
                print('Webcam: No camera calibration files with pattern ' + file_name_pattern + ' found.')

            assert camera_calibration, 'Webcam: Failed to successfully load camera calibration results.'
                
            print('Webcam: Loaded camera calibration results from file =', file_name)
            print('Webcam: Loaded camera calibration results =', camera_calibration)
            self.color_camera_info['camera_matrix'] = np.array(camera_calibration['camera_matrix'])
            self.color_camera_info['distortion_coefficients'] = np.array(camera_calibration['distortion_coefficients'])

        if self.use_logitech_c930:
            # Reset the Mi USB Webcam HD to avoid a bug that
            # results in ~5Hz frame rate and incorrect settings after
            # the first use of the camera.

            # The user needs to have permission to reset the USB
            # device. Make sure the user is a member of plugdev and
            # that that the appropriate udev rule for Logitech Webcam
            # C903e has been set up.

            # You can check that the user is a member of the plugdev group by running
            #
            # groups
            #
            # on the command line. If you need to add the user, you
            # can use the following command:
            #
            # adduser username plugdev

            vendor = 0x046d
            product = 0x0843
            if self.backe == True:
			    dev = usb.core.find(idVendor=vendor, idProduct=product, backend=self.backend)
				dev.set_configuration()
            else:			
                dev = usb.core.find(idVendor=vendor, idProduct=product)
            dev.reset()
            time.sleep(1.0)

        if self.use_sony_alpha:
            vendor = 0x054C                  
            product = 0x0D9F
            if self.backe == True:
			    dev = usb.core.find(idVendor=vendor, idProduct=product, backend=self.backend)
				dev.set_configuration()
            else:			
                dev = usb.core.find(idVendor=vendor, idProduct=product)
            dev.reset()
			
        if self.use_itl612:                         # e.g. wuHan technologies itl612 infa red
            vendor = 0x04B4                         # i dont know these yet !
            product = 0x8613		
            dev = usb.core.find(idVendor=vendor, idProduct=product, backend=self.backend)
		    # example on how to issue command : in this example it is set factory default  
            cb = 0x7                                # compute the checksum byte
            cb ^= 0x01
            cb ^= 0x00
            cb ^= 0x5
            dev.ctrl_transfer(0x55, 0xAA, 0x07, 0x01, 0x00, 0x05, 0x0, 0x0, 0x0, 0x0, cb)
			
        self.webcam = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        self.webcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if self.use_logitech_c930:

            # Maximum resolutions and framerates for TWO Logitech
            # C930e cameras plugged into the Stretch 3 trunk
            #
            # 640x480 at 15 fps
            # 1600x896 at 15 fps (startup may be less robust)
            # 1920x1080 at 10 fps

            image_size = (image_height, image_width)
            #image_size = (896, 1600) #(1080, 1920) #(480, 640) #(600, 800) #(480, 640)
            
            frames_per_second = fps
            #frames_per_second = 15 #30 #10

            fourcc_value = cv2.VideoWriter_fourcc(*'MJPG')
            self.webcam.set(cv2.CAP_PROP_FOURCC, fourcc_value)
            self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size[0])
            self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, image_size[1])            
            self.webcam.set(cv2.CAP_PROP_FPS, frames_per_second)

            ret, frame = self.webcam.read() 

            # auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
            # 1: Manual Mode
            # 3: Aperture Priority Mode
            # exposure_time_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=333 flags=inactive
            # exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
            # focus_absolute 0x009a090a (int)    : min=0 max=255 step=5 default=0 value=0 flags=inactive
            # focus_automatic_continuous 0x009a090c (bool)   : default=1 value=1

            exposure_time = 200 #120 #150 #200 #250
            webcam_command_line_configuration = 'v4l2-ctl -d ' + self.camera_device + ' -c auto_exposure=1,exposure_time_absolute=' + str(exposure_time) + ',focus_automatic_continuous=0'
            subprocess.check_call(webcam_command_line_configuration, shell=True)

    def get_next_frame(self): 

        # Wait for a coherent pair of frames: depth and color
        ret, color_image = self.webcam.read()
            
        if self.show_images:
            cv2.imshow('Webcam Image', color_image)
            cv2.waitKey(1)

        return color_image, self.color_camera_info


    def __del__(self):
        # Stop streaming
        print('Webcam.__del__: releasing the OpenCV camera.')
        self.webcam.release()
        
    
if __name__ == '__main__':
    print('cv2.__path__ =', cv2.__path__)
    parser = argparse.ArgumentParser(
        prog='Stretch Dexterous Teleop',
        description='Webcam module used by Stretch Dex Teleop.'
    )
    
    parser.add_argument('-s', '--second', action='store_true', help = 'If there are two Logitech C930e cameras available, use the second one found. The default is to use the first.')
    
    args = parser.parse_args()
    use_second_camera = args.second
            
    
    webcam = V4L2UsbWebcam(show_images=True, use_second_camera=use_second_camera)
    start_time = time.time()
    iterations = 0
    while True:
        iterations = iterations + 1
        current_time = time.time()
        total_duration = current_time - start_time
        average_period = total_duration / iterations
        average_frequency = 1.0/average_period
        image, camera_info = webcam.get_next_frame()
        print()
        print('--- Webcam Timing ---')
        print('number of frames =', iterations)
        print('average period =', "{:.2f}".format(average_period * 1000.0),  'ms')
        print('average frequency =', "{:.2f}".format(average_frequency),  'Hz')
        print('-----------------------------------------------')
