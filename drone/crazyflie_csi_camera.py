#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# MIT License
# modified from script for jetson Nano Copyright (c) 2019-2022 JetsonHacks
#
# A simple code snippet
# Using two  CSI cameras (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit with two CSI ports (Jetson Nano, Jetson Xavier NX) via OpenCV
# Drivers for the camera and OpenCV are included in the base image in JetPack 4.3+
#
# This script will open a window and place the camera stream from each camera in a window
# arranged horizontally.
# The camera streams are each read in their own thread, as when done sequentially there
# is a noticeable lag
#
# it shows how to use gstreamer pipeline with openCV to read CSI cameras and it is linked to the operation onboard of a crazyflie drone
#
import cv2
import threading
import numpy as np

from threading import Thread
import signal
import time
import sys

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
import time

try:
    import zmq
except ImportError as e:
    raise Exception("ZMQ library probably not installed ({})".format(e))

# think this is in radians if not change to 360 degrees
import math
CRAZY_MAX_RPY = math.pi*2.0
CRAZY_MIN_RPY = -math.pi*2.0

# common functions
def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def check_thrust(th, cmaxt, cmint ):
    th = constrain(th, -cmaxt, cmint)
    return th

def check_rpy(val):
    val = constrain(val, -CRAZY_MAX_RPY, CRAZY_MAX_RPY)
    return val

# connection to drone    
class _ConnThread(Thread):

    def __init__(self, socket, *args):
        super(_ConnThread, self).__init__(*args)
        self._socket = socket

    def run(self):
        while True:
            msg = self._socket.recv_json()
            print(msg)

# thread controlling the drone
import math

class _CtrlThread(Thread):

    def __init__(self, socket, *args):
        super(_CtrlThread, self).__init__(*args)
        self._socket = socket
        self._thrust_max = 30000
        self._thrust_min = 20000
        self._thrust = self._thrust_min
        self._thrust_step = 100
        self._roll = -math.pi*2.0
        self._pitch = -math.pi*2.0
        self._yaw = -math.pi*2.0
        self._rpy_step = 0.1
        self._cmd = {
            "version": 1,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "thrust": 0.0
        }

    def run(self):
        print("Starting to send control commands!")
        while True:
            key = getKey()
            if key == 'w' :
                self._thrust = check_thrust(self._thrust + self._thrust_step, self._thrust_max, self._thrust_min)
                self._cmd["thrust"] = self._thrust
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'x' :
                self._thrust = check_thrust(self._thrust - self._thrust_step, self._thrust_max, self._thrust_min)
                self._cmd["thrust"] = self._thrust
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'a' :
                self._roll = check_rpy(self._roll - self._rpy_step)
                self._cmd["roll"] = self._roll
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'd' :
                self._roll = check_rpy(self._roll - self._rpy_step)
                self._cmd["roll"] = self._roll
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'u' :
                self._pitch = check_rpy(self._pitch - self._rpy_step)
                self._cmd["pitch"] = self._pitch
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'n' :
                self._pitch = check_rpy(self._pitch - self._rpy_step)
                self._cmd["pitch"] = self._pitch
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'h' :
                self._yaw = check_rpy(self._yaw - self._rpy_step)
                self._cmd["yaw"] = self._yaw
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'k' :
                self._yaw = check_rpy(self._yaw - self._rpy_step)
                self._cmd["yaw"] = self._yaw
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == ' ' or key == 's' :
                self._thrust = self._thrust_min
                self._pitch = 0.0
                self._roll = 0.0 
                self._yaw = 0.0 
                self._cmd["yaw"] = self._yaw
                self._cmd["pitch"] = self._pitch  
                self._cmd["roll"] = self._roll
                self._cmd["thrust"] = self._thrust   
                self._socket.send_json(self._cmd)
                time.sleep(0.01)   
            elif key == 't' :
                self._yaw = 0.0  
                self._cmd["yaw"] = self._yaw   
                self._socket.send_json(self._cmd)
                time.sleep(0.01)  
            elif key == 'g' :
                self._pitch = 0.0  
                self._cmd["pitch"] = self._pitch   
                self._socket.send_json(self._cmd)
                time.sleep(0.01) 
            elif key == 'b' :
                self._roll = 0.0  
                self._cmd["roll"] = self._roll   
                self._socket.send_json(self._cmd)
                time.sleep(0.01)
            elif key == 'y':
                self._thrusr = 0.0  
                self._cmd["thrust"] = self._thrust   
                self._socket.send_json(self._cmd)
                time.sleep(0.01)      
            elif key == 27:
                break

# camera class gsml CSI camera            
class CSI_Camera:

    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def __del__(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()
            self.read_thread = None            
        self.running = False
        
    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(gstreamer_pipeline_string, cv2.CAP_GSTREAMER)
            
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
            self.running = True
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080
"""

# return the gstreamer pipeline
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# runs both cameras with the gstreamer pipelines as specified
CAMERA_ACT = 0
def run_cameras():
    global CAMERA_ACT
    CAMERA_ACT = 1
    window_title = "Dual CSI Cameras"
    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=0,
            capture_width=1920,
            capture_height=1080,
            flip_method=0,
            display_width=960,
            display_height=540,
        )
    )
    left_camera.start()

    right_camera = CSI_Camera()
    right_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=1920,
            capture_height=1080,
            flip_method=0,
            display_width=960,
            display_height=540,
        )
    )
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
                # Use numpy to place images next to each other
                camera_images = np.hstack((left_image, right_image)) 
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, camera_images)
                else:
                    break

                keyCode = cv2.waitKey(30) & 0xFF
                # Stop the program on the ESC key
                if keyCode == 27:
                    break
        finally:

            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open both cameras")
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()
    CAMERA_ACT = 0
    
if __name__ == "__main__":

    # signal interrupt handler
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    # drone address
    SRV_ADDR = "tcp://127.0.0.1"
    CF_URI = "radio://0/10/250K"

    context = zmq.Context()
    client_conn = context.socket(zmq.REQ)
    client_conn.connect("{}:2000".format(SRV_ADDR))
    
    param_conn = context.socket(zmq.SUB)
    param_conn.connect("{}:2002".format(SRV_ADDR))
    param_conn.setsockopt_string(zmq.SUBSCRIBE, u"")

    conn_conn = context.socket(zmq.SUB)
    conn_conn.connect("{}:2003".format(SRV_ADDR))
    conn_conn.setsockopt_string(zmq.SUBSCRIBE, u"")

    ctrl_conn = context.socket(zmq.PUSH)
    ctrl_conn.connect("{}:2004".format(SRV_ADDR))

    param_thread = _ParamThread(param_conn)
    param_thread.start()

    conn_thread = _ConnThread(conn_conn)
    conn_thread.start()

    print("Trying unknown command ...", end=' ')
    scan_cmd = {
        "version": 1,
        "cmd": "blah"
    }
    client_conn.send_json(scan_cmd)
    resp = client_conn.recv_json()
    if resp["status"] != 0:
        print("fail! {}".format(resp["msg"]))
    else:
        print("done!")

    print("Scanning for Crazyflies ...", end=' ')
    scan_cmd = {
        "version": 1,
        "cmd": "scan"
    }
    client_conn.send_json(scan_cmd)
    resp = client_conn.recv_json()
    print("done!")
    for i in resp["interfaces"]:
        print("\t{} - {}".format(i["uri"], i["info"]))

    connect_cmd = {
        "version": 1,
        "cmd": "connect",
        "uri": "{}".format(CF_URI)
    }
    print("Connecting to {} ...".format(connect_cmd["uri"]), end=' ')
    client_conn.send_json(connect_cmd)
    resp = client_conn.recv_json()
    if resp["status"] != 0:
        print("fail! {}".format(resp["msg"]))
        sys.exit(1)
    print("done!")    

    print("Parameter variables")
    for group in resp["param"]:
        print("\t{}".format(group))
        for name in resp["param"][group]:
            print("\t  {} ({}, {})= {}".format( name, resp["param"][group][name]["type"], resp["param"][group][name]["access"], resp["param"][group][name]["value"]))
            
    # how to set the drone parameters
    param_cmd = {
        "version": 1,
        "cmd": "param",
        "name": "system.selftestPassed",
        "value": True
    }

    print("Setting param {} to {}...".format(param_cmd["name"], param_cmd["value"]), end=' ')
    client_conn.send_json(param_cmd)
    resp = client_conn.recv_json()
    if resp["status"] == 0:
        print("param set is done!")
    else:
        print("failed with param set ! {}".format(resp["msg"]))    
    
    # Start sending control commands depending on the keyboard
    ctrl = _CtrlThread(ctrl_conn)
    ctrl.start()

    # run the camera thread
    run_cameras()

    # clean up drone connection if you get ESC key and your also closing down the cameras follwed by X
    time.sleep(2)
    while True:
        if CAMERA_ACT == 0 :                                    # we have closed the camera windows
            keyCode = cv2.waitKey(30) & 0xFF
            # Disconnect on the X key
            if keyCode == 'X':
                break       
    
    connect_cmd = {
        "version": 1,
        "cmd": "disconnect",
        "uri": "{}".format(CF_URI)
    }
    print("Disconnecting from {} ...".format(connect_cmd["uri"]), end=' ')
    client_conn.send_json(connect_cmd)
    resp = client_conn.recv_json()
    if resp["status"] != 0:
        print("fail! to disonnect from the drone {}".format(resp["msg"]))
        sys.exit(1)
    print("done!")
