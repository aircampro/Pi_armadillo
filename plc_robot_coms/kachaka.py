#!/usr/bin/env python3
#
# Example of kachaka bot moving and showing an undistorted image
# ref:- https://github.com/atinfinity/kachaka-api/blob/main/python
#
from __future__ import annotations
import argparse
from kachaka_api.aio import KachakaApiClient 
import numpy as np
import cv2 
import threading
import ctypes
import time

g_RUN = True

# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="import map data from file")
    parser.add_argument("--ip_address",  type=str,  help="IP Address of Kachaka",)
    parser.add_argument("--port",  type=str,  help="port of Kachaka",)
    return parser.parse_args()

def main() -> None:
    global g_RUN
    args = parse_args()
    port_def = 26400
    if not args.port == None:
        port_def = args.port
    client = (                                                         # connect to kachaka bot
        KachakaApiClient()
        if args.ip_address is None
        else KachakaApiClient(f"{args.ip_address}:{port_def}")
    )
    def get_img_undistorted():
        image = client.get_front_camera_ros_compressed_image()
        camera_info = client.get_front_camera_ros_camera_info()
        cv_image = cv2.imdecode(np.frombuffer(image.data, dtype=np.uint8), flags=1)
        mtx = np.array(camera_info.K, dtype=float).reshape(3, 3)
        dist = np.array(camera_info.D)
        height = camera_info.height
        width = camera_info.width
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 0, (width, height))
        map_x, map_y = cv2.initUndistortRectifyMap(mtx, dist, None, new_camera_mtx, (width, height), 5)
        undistorted_image = cv2.remap(cv_image, map_x, map_y, cv2.INTER_LINEAR)
        return undistorted_image
    def say(msg="hi this is kachaka"):
        client.speak(msg)
    # moves kachaka bot
    def move(linear, angular):
        client.set_robot_velocity(linear=linear, angular=angular)
    # defines the movement grid for the robot
    #
    def move_bot(steps_forward = 5, step_lin_vel = 0.5, left = 0.5, right = -0.5, wait_tm=2):
        say("starting movement cycle")
        for i in range(0,steps_forward):    # move forward
            move(step_lin_vel, 0)
            time.sleep(wait_tm)
        move(0, left)                       # turn left
        for i in range(0,steps_forward):    # move forward
            move(step_lin_vel, 0)
            time.sleep(wait_tm)
        move(0, left)                       # turn left
        for i in range(0,steps_forward):    # move forward
            move(step_lin_vel, 0)
            time.sleep(wait_tm)
        move(0, right)                       # turn right
        for i in range(0,steps_forward):     # move forward
            move(step_lin_vel, 0)
            time.sleep(wait_tm)
        for i in range(0,steps_forward*2):   # move backward
            move(-step_lin_vel, 0)
            time.sleep(wait_tm)   
        say("complated movement cycle")
    # continuosly move round the grid
    def continuos_move(wait_tm=2):
        global g_RUN
        while g_RUN == True:
            move_bot()
            time.sleep(wait_tm)

    mv_thread = twe(name = 'Thread Move', target=continuos_move, args=(), kwargs={})
    mv_thread.start()
    while g_RUN == True:
        try:
            im = get_img_undistorted()
            cv2.imshow("undistorted image", im)                    # you could aldo stream this
        except KeyboardInterrupt:
            g_RUN = False       
    # join thread
    time.sleep(6)
    if mv_thread.is_alive(): mv_thread.raise_exception()
    mv_thread.join()
if __name__ == "__main__":
    main()