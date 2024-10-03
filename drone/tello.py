#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# tello.py
#
import socket           # uses UDP socket to communicate with drone
import threading       
import time             
import numpy as np      
import libh264decoder   # H.264 decoder needs libh264decoder.so)

class Tello:
    """Tello Drone Control Classス"""

    def __init__(self, local_ip, local_port, imperial=False, command_timeout=.3, tello_ip='192.168.10.1', tello_port=8889):

        """
        Initialization of the class. Bind local IP/port and put Tello in command mode.

        :param local_ip (str): Local IP address to bind (make UDP server)
        :param local_port (int): Local port number to bind to
        :param imperial (bool): If True, speed is in miles per hour and distance is in feet.
                                If False, speed is in km/h and distance is in meters. Default is False
        :param command_timeout (int|float): Time to wait for command response. Default is 0.3 seconds.
        :param tello_ip (str): Tello's IP address. 192.168.10.1 if not EDU
        :param tello_port (int): Tello port.Usually 8889
        """

        self.abort_flag = False     
        self.decoder = libh264decoder.H264Decoder() 
        self.command_timeout = command_timeout      
        self.imperial = imperial    
        self.response = None    
        self.frame = None       
        self.is_freeze = False  
        self.last_frame = None  
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        
        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
        self.tello_address = (tello_ip, tello_port)     
        self.local_video_port = 11111                   
        self.last_height = 0                            
        self.socket.bind((local_ip, local_port))        

        # define and start recv thread
        self.receive_thread = threading.Thread(target=self._receive_thread)     
        self.receive_thread.daemon = True   

        self.receive_thread.start()         

        self.socket.sendto(b'command', self.tello_address)          # 'command' as per SDK
        print ('sent: command')
        self.socket.sendto(b'streamon', self.tello_address)         # 'streamon'
        print ('sent: streamon')

        self.socket_video.bind((local_ip, self.local_video_port))   

        # define and start  video thread
        self.receive_video_thread = threading.Thread(target=self._receive_video_thread)     
        self.receive_video_thread.daemon = True 

        self.receive_video_thread.start()       

    def __del__(self):
        """clean-up"""

        self.socket.close()         
        self.socket_video.close()   

    def read(self):
        """read frame"""
        if self.is_freeze:          
            return self.last_frame
        else:                       
            return self.frame

    def video_freeze(self, is_freeze=True):
        """freeze video"""
        self.is_freeze = is_freeze  
        if is_freeze:               
            self.last_frame = self.frame

    def _receive_thread(self):
        """
        rcv data response to command

        """
        while True:
            try:
                self.response, ip = self.socket.recvfrom(3000)     
                #print(self.response)
            except socket.error as exc:     
                print ("Caught exception socket.error : %s" % exc)

    def _receive_video_thread(self):
        """
        thread for video grabber

        """
        packet_data = ""    
        while True:
            try:
                res_string, ip = self.socket_video.recvfrom(2048)   
                packet_data += res_string       

                if len(res_string) != 1460:     ．
                    for frame in self._h264_decode(packet_data):    
                        self.frame = frame
                    packet_data = ""    

            except socket.error as exc:
                print ("Caught exception socket.error : %s" % exc)

    def _h264_decode(self, packet_data):
        """
        H264 decode

        :param packet_data: H.264 raw data

        :return: decoded frame
        """
        res_frame_list = []     
        frames = self.decoder.decode(packet_data)   
        for framedata in frames:    
            (frame, w, h, ls) = framedata   
            if frame is not None:   
                # print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)

                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')     
                frame = (frame.reshape((h, ls / 3, 3)))     
                frame = frame[:, :w, :]                     
                res_frame_list.append(frame)                

        return res_frame_list   

    def send_command(self, command):
        """
        Tello Communication Function

        :param command: what you want to say to the drone
        :return (str): Tello reply

        """

        print (">> send cmd: {}".format(command))
        self.abort_flag = False     
        timer = threading.Timer(self.command_timeout, self.set_abort_flag)      

        self.socket.sendto(command.encode('utf-8'), self.tello_address)    

        timer.start()   
        while self.response is None:        
            if self.abort_flag is True:    
                break
        timer.cancel()  

        if self.response is None:       
            response = 'none_response'
        else:                           
            response = self.response.decode('utf-8')

        self.response = None    

        return response     

    def set_abort_flag(self):
        """

        """

        self.abort_flag = True

    def takeoff(self):
        """
        drone take-off

        Returns:
            str: 'OK 'FALSE'.

        """

        return self.send_command('takeoff')

    def get_tof(self):
        """

        Returns:
            str: Get distance from TOF in cm	30-1000.

        """

        return self.send_command('tof')

    def get_flight_time(self):
        """

        Returns:
            str: Get flight time

        """

        return self.send_command('time')
        
    def get_attitude(self):
        """

        Returns:
            str: Get imu data

        """

        return self.send_command('attitude')

    def get_acceleration(self):
        """

        Returns:
            str: Get imu angular acceleration

        """

        return self.send_command('acceleration')
        
    def get_serial_number(self):
        """

        Returns:
            str: Get serial number

        """

        return self.send_command('sn')

    def get_barometer(self):
        """

        Returns:
            str: Get barometer

        """

        return self.send_command('baro')
        
    def sdk(self):
        """

        Returns:
            str: sdk verison

        """

        return self.send_command('sdk')
        
    def esd(self):
        """

        Returns:
            str: Emergency stop (stop the motor immediately)

        """

        return self.send_command('emergency')

    def wifi(self):
        """

        Returns:
            str: return signal strength

        """

        return self.send_command('wifi')

    def rc(self,a,b,c,d):
        """

        rc a b c d	Set remoto controller Control via four channels
　　　　   ”a” = {roll} left/right (-100,100)
　　　　   ”b” = {pitch} forward/backward (-100,100)
　　　　   ”c” = {throttle} up/down (-100,100)
　　　　   ”d” = {yaw}  (-100,100)

        """

        return self.send_command('rc %s %s %s %s' % (a,b,c,d))
        
    def get_temperature(self):
        """

        Returns:
            str: return temperature

        """

        return self.send_command('temp')
        
    def hover(self):
        """

        Returns:
            str: hover

        """

        return self.send_command('stop')

    def set_imperial(self):
        """
        changes units to imperial

        """
        self.imperial = True

    def set_metric(self):
        """
        changes units to metric

        """
        self.imperial = False
        
    def set_speed(self, speed):
        """
        set speed

        Metric: .1 to 3.6 km/h
        Imperial: .1 to 2.2 Mile/h

        Args:
            speed (int|float): 

        Returns:
            str: Tello 'OK' 'FALSE'.

        """

        speed = float(speed)

        if self.imperial is True:       
            speed = int(round(speed * 44.704))      # Mile/h -> cm/s
        else:
            speed = int(round(speed * 27.7778))     # km/h -> cm/s

        return self.send_command('speed %s' % speed)

    def rotate_cw(self, degrees):
        """
        rotate cloackwise

        Args:
            degrees (int): 1〜360

        Returns:
            str: Tello．'OK' 'FALSE'.

        """

        return self.send_command('cw %s' % degrees)

    def rotate_ccw(self, degrees):
        """
        rotate counter cloackwise

        Args:
            degrees (int):， 1〜360度.

        Returns:
            str: Tello'OK' 'FALSE'.

        """
        return self.send_command('ccw %s' % degrees)

    def flip(self, direction='f'):
        """
        flip drone

        Args:
            direction (str): ， 'l', 'r', 'f', 'b'.

        Returns:
            str: Tello．'OK' 'FALSE'.
        """

        return self.send_command('flip %s' % direction)

    def get_response(self):
        """
        Tello response

        Returns:
            int: Tello code

        """
        response = self.response
        return response

    def get_height(self):
        """
        height cm

        Returns:
            int: (cm)

        """
        height = self.send_command('height?')
        height = str(height)
        height = filter(str.isdigit, height)
        try:
            height = int(height)
            self.last_height = height
        except:
            height = self.last_height
            pass
        return height

    def get_battery(self):
        """
        battery 

        Returns:
            int: battery usage

        """

        battery = self.send_command('battery?')

        try:
            battery = int(battery)
        except:
            pass

        return battery

    def get_flight_time(self):
        """
        flight time

        Returns:
            int: Get current flight time [seconds]

        """

        flight_time = self.send_command('time?')

        try:
            flight_time = int(flight_time)
        except:
            pass

        return flight_time

    def get_speed(self):
        """
        get speed

        Returns:
            int:  km/h or Mile/h

        """

        speed = self.send_command('speed?')

        try:
            speed = float(speed)

            if self.imperial is True:
                speed = round((speed / 44.704), 1)      # cm/s -> mile/h
            else:
                speed = round((speed / 27.7778), 1)     # cm/s -> km/h
        except:
            pass

        return speed

    def land(self):
        """
        land

        Returns:
            str: Tello 'OK' 'FALSE'.

        """

        return self.send_command('land')

    def move(self, direction, distance):
        """

        Args:
            direction (str): 'forward', 'back', 'right' or 'left'．
            distance (int|float): 

        Returns:
            str: Tello 'OK' 'FALSE'.

        """

        distance = float(distance)

        if self.imperial is True:
            distance = int(round(distance * 30.48))     # feet -> cm
        else:
            distance = int(round(distance * 100))       # m -> cm

        return self.send_command('%s %s' % (direction, distance))

    def move_backward(self, distance):
        """

        Args:
            distance (int): 

        Returns:
            str: Tello 'OK' 'FALSE'.

        """

        return self.move('back', distance)

    def move_down(self, distance):
        """．

        Args:
            distance (int): 

        Returns:
            str: Tello 'OK' 'FALSE'.

        """

        return self.move('down', distance)

    def move_forward(self, distance):
        """
        Args:
            distance (int): 

        Returns:
            str: Tello 'OK' 'FALSE'..

        """
        return self.move('forward', distance)

    def move_left(self, distance):
        """
        Args:
            distance (int): 

        Returns:
            str: Tello 'OK' 'FALSE'.

        """
        return self.move('left', distance)

    def move_right(self, distance):
        """
        Args:
            distance (int): 

        Returns:
            str: Tello 'OK' 'FALSE'..

        """
        return self.move('right', distance)

    def move_up(self, distance):
        """
        Args:
            distance (int): 

        Returns:
            str: Tello 'OK' 'FALSE'.

        """

        return self.move('up', distance)