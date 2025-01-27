#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This module provides a class that controls the serial connection to P+F PGV100 vision angle/position 
# https://www.pepperl-fuchs.com/global/en/52998.htm
# untested
# It need pySerial(http://pyserial.sourceforge.net/)
# 
#
import serial
import serial.tools.list_ports
import datetime
import glob

def search_com_port():
    coms = serial.tools.list_ports.comports()
    list_com = []
    for com in coms:
        list_com.append(com.device)
    print('Connected COM ports: ' + str(list_com))
    used_port = list_com[0]
    print('Using first listed COM port: ' + used_port)
    return used_port

def get_first_usb_port():
    ttydev=glob.glob("/dev/tty*usb*")[0]
    return ttydev

def show_all_usb_ports():
    return glob.glob("/dev/tty*usb*")
	
class PF_PGV100(object):
   
	def __init__(self):
            self.myserial = serial.Serial()
            print('Generated the serial object for the KonishiMokei_Xbus_Servo')
            self.pos_y_mm = 0
            self.pos_x_mm = 0
            self.angle = 0
            self.f_run = 0
        
	def __del__(self):
            self.myserial.close()
		
	def open_port(self, port=None, baudrate=115200, parity=serial.PARITY_NONE, timeout=1):
            if port == None:
                port = search_com_port()		
            self.myserial.port = port
            self.myserial.baudrate = baudrate
            self.myserial.timeout = timeout
            self.myserial.parity = parity
            self.myserial.xonxoff = False                              # Enable software flow control.          False
            self.myserial.rtscts = False                               # Enable hardware (RTS/CTS) flow control. False 
            self.myserial.dsrdtr = False                               # Enable hardware (DSR/DTR) flow control. False
	    try:
	        self.myserial.open()
	    except IOError:
		raise IOError('Failed to open port, check the device and port number')
	    else:
		print('Succeeded to open communication to PGV100 on port: ' + port)

	def close_port(self):
		self.myserial.close()

	def set_port(self, baudrate=115200, timeout=0x01):
		self.myserial.baudrate = baudrate
		self.myserial.timeout = timeout
		self.myserial._reconfigurePort()
		print('Succeeded to set baudrate:%d, timeout:%d' % (baudrate, timeout))

	def select_no_lane(self):
		send = [ 0xE0, 0x1F ]
		self._write_serial(send)

	def select_follow_right(self):
		send = [ 0xE4, 0x1B ]
		self._write_serial(send)
        
	def select_follow_left(self):
		send = [ 0xE8, 0x17 ]
		self._write_serial(send)
        
	def select_straight_ahead(self):
		send = [ 0xEC, 0x13 ]
		self._write_serial(send)

	def request_position_info(self):
		send = [ 0xC8, 0x37 ]
		self._write_serial(send)
        
	def get_data(self):
            self.f_run = 1
            to = datetime.timedelta(seconds=2)                           # set timeout on message
            start_t = datetime.datetime.now()
        
            while self.f_run == 1 and ((datetime.datetime.now() - start_t) < to):
                try:
                    if self.myserial.in_waiting > 0: 
                        recv_data = self.myserial.read_all()
                        recv_data = recv_data.decode('utf-8')
                        # Get Lane-Detection from the byte array [Bytes 1-2]
                        if len(recv_data) > 12:
                            agv_lane_detect_str = str(recv_data[0]) + str(recv_data[1]))
                            agv_lane_detect_int = int(agv_lane_detect_str)
                            mask = 0b0000000011000000
                            agv_c_lane_count_des = agv_lane_detect_int and mask
                            mask = 0b0000000000010000
                            agv_no_color_lane_des = agv_lane_detect_int and mask
                            mask = 0b0000010000000000
                            agv_no_pos_des = agv_lane_detect_int and mask
                            mask = 0b0000000100000000
                            tag_detected_des = agv_lane_detect_int and mask 
    
                            # get angle
                            agv_ang_str = str(recv_data[10]) + str(recv_data[11])                    
                            agv_ang_des = int(agv_ang_str) / (10.0)
                            if (agv_ang_des > 180.0) :                             # this makes x-pos zero centered
                                agv_ang_des -= 360.0
                            self.angle = agv_ang_des
                        
                            # Get the X-Position from the byte array [Bytes 3-4-5-6]
                            mask = 0x07
                            x_pos_3 = (int(recv_data[2]) and mask)
                            mask = 0x7F
                            x_pos_2 = (int(recv_data[3]) and mask)
                            x_pos_1 = (int(recv_data[4]) and mask)
                            x_pos_0 = (int(recv_data[5]) and mask)
                            agv_x_pos_str = str(x_pos_3) + str(x_pos_2) + str(x_pos_1) + str(x_pos_0)
                            agv_x_pos_des = int(agv_x_pos_str)
                            if (tag_detected_des != 0):
                                if (agv_x_pos_des > 2000.0) :                                              # this makes x-pos zero centered
                                    agv_x_pos_des = agv_x_pos_des - pow(2,24) - 1
                            self.pos_x_mm = agv_x_pos_des / 10.0
                        
                            # Get Y-Position from the byte array [Bytes 7-8]
                            y_pos_1 = (int(recv_data[6]) and mask)
                            y_pos_0 = (int(recv_data[7]) and mask)
                            agv_y_pos_str = str(y_pos_1) + str(y_pos_0)
                            agv_y_pos_des = int(agv_y_pos_str)
                            if (agv_y_pos_des > 2000.0): # this makes y-pos zero centered
                                agv_y_pos_des = agv_y_pos_des - 16383.0
                            # We get opposite values when we try the read the y-pos value from the colored and code strip.
                            # So this is checking which strip that we're reading.
                            if (agv_no_pos_des):
                                agv_y_pos_des *= -1
                            self.pos_y_mm = agv_y_pos_des / 10.0
                            self.f_run = 0
                        
            except KeyboardInterrupt:
                print("P+F PGV100 - Stop by Ctrl+C")
                break
        
	def _check_range(self, value, lower_range, upper_range, name='value'):
		if value < lower_range or value > upper_range:
			raise ValueError(name + ' must be set in the range from '
							 + str(lower_range) + ' to ' + str(upper_range))

	def _write_command(self, send):
		self.myserial.flushOutput()
		self.myserial.flushInput()
		self.myserial.write(bytearray(send))

	def _write_serial(self, send):
        self._write_command(send)

    def int_to_hexstring(self, i):
        return f"0x{i:02x}"

    def hexstring_to_int(self, h):
        return int(h, 16)
        
# TEST :: set torque on drive and move angle/position then read back all data
#
def main():
    pgv100 = PF_PGV100()
    pgv100.open_port()
    pgv100.select_straight_ahead()
    pgv100.request_position_info()
    pgv100.get_data()
    print("PF PGV100 angle",pgv100.angle, " x ",pgv100.pos_x_mm," y ",pgv100.pos_y_mm)
    pgv100.close_port()	

LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        main()
