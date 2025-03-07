#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This module provides a class that controls the serial interface to Hoverboard.
# 
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# 
# ref:- Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
#       From Emanuel FERU's hoverboard-firmware-hack-FOC firmware
#
import serial
import serial.tools.list_ports

#   uint16_t start;
#   int16_t  steer;
#   int16_t  speed;
#   uint16_t checksum;
#
class serial_send_cmd_t():

    def __init__(self,steer,speed):
        self.START_FRAME = 0xABCD
        self.start = self.START_FRAME
        self.steer = steer
        self.speed = speed
        self.checksum = self.calc_checksum()
        
    def calc_checksum(self):
        return self.start ^ self.steer ^ self.speed    

#   uint16_t start;
#   int16_t  cmd1;
#   int16_t  cmd2;
#   int16_t  speedR_meas;
#   int16_t  speedL_meas;
#   int16_t  wheelR_cnt;
#   int16_t  wheelL_cnt; 
#   int16_t  batVoltage;
#   int16_t  boardTemp;
#   uint16_t cmdLed;
#   uint16_t checksum;
#
class serial_rcv_cmd_t():

    def __init__(self, arr, m='little'):
        self.START_FRAME = 0xABCD
        self.start = self.endian(arr[0],arr[1],m);
        self.cmd1 = self.endian(arr[2],arr[3],m);
        self.cmd2 = self.endian(arr[4],arr[5],m);
        self.speedR_meas = self.endian(arr[6],arr[7],m);
        self.speedL_meas = self.endian(arr[8],arr[9],m);
        self.wheelR_cnt = self.endian(arr[10],arr[11],m);
        self.wheelL_cnt = self.endian(arr[12],arr[13],m); 
        self.batVoltage = self.endian(arr[14],arr[15],m);
        self. boardTemp = self.endian(arr[16],arr[17],m);
        self.cmdLed = self.endian(arr[18],arr[19],m);
        self.checksum_read = self.endian(arr[20],arr[21],m);
        
    def calc_checksum(self):
        self.checksum = self.start ^ self.cmd1 ^ self.cmd2 ^ self.speedR_meas ^ self.speedL_meas ^ self.wheelR_cnt ^ self.wheelL_cnt  ^ self.batVoltage ^ self.batVoltage 
        if self.checksum == self.checksum_read:
            return True
        else:
            return False
            
    def endian(self, a, b, mode='little'):
        if mode == 'little':
            return (a << 8) | b
        else:
            return (b << 8) | a        
            
class Hoverboard(object):

	def __init__(self):
		self.myserial = serial.Serial()
		print('Generated the serial object for the FutabaRS Servo')
		self.mode = 'normal'

	def __del__(self):
		self.myserial.close()

    def search_com_port(self):
        coms = serial.tools.list_ports.comports()
        list_com = []
        for com in coms:
            list_com.append(com.device)
        print('Connected COM ports: ' + str(list_com))
        used_port = list_com[0]
        print('Using first listed COM port: ' + used_port)
        return used_port

    # open the specified port or the first listed if not specified    
	def open_port(self, port=None, baudrate=115200, timeout=1):
        if port == None:
            port = self.search_com_port()		
		self.myserial.port = port
		self.myserial.baudrate = baudrate
		self.myserial.timeout = timeout
		self.myserial.parity = serial.PARITY_NONE
		try:
			self.myserial.open()
		except IOError:
			raise IOError('Failed to open port, check the device and port number')
		else:
			print('Succeede to open port: ' + port)

	def close_port(self):
		self.myserial.close()

	def set_port(self, baudrate=115200, timeout=0x01):
		self.myserial.baudrate = baudrate
		self.myserial.timeout = timeout
		self.myserial._reconfigurePort()
		print('Succeede to set baudrate:%d, timeout:%d' % (baudrate, timeout))

	def set_steer_speed(self, steer=100, speed=1000):
		self._check_range(steer, -65534, 65535, 'steer')
		self._check_range(speed, -65534, 65535, 'speed')
        ss = serial_send_cmd_t(steer,speed)
        
		send = [ss.start >> 8,
				ss.start & 0xFF,
				ss.steer >> 8,
				ss.steer & 0xFF,
				ss.speed >> 8,
				ss.speed & 0xFF,
				ss.checksum >> 8,
				ss.checksum & 0xFF]

		self._write_serial(send)

	def read_data(self, len=22, endi='little'):
		receive = self.myserial.read(len)                  # may also use if self.myserial.in_waiting >0: receive = self.myserial.read_all()      
		rec_nums = [ord(r) for r in receive]
		receive = [chr(r) for r in receive]
		print('received ', receive)

        return rec_nums, receive

    def parse_data(self, read_arr):
        if (read_arr[0] == 0xAB and read_arr[1] == 0xCD) or (read_arr[0] == 0xCD and read_arr[1] == 0xAB) :
            rcv = serial_rcv_cmd_t(read_arr)
            ok = rcv.calc_checksum()
            if ok == True:
                return rcv
        else:
            print("incorrect start bytes")
            return [-1]

	def _check_range(self, value, lower_range, upper_range, name='value'):
		if value < lower_range or value > upper_range:
			raise ValueError(name + ' must be set in the range from ' + str(lower_range) + ' to ' + str(upper_range))

	def _write_command(self, send):
		self.myserial.flushOutput()
		self.myserial.flushInput()
		self.myserial.write(bytearray(send))

	def _write_serial(self, send):
        self._write_command(send)

# TEST :: set steer and speed and read its data
#
def main():
    rpts = 0
    while rpts < 2:
        hover = Hoverboard()
        hover.open_port()
        hover.set_port()
        hover.set_steer_speed(10,2000)
	    time.sleep(1)
        hover.set_steer_speed(100,200)
        ord_data, char_data = hover.read_data()	
        print(char_data)
	    read_data = hover.parse_data(ord_data)
        print("speed R meas ",hover.speedR_meas, "speed L meas ",hover.speedL_meas, "wheelR ",hover.wheelR_cnt, "wheelL ",wheelL_cnt)
        print("batVoltage ",hover.speedR_meas.batVoltage, "board temp. ",hover.boardTemp)
        rpts += 1 
        if rpts == 2:
            hover.set_steer_speed(0,0)        
    hover.close_port()	

LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        main()