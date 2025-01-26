#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This module provides a class that reads a Futada Remote Control over SBUS Serial protocol
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# 
#
import serial
import serial.tools.list_ports

def search_com_port():
    coms = serial.tools.list_ports.comports()
    list_com = []
    for com in coms:
        list_com.append(com.device)
    print('Connected COM ports: ' + str(list_com))
    used_port = list_com[0]
    print('Using first listed COM port: ' + used_port)
    return used_port

GET_DATA = 0	
class FutabaRC_SBUS(object):

	def __init__(self):
		self.myserial = serial.Serial()
		print('Generated the serial object for the FutabaRS Servo')
		self.mode = 'normal'
        self.sbus_chan = []
            
	def __del__(self):
		self.myserial.close()
		
	def open_port(self, port=None, baudrate=115200, timeout=1):
        if port == None:
            port = search_com_port()		
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

	def get_data(self):

        # get the sbus response from the controller 
        #
        char_rx = 0
        
        while GET_DATA == 0:
		    receive = self.myserial.read(1)                                                  # read char by char for best syncing
		    receive = [chr(r) for r in receive]
		    print('receive', receive)
            if ord(receive) == 0x0F and char_rx == 0:                                        # sbus start colecting the bytearray
                char_rx += 1
                Chdata = []
                sbus_data = []
                sbus_data.append(ord(receive))
            elif char_rx > 0:
                char_rx += 1
                sbus_data.append(ord(receive))
                
            if char_rx == 3:
                Chdata.append(sbus_data[1]|(sbus_data[2]<<8)&0x07ff)
            elif char_rx == 4:
                Chdata.append(sbus_data[3]<<5|sbus_data[2]>>3)&0x07ff
            elif char_rx == 6:
                Chdata.append(sbus_data[3]>>6|sbus_data[4]<<2|sbus_data[5]<<10)&0x07ff
            elif char_rx == 7:
                Chdata.append(sbus_data[6]<<7|sbus_data[5]>>1)&0x07ff
            elif char_rx == 8:
                Chdata.append(sbus_data[7]<<4|sbus_data[6]>>4)&0x07ff
            elif char_rx == 10:
                Chdata.append(sbus_data[7]>>7|sbus_data[8]<<1|sbus_data[9]<<9)&0x07ff
            elif char_rx == 11:
                Chdata.append((sbus_data[9]>>2|sbus_data[10]<<6) & 0x07FF)
            elif char_rx == 12:
                Chdata.append((sbus_data[10]>>5|sbus_data[11]<<3) & 0x07FF)
            elif char_rx == 14:
                Chdata.append((sbus_data[12]|sbus_data[13]<< 8) & 0x07FF)
            elif char_rx == 15:
                Chdata.append((sbus_data[13]>>3|sbus_data[14]<<5) & 0x07FF)
            elif char_rx == 16:
                Chdata.append((sbus_data[14]>>6|sbus_data[15]<<2|sbus_data[16]<<10) & 0x07FF)
            elif char_rx == 17:
                Chdata.append((sbus_data[16]>>1|sbus_data[17]<<7) & 0x07FF)
            elif char_rx == 19:
                Chdata.append((sbus_data[17]>>4|sbus_data[18]<<4) & 0x07FF)
            elif char_rx == 21:
                Chdata.append((sbus_data[18]>>7|sbus_data[19]<<1|sbus_data[20]<<9) & 0x07FF)
            elif char_rx == 22:
                Chdata.append((sbus_data[20]>>2|sbus_data[21]<<6) & 0x07FF)
            elif char_rx == 23:
                Chdata.append((sbus_data[21]>>5|sbus_data[22]<<3) & 0x07FF)
            elif char_rx == 24:
                Chdata.append(sbus_data[23])
            elif char_rx == 25:
                Chdata.append(sbus_data[24])
                char_rx=0
                for e,f in enumerate(Chdata):
                    print("channel =",e," value="f)
                self.sbus_chan = ChData

	# The following functions are provided for use in PRS class
	def _calc_checksum(self, send):
		checksum = send[2]
		for i in range(3, len(send)):
			checksum ^= send[i]
		return checksum

	def _check_range(self, value, lower_range, upper_range, name='value'):
		if value < lower_range or value > upper_range:
			raise ValueError(name + ' must be set in the range from '
							 + str(lower_range) + ' to ' + str(upper_range))

	def _write_command(self, send):
		self.myserial.flushOutput()
		self.myserial.flushInput()
		self.myserial.write(bytearray(send))

# TEST :: set torque on drive and move angle/position then read back all data
#
def main():
    futa_rc_sbus = FutabaRC_SBUS()
    futa_rc_sbus.open_port()
    try:
        futa_rc_sbus.get_data()                              # in use you might want to thread this operation 
    except KeyboardInterrupt:  
        GET_DATA = 1    
        futa_rc_sbus.close_port()	

LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        main()