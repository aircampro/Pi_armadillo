#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This module provides a class that controls the serial servo motor manufactured by Konishi Mokei Co.,Ltd  0
# JR PROPO XBUS Series XBUS Protocol v1.1.
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# 
#
import serial
import serial.tools.list_ports
import datetime

# XBUS write message format
#
# Command
# Length
# Key 0x00
# CH-ID
# Order
# Data1
# Data2
# Data3
# Data4
# CRC8

# user commands
KM_JR_Set_Command=0x20  
KM_JR_Get_Command=0x21  
KM_JR_Status_Command=0x22  

# order byte definitions
KM_JR_Order_Mode = 0x01   
KM_JR_OrderDat_Op_Mode=0x01  
KM_JR_OrderDat_ID_Set_Mode=0x02 

KM_JR_Order_ID=0x03   
KM_JR_Order_Version=0x04        # get only
KM_JR_Order_Product=0x05        # get only
KM_JR_Order_Para_Reset=0x07     #------set only
KM_JR_Order_Para_Write=0x08     # ------set only
KM_JR_Order_Reverse=0x10  
KM_JR_Order_Neutral=0x11  
KM_JR_Order_Trav_High=0x12  
KM_JR_Order_Trav_Low=0x13  
KM_JR_Order_Limit_High=0x14  
KM_JR_Order_Limit_Low=0x15  
KM_JR_Order_P_Gain=0x16  
KM_JR_Order_I_Gain=0x17 
KM_JR_Order_D_Gain=0x18 
KM_JR_Order_Dead_Band=0x19  
KM_JR_Order_Boost=0x1A  
KM_JR_Order_Alarm_Level1=0x1B 
KM_JR_Order_Alarm_Delay2=0x1C 
KM_JR_Order_Angle=0x1D 
KM_JR_OrderDat_Angle120= 0x00      # Normal operation (maximum angle 120 degrees) 
KM_JR_OrderDat_Angle180=0x01       # Maximum angle 180 degrees (Some models may not reach 180 # degrees)     
KM_JR_OrderDat_Angle150=0x02       # Maximum angle 150 degrees
KM_JR_Order_Slow_Start=0x1E  
KM_JR_Order_Stop_Mode=0x1F  
KM_JR_Order_Current_Position=0x20  #------get only 
KM_JR_Order_Current_Power=0x21     #------get only  
KM_JR_Order_Speed_Limit=0x22  
KM_JR_Order_Max_Integer=0x23  
KM_JR_Order_PWM_Mode=0x24 
KM_JR_OrderDat_PWM_0=0x00         # Normal Mode 
KM_JR_OrderDat_PWM_1520=0x01      # F mode 1 (Neutral 1520) 
KM_JR_OrderDat_PWM_760=0x02       # F Mode 2 (Neutral 760) 
KM_JR_OrderDat_PWM_300=0x03       # S mode (neutral 300)
KM_JR_Order_Interpolate=0x25  
KM_JR_Order_Current=0x26  

def search_com_port():
    coms = serial.tools.list_ports.comports()
    list_com = []
    for com in coms:
        list_com.append(com.device)
    print('Connected COM ports: ' + str(list_com))
    used_port = list_com[0]
    print('Using first listed COM port: ' + used_port)
    return used_port
	
class KonishiMokei_Xbus_Servo(object):
   
	def __init__(self):
		self.myserial = serial.Serial()
		print('Generated the serial object for the KonishiMokei_Xbus_Servo')
        self.f_out = []

	def __del__(self):
		self.myserial.close()
		
	def open_port(self, port=None, baudrate=250000, parity=serial.PARITY_NONE, timeout=1):
        if port == None:
            port = search_com_port()		
		self.myserial.port = port
		self.myserial.baudrate = baudrate
		self.myserial.timeout = timeout
		self.myserial.parity = parity
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

    # XBUS write message format (the default example sets the P Gain)
    #
    # Command
    # Length
    # Key 0x00
    # CH-ID
    # Order
    # Data1
    # Data2
    # Data3
    # Data4
    # CRC8
    #
	def send_msg_to_servo(self, cmd=KM_JR_Set_Command, len=8, chid=1, order=KM_JR_Order_P_Gain, dataAr ):

		send = [cmd,
				len,
				0x00,
				chid,
				order]
        for byt in dataAr:
            send.append(byt)        
		send.append(self._crc8(send))
		self._write_serial(send)
        if cmd == KM_JR_Get_Command:
            self.get_data()

	def get_data(self):
        int_data_1 = 0
        int_data_2 = 0
        int_data_3 = 0
        self.f_out = []
        f_run = 1
        to = datetime.timedelta(seconds=5)                           # set timeout on message
        start_t = datetime.datetime.now()
        
        while f_run == 1 and ((datetime.datetime.now() - start_t) < to):
            try:
                recv_data = self.myserial.read()
                int_data = int.from_bytes(recv_data, byteorder='big')
       
                if( (int_data_3==164)and(int_data_2==26)and(int_data_1==2)and(int_data==1) ):
                    self.f_out.append( int_data )            
                    f_run = 0 
                else:
                    self.f_out.append( int_data )
           
                int_data_3 = int_data_2
                int_data_2 = int_data_1
                int_data_1 = int_data

            except KeyboardInterrupt:
                print("Konishi Mokei Co.,Ltd JR PROPO XBUS - Stop by Ctrl+C")
                break

	# The following functions are provided for use in PRS class
	def _calc_checksum(self, send):
		checksum = send[2]
		for i in range(3, len(send)):
			checksum ^= send[i]
		return checksum

    # crc8 calculation as per c code in motor manual
    s_crc_array = [  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,   
                     0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,  
                     0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,  
                     0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,  
                     0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,  
                     0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,  
                     0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,  
                     0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,  
                     0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,  
                     0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,  
                     0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,  
                     0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,  
                     0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,  
                     0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,  
                     0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,  
                     0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,  
                     0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,  
                     0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,  
                     0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,  
                     0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,  
                     0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,  
                     0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,  
                     0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,  
                     0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,  
                     0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,  
                     0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,  
                     0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,  
                     0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,  
                     0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,  
                     0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,  
                     0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,  
                     0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, ]
 
	def _crc_table(self, dataV, crc):
        index = (dataV ^ crc) and 0xFF
		crc = self.s_crc_array[index]
		return crc

	def _crc8(self, send):
        crc = 0
		for i in range(0, len(send)):
			crc = self._crc_table(send[i], crc)
		return crc
        
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

# TEST :: set torque on drive and move angle/position then read back all data
#
def main():
    kmx_servo = KonishiMokei_Xbus_Servo()
    kmx_servo.open_port()
    # setting mode
    kmx_servo.send_msg_to_servo(order=KM_JR_Order_Mode,[0,0,0,KM_JR_OrderDat_Op_Mode])
    # set p band
    kmx_servo.send_msg_to_servo([0,0,0,2])
    # set angle
    kmx_servo.send_msg_to_servo(order=KM_JR_Order_Angle,[0,0,0,KM_JR_OrderDat_Angle180])	
    # get the drive position writes it into f_out list
    kmx_servo.send_msg_to_servo(cmd=KM_JR_Get_Command,order=KM_JR_Order_Current_Position,[0,0,0,0])   
    for v in kmx_servo.f_out:
        print(" message ",v)
    kmx_servo.close_port()	

LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        main()