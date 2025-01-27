#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This module provides a class that controls the serial servo motor manufactured by Futaba Corp.
# RS30x and RS40x
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# 
# ref:- https://github.com/HiroakiMatsuda/pyrs (C) 2012 Matsuda Hiroaki
# modifidied by ACP to include angle control command
# drives supported are :- FUTABA RS40x and RS30x
#
import serial
import serial.tools.list_ports

class FutabaRS_Servo(object):

    # Address for various control actions in drives RAM memory
    #
    ADDR_MODEL_NUMBER_L = 0  # 0x00
    ADDR_FIRMWARE_VERSION = 2  # 0x02
    ADDR_SERVO_ID = 4  # 0x04
    ADDR_REVERSE = 5  # 0x05
    ADDR_BAUD_RATE = 6  # 0x06
    ADDR_RETURN_DELAY = 7  # 0x07
    ADDR_CW_ANGLE_LIMIT_L = 8  # 0x08
    ADDR_CCW_ANGLE_LIMIT_L = 10  # 0x0A
    ADDR_TEMPERATURE_LIMIT_L = 14  # 0x0E
    ADDR_TORQUE_IN_SILENCE = 22  # 0x16
    ADDR_WARM_UP_TIME = 23  # 0x17
    ADDR_CW_COMPLIANCE_MARGIN = 24  # 0x18
    ADDR_CCW_COMPLIANCE_MARGIN = 25  # 0x19
    ADDR_CW_COMPLIANCE_SLOPE = 26  # 0x1A
    ADDR_CCW_COMPLIANCE_SLOPE = 27  # 0x1B
    ADDR_PUNCH_L = 28  # 0x1C

    ADDR_GOAL_POSITION_L = 30  # 0x1E
    ADDR_GOAL_TIME_L = 32  # 0x20
    ADDR_MAX_TORQUE = 35  # 0x23
    ADDR_TORQUE_ENABLE = 36  # 0x24
    ADDR_PID_COEFFICIENT = 38  # 0x26
    ADDR_PRESENT_POSITION_L = 42  # 0x2A
    ADDR_PRESENT_TIME_L = 44  # 0x2C
    ADDR_PRESENT_SPEED_L = 46  # 0x2E
    ADDR_PRESENT_CURRENT_L = 48  # 0x30
    ADDR_TEMPERATURE_L = 50  # 0x32
    ADDR_VOLTAGE_L = 52  # 0x34

    ADDR_WRITE_FLASH_ROM = 255  # 0xFF
    ADDR_RESET_MEMORY = 255  # 0xFF

    # フラグ data for the flags 
    FLAG4_RESET_MEMORY_MAP = 16  # 0x10
    FLAG30_NO_RETURN_PACKET = 0
    FLAG30_MEM_MAP_ACK = 1
    FLAG30_MEM_MAP_00_29 = 3
    FLAG30_MEM_MAP_30_59 = 5
    FLAG30_MEM_MAP_20_29 = 7
    FLAG30_MEM_MAP_42_59 = 9
    FLAG30_MEM_MAP_30_41 = 11
    FLAG30_MEM_MAP_60_127 = 13
    FLAG30_MEM_MAP_SELECT = 15

    PACKET_DATA_INDEX = 7

    # indexs when changing baud rate
    BAUD_RATE_INDEX_9600 = 0x00
    BAUD_RATE_INDEX_14400 = 0x01
    BAUD_RATE_INDEX_19200 = 0x02
    BAUD_RATE_INDEX_28800 = 0x03
    BAUD_RATE_INDEX_38400 = 0x04
    BAUD_RATE_INDEX_57600 = 0x05
    BAUD_RATE_INDEX_76800 = 0x06
    BAUD_RATE_INDEX_115200 = 0x07
    BAUD_RATE_INDEX_153600 = 0x08
    BAUD_RATE_INDEX_230400 = 0x09
    
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

	def torque_on(self, id=0x01, mode=0x01, return_packet=0x01):
		self._check_range(id, 1, 127, 'id')
		self._check_range(mode, 0, 2, 'mode')
		self._check_range(return_packet, 0, 15, 'return_packet')

		send = [0xFA,
				0xAF,
				id,
				return_packet,
				0x24,
				0x01,
				0x01,
				mode & 0x00FF]
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)

    # Moving the servo to the target position (RAM=1EH)
	#
	def target_position(self, id-0x01, position, time, return_packet=0x01):
		self._check_range(id, 1, 127, 'id')
		self._check_range(position, -1500, 1500, 'position')
		self._check_range(time, 0, 16383, 'time')
		self._check_range(return_packet, 0, 15, 'return_packet')

		send = [0xFA,
				0xAF,
				id,
				return_packet,
				0x1E,
				0x04,
				0x01,
				position & 0x00FF,
				(position & 0xFF00) >> 8,
				time & 0x00FF,
				(time & 0xFF00) >> 8]
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)


    def set_target_time(self, speed_second, id=1, return_packet=0x01):
        """
        :param speed_second:
        :param id:
        :return:
        """
		self._check_range(id, 1, 127, 'id')
		self._check_range(speed_second, 0, 163.83, 'speed_second')
		self._check_range(return_packet, 0, 15, 'return_packet')
        speed_hex = format(int(speed_second * 100) & 0xffff, '04x')
        speed_hex_h = int(speed_hex[0:2], 16)
        speed_hex_l = int(speed_hex[2:4], 16)
        
		send = [0xFA,
				0xAF,
				id,
				return_packet,
				self.ADDR_GOAL_TIME_L,
				0x02,
				0x01,
				speed_hex_h,
				speed_hex_l]
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)       
        
    # Moving the servo to the target angle (RAM=1EH) Unit
    # =0.1 degree,
    # e.g. 123 degree→ 1230→04CEH
    # FA AF 01 00 1E 02 01 CE 04 D6
	#
	def target_angle(self, id=0x01, angle, return_packet=0x01):
		self._check_range(id, 1, 127, 'id')
		self._check_range(angle, -144, 144, 'angle')
		self._check_range(return_packet, 0, 15, 'return_packet')

		send = [0xFA,
				0xAF,
				id,
				return_packet,
				0x1E,
				0x02,
				0x01,
				angle & 0x00FF,
				(angle & 0xFF00) >> 8]
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)
        
	def send_command_byte(self, id=0x01, val, ram_cmd_reg=self.ADDR_PID_COEFFICIENT, return_packet=0x01, cnt=0x01):
		self._check_range(id, 1, 127, 'id')

        if val == None:
		    send = [0xFA,
				    0xAF,
				    id,
				    return_packet,
				    ram_cmd_reg,
				    0x00,
				    cnt ]
        else:      
		    send = [0xFA,
				    0xAF,
				    id,
				    return_packet,
				    ram_cmd_reg,
				    0x02,
				    cnt,
				    val & 0x00FF,
				    (val & 0xFF00) >> 8]
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)

	def send_command_byte_array(self, id=0x01, valArr, ram_cmd_reg=self.ADDR_WRITE_FLASH_ROM, return_packet=0x01):
		self._check_range(id, 1, 127, 'id')

		send = [0xFA,
				0xAF,
				id,
				return_packet,
				ram_cmd_reg,
				len(valArr),
				0x01]
        for b in valArr:
            send.append(b)
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)
            
    def save_rom(self,sid):
        return self.send_command_byte(id=sid, val=None, ram_cmd_reg=self.ADDR_WRITE_FLASH_ROM, return_packet=0x40, cnt=0x00 )

    def set_max_torque(self, sid, torq_spt):
        torq_spt = int(torq_spt)
        return self.send_command_byte(id=sid, val=torq_spt, ram_cmd_reg=self.ADDR_MAX_TORQUE, return_packet=0x00, cnt=0x01 )

    def set_limit_cw_pos(self, sid, limit_position):
        limit_position_hex = format(int(limit_position * 10) & 0xffff, '04x')
        limit_position_hex_h = int(limit_position_hex[0:2], 16)
        limit_position_hex_l = int(limit_position_hex[2:4], 16)
        return self.send_command_byte_array(id=sid, valArr=[limit_position_hex_l, limit_position_hex_h], ram_cmd_reg=self.ADDR_CW_ANGLE_LIMIT_L, return_packet=0x00, cnt=0x01 )

    def set_limit_ccw_pos(self, sid, limit_position):
        limit_position_hex = format(int(limit_position * 10) & 0xffff, '04x')
        limit_position_hex_h = int(limit_position_hex[0:2], 16)
        limit_position_hex_l = int(limit_position_hex[2:4], 16)
        return self.send_command_byte_array(id=sid, valArr=[limit_position_hex_l, limit_position_hex_h], ram_cmd_reg=self.ADDR_CCW_ANGLE_LIMIT_L, return_packet=0x00, cnt=0x01 )
        
	def send_command_msg(self, id=0x01, valArr, ram_cmd_reg=self.ADDR_WRITE_FLASH_ROM, return_packet=0x01):
		self._check_range(id, 1, 127, 'id')

		send = [0xFA,
				0xAF,
				id,
				return_packet,
				ram_cmd_reg,
				len(valArr)*2,
				0x01]
        for b in valArr:
            send.append(b & 0x00FF)
            send.append((b & 0xFF00) >> 8)
		send.append(self._calc_checksum(send))

		self._write_serial(send, return_packet)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)
			
	def multi_torque_on(self, servo_data):
		for servo in servo_data:
			self._check_range(servo[0], 1, 127, 'id')
			self._check_range(servo[1], 0, 2, 'mode')

		send = [0xFA, 0xAF, 0x00, 0x00, 0x24, 0x02, len(servo_data)]
		for servo in servo_data:
			send.append(servo[0])
			send.append(servo[1])
		send.append(self._calc_checksum(send))

		self._write_serial(send, 0)

		return 'multi_torque_on:' + str(servo_data)

	def multi_target_position(self, servo_data):
		for servo in servo_data:
			self._check_range(servo[0], 1, 127, 'id')
			self._check_range(servo[1], -1500, 1500, 'position')
			self._check_range(servo[2], 0, 16383, 'time')

		send = [0xFA, 0xAF, 0x00, 0x00, 0x1E, 0x05, len(servo_data)]
		for servo in servo_data:
			send.append(servo[0])
			send.append(servo[1] & 0x00FF)
			send.append((servo[1] & 0xFF00) >> 8)
			send.append(servo[2] & 0x00FF)
			send.append((servo[2] & 0xFF00) >> 8)
		send.append(self._calc_checksum(send))

		self._write_serial(send, 0)

		return 'multi_target_position:' + str(servo_data)

	def get_selected_item(self, id=0x01, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_PRESENT_CURRENT_L, len=0x02, count=0x01, sign=False, endi='little'):
		self._check_range(id, 1, 127, 'id')

		send = [0xFA, 0xAF, id, flags_memmap, addr, len, count]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)
         
		receive = self.myserial.read(len)                  # may also use if self.myserial.in_waiting >0: receive = self.myserial.read_all()
        
        num = int.from_bytes(receive, endi, signed=sign)
		rec_nums = [ord(r) for r in receive]
		receive = [chr(r) for r in receive]
		print('received ', receive)

        return num, rec_nums, receive

    def get_current(self, sid=0x1):
        return self.get_selected_item(id=sid)
        
    def get_voltage(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_VOLTAGE_L, len=2, count=0x01, sign=True)  
        
    def get_target_pos(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_GOAL_POSITION_L, len=2, count=0x01, sign=True)

    def get_current_pos(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_PRESENT_POSITION_L, len=2, count=0x01, sign=True)

    def get_target_time(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_GOAL_TIME_L, len=2, count=0x01)

    def get_pid_coeff(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_PID_COEFFICIENT, len=1, count=0x01)

    def get_max_torque(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_MAX_TORQUE, len=1, count=0x01)

    def get_servo_id(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_SERVO_ID, len=1, count=0x01)
        
    def get_current_speed(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_PRESENT_SPEED_L, len=2, count=0x01, sign=True)

    def get_current_time(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_PRESENT_TIME_L, len=2, count=0x01, sign=False)
        
    def get_limit_cw_position(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_CW_ANGLE_LIMIT_L, len=2, count=0x01, sign=True)

    def get_limit_ccw_position(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_CCW_ANGLE_LIMIT_L, len=2, count=0x01, sign=True)        

    def get_limit_temperature(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_TEMPERATURE_LIMIT_L, len=2, count=0x01, sign=True) 

    def get_temperature(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_TEMPERATURE_L, len=2, count=0x01, sign=True) 

    def get_warm_up_time(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_WARM_UP_TIME, len=2, count=0x01) 

    def get_torque_in_silence(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_TORQUE_IN_SILENCE, len=2, count=0x01) 

    def get_punch(self, sid=0x1):
        return self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_PUNCH_L, len=2, count=0x01) 

    def get_compliance(self, sid=0x1):
        a, b, c = self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_CW_COMPLIANCE_MARGIN, len=2, count=0x01) 
        a1, b, c = self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_CCW_COMPLIANCE_MARGIN, len=2, count=0x01) 
        a2, b, c = self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_CW_COMPLIANCE_SLOPE, len=2, count=0x01) 
        a3, b, c = self.get_selected_item(id=sid, flags_memmap=self.FLAG30_MEM_MAP_SELECT, addr=self.ADDR_CCW_COMPLIANCE_SLOPE, len=2, count=0x01) 
        return a, a1, a2, a3
        
	def get_data(self, id=0x01, mode='all'):
		self._check_range(id, 1, 127, 'id')

		modes = ('all', 'angle', 'time', 'speed', 'load', 'tempreture', 'voltage', 'list')
		if mode not in modes:
			raise ValueError('mode is not defined, select from the list below\n'
							 + str(modes))

		elif mode is 'list':
			return modes

		send = [0xFA, 0xAF, id, 0x09, 0x00, 0x00, 0x01]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 26)

		receive = self.myserial.read(26)
		receive = [chr(r) for r in receive]
		print('receive', receive)

		# receive = hex(receive)

		try:
			angle = ((ord(receive[8]) << 8) & 0x0000FF00) | (ord(receive[7]) & 0x000000FF)
			time = ((ord(receive[10]) << 8) & 0x0000FF00) | (ord(receive[9]) & 0x000000FF)
			speed = ((ord(receive[12]) << 8) & 0x0000FF00) | (ord(receive[11]) & 0x000000FF)
			load = ((ord(receive[14]) << 8) & 0x0000FF00) | (ord(receive[13]) & 0x000000FF)
			tempreture = ((ord(receive[16]) << 8) & 0x0000FF00) | (ord(receive[15]) & 0x000000FF)
			voltage = ((ord(receive[18]) << 8) & 0x0000FF00) | (ord(receive[17]) & 0x000000FF)
		except IndexError:
			print('Could not get the data.Check the cables, connectors, and a power supply.')

		else:
			if angle > 1800:
				angle = -((angle - 1) ^ 0xFFFF)

			if mode is 'all':
				return id, angle, time, speed, load, tempreture, voltage
			elif mode is 'angle':
				return id, angle
			elif mode is 'time':
				return id, time
			elif mode is 'speed':
				return id, speed
			elif mode is 'load':
				return id, load
			elif mode is 'tempreture':
				return id, tempreture
			elif mode is 'voltage':
				return id, voltage

	def servo_reset(self, id=0x01):
		self._check_range(id, 1, 127, 'id')

		send = [0xFA, 0xAF, id, 0x20, 0xFF, 0x00, 0x00]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 0)

    def memory_reset(self, sid=0x01):
        self.send_command_byte(id=sid, val=0, ram_cmd_reg=self.ADDR_RESET_MEMORY, return_packet=self.FLAG4_RESET_MEMORY_MAP, cnt=0)    
        
	def set_torque_limit(self, id, limit=100):
		self._check_range(id, 1, 127, 'id')
		self._check_range(limit, 0, 100, 'limit')

		send = [0xFA, 0xAF, id, 0x01, 0x23, 0x01, 0x01, limit & 0x00FF]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)

		return self._check_ack(id)

	def set_damper(self, id, damper=16):
		self._check_range(id, 1, 127, 'id')
		self._check_range(damper, 0, 255, 'damper')

		send = [0xFA, 0xAF, id, 0x01, 0x20, 0x01, 0x01, damper & 0x00FF]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)

		return self._check_ack(id)

	def set_compliance(self, id, cwcm=1, ccwcm=1, cwcs=4, ccwcs=4, punch=1300):
		self._check_range(id, 1, 127, 'id')
		self._check_range(cwcm, 0, 255, 'cwcm')
		self._check_range(ccwcm, 0, 255, 'ccwcm')
		self._check_range(cwcs, 0, 255, 'cwcs')
		self._check_range(ccwcs, 0, 255, 'ccwcs')
		self._check_range(punch, 0, 10000, 'punch')

		send = [0xFA, 0xAF, id, 0x01, 0x18, 0x06, 0x01, cwcm & 0x00FF, ccwcm & 0x00FF,
				cwcs & 0x00FF, ccwcs & 0x00FF, punch & 0x00FF, (punch & 0xFF00) >> 8]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)

		return self._check_ack(id)

	def set_rpu(self):
		self.mode = 'rpu'

	def set_normal(self):
		self.mode = 'normal'

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

	def _check_ack(self, id):
		receive = self.myserial.read(1)
		length = len(receive)

		if length == 1:
			ack = ord(receive)
			if ack == 0x07:
				return id, 'ACK'
			elif ack == 0x08:
				return id, 'NACK'
			else:
				return id, 'unKnown'
		elif length != 1:
			return id, 'unReadable'

	def _write_rpu(self, send, length):
		if length == 0:
			send_rpu = [0x53, len(send)]
			send_rpu += send
		else:
			send_rpu = [0x54, len(send) + 1]
			send_rpu += send
			send_rpu.append(length)

		self.myserial.flushOutput()
		self.myserial.flushInput()
		self.myserial.write("".join(map(chr, send_rpu)).encode())

	def _write_command(self, send):
		self.myserial.flushOutput()
		self.myserial.flushInput()
		# print(bytearray(map(chr, send)))
		# print("".join(map(chr, send)))
		# print("".join(map(chr, send)).encode())
		# self.myserial.write(bytearray(map(chr, send)))
		# self.myserial.write(bytearray("".join(map(chr, send)).encode()))
		# self.myserial.write("".join(map(chr, send)).encode())
		self.myserial.write(bytearray(send))

	def _write_serial(self, send, length):
		if self.mode == 'rpu':
			self._write_rpu(send, length)
		else:
			self._write_command(send)

# TEST :: set torque on drive and move angle/position then read back all data
#
def main():
    rpts = 0
    while rpts < 2:
        frs_servo = FutabaRS_Servo()
        frs_servo.open_port()
        frs_servo.torque_on()
	    frs_servo.servo_reset()
        frs_servo.target_angle(140)	
	    frs_servo.target_angle(-140)
	    frs_servo.target_position(-1000,163)	
	    frs_servo.target_position(1200,1630)	
        angle, time, speed, load, temperature, voltage = frs_servo.get_data()
        print("angle ",angle," time ",time," speed ",speed," load ",load)
        print("temperature ",temperature," voltage ",voltage)
        pc, x1, x2 = frs_servo.get_pid_coeff()
        print("pid cooeff ", pc)
        c, x1, x2 = frs_servo.get_current()
        print("current ", c)
        p, x1, x2 = frs_servo.get_punch()
        print("punch ", p)
        rpts += 1
        if rpts == 1:
            frs_servo.set_rpu()              # now repeat with rpu mode set to on           
    frs_servo.close_port()	

LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        main()