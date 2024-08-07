#!/usr/bin/env python
# simple CAN receiver for 15s to demonstrate the isoTP Can sender example
#
import can
import time

# define the bus
bus = can.interface.Bus(bustype='vector', channel=0, bitrate=500000, app_name='python-can')

#take start time as a reference
start_time = time.time()

while time.time() - start_time < 15 :                                               # for 15sec
	recv_msg = bus.recv(timeout=1)                                                  # receive or timeout
	if recv_msg != None:                                                            # something was received
		if hex(recv_msg.data[0]) == "0x10":                                         # data[0] is 10 thats the sender we have
		    print(recv_msg.data.encode('utf-8'))                                    # print the message received
			# send the arbotration command back
			msg = can.Message(arbitration_id = 0xF1,
		        data= [0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00],                    # Flow Control
		        is_extended_id = False)
			bus.send(msg)
