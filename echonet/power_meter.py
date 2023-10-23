#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Read EchoNet power meter and transfer data to specified influx database
"""
from __future__ import print_function

import sys
import serial
import time
import random
import ctypes #unit32
import datetime

#-----------------------set-up------------------------
#influxDB
from influxdb import InfluxDBClient
client = InfluxDBClient(host='192.168.31.53', port=8086, username='root', password='password', database='sensor')
measurement = 'power'
tags = {'place': 'umi','host': 'RL7023'}

# text file name
TXT_FL='/home/pi/eNet7.txt'

# B Root Authentication ID
rbid  = "The ID I got in the B route"
# B Root authentication password
rbpwd = "password"
# serial port device name
serialPortDev = '/dev/ttyUSB_power' 

#--------------------initialise------------------------ 

# Serial port initialization
ser = serial.Serial(serialPortDev, 115200)
# # Try to get the version for the time being (even if you don't do it)
ser.write("SKVER\r\n")
print(ser.readline(), end="") # echo back
print(ser.readline(), end="") # version

# B Root authentication password setting
ser.write("SKSETPWD C " + rbpwd + "\r\n")
print(ser.readline(), end="") # echo back
print(ser.readline(), end="") # version

# B Root authentication ID
ser.write("SKSETRBID " + rbid + "\r\n")
#print(ser.readline(), end="") # echo back
#print(ser.readline(), end="") # version

scanDuration = 4;   # Scan time.In the sample it is 6, but I can go even 4.(If not, increase it and try again)
scanRes = {}        # Container of scan results

# upload to influx database
def influx_json(field, nowtime=''):
	json_body = [
		{
			'measurement': measurement,
			'tags': tags,
			'fields': field
			'time': nowtime
		}
	]
	client.write_points(json_body)
	print(json_body)
	#sys.exit()

# Scan retry loop (until something is found)
while not scanRes.has_key("Channel") :
	# Perform active scan (with IE)
    # It takes time.about 10 seconds?
	ser.write("SKSCAN 2 FFFFFFFF " + str(scanDuration) + "\r\n")

	# Loop to end of scan for 1 scan
	scanEnd = False
	while not scanEnd :
		line = ser.readline()
		print(line, end="")

		if line.startswith("EVENT 22") :
			# The scan is over (whether it's found or not)
			scanEnd = True
		elif line.startswith("  ") :
			#  When I scan and find it, I open 2 spaces and the data comes
			#  
			#  Channel:39
			#  Channel Page:09
			#  Pan ID:FFFF
			#  Addr:FFFFFFFFFFFFFFFF
			#  LQI:A7
			#  PairID:FFFFFFFF
			cols = line.strip().split(':')
			scanRes[cols[0]] = cols[1]
	scanDuration+=1


	if 7 < scanDuration and not scanRes.has_key("Channel"):
		# You can specify up to 14 as an argument, but if you fail with 7, it is useless
		print("\033[33m Scan Retry .... \033[0m")
		sys.exit(3)  # exit and return a 3 to the stack

# set the channel according to the scan result。
ser.write("SKSREG S2 " + scanRes["Channel"] + "\r\n")
print(ser.readline(), end="") # echo back
print(ser.readline(), end="") # OK should come (no check)

# Set Pan ID from scan result
ser.write("SKSREG S3 " + scanRes["Pan ID"] + "\r\n")
print(ser.readline(), end="") # echo back
print(ser.readline(), end="") # OK should come (no check)

# Convert MAC address (64bit) to IPV6 link local address.
# (I am converting using the function of BP35A1, but there is also a story that it is just a string conversion?？)
ser.write("SKLL64 " + scanRes["Addr"] + "\r\n")
print(ser.readline(), end="") # echo back
ipv6Addr = ser.readline().strip()
print(ipv6Addr)

# Start the PANA connection sequence.
ser.write("SKJOIN " + ipv6Addr + "\r\n");
print(ser.readline(), end="") # echo back
print(ser.readline(), end="") # OK should come (no check)

# Wait for PANA connection to complete (return about 10 lines)
bConnected = False
while not bConnected :
	line = ser.readline()
	print(line, end="")
	if line.startswith("EVENT 24") :
		print("\033[31m PANA Connection Failed .... \033[0m")
		sys.exit(4)  # exit with state 4
	elif line.startswith("EVENT 25") :
		# PANA Connection Sucess
		print("\033[32m PANA Connection Successful .... \033[0m")
		bConnected = True

# After this, set the serial communication timeout
ser.timeout = 2

# Smart meter throws instance list notification
# (ECHONET-Lite_Ver.1.12_02.pdf p.4-16)
print(ser.readline(), end="") # skip this read

# === If you change the array of powers, you may be able to get other values as well, so try it. ===
#powers = ["\xD3","\xD7","\xE0","\xE1","\xE2","\xE3","\xE4","\xE5","\xE7","\xE8","\xEA","\xEB"]
#powers = ["\xE0","\xE1","\xE2","\xE3","\xE4","\xE5","\xE7","\xE8","\xEA","\xEB"]
powers = ["\xE0","\xE1","\xE3","\xE7","\xE8","\xEA"]
# for echonet in powers:
while powers:
	echonetLiteFrame = ""
	echonetLiteFrame += "\x10\x81"	  # EHD (Reference: EL p.3-2)
	echonetLiteFrame += "\x00\x01"	  # TID (reference: EL p.3-3)
	# ここから EDATA
	echonetLiteFrame += "\x05\xFF\x01"  # SEOJ (Reference: EL p.3-3 AppH p.3-408～)
	echonetLiteFrame += "\x02\x88\x01"  # DEOJ (Reference: EL p.3-3 AppH p.3-274～)
	echonetLiteFrame += "\x62"		    # ESV (62: Property value read request) (Reference: EL p.3-5)
	echonetLiteFrame += "\x01"		    # OPC (1 piece) (Reference: EL p.3-7)

	randp = random.choice(powers)
	echonetLiteFrame += randp
	print(randp)

	echonetLiteFrame += "\x00"
	command = "SKSENDTO 1 {0} 0E1A 1 {1:04X} {2}".format(ipv6Addr, len(echonetLiteFrame), echonetLiteFrame)
	ser.write(command)

	print(ser.readline(), end="") # echo back
	print(ser.readline(), end="") # EVENT 21 should come (unchecked)
	print(ser.readline(), end="") # OK
	line = ser.readline()		  # ERXUDP
	print(line, end="")

	if line.startswith("ERXUDP") :
		cols = line.strip().split(' ')
		res = cols[8]                   # UDP received data portion field 9
		seoj = res[8:8+6]               # position 6 plus 6
		ESV = res[20:20+2]              # position 20 plus 2
		print(f" ESV : {ESV}")

		if seoj == "028801" and ESV == "72" :
			EPC = res[24:24+2]
			#print(EPC)
            date_time_now = datetime.datetime.now()
            
			if EPC == "D3" :            # Coefficient D304 00000001=1
				print(EPC)
			elif EPC == "D7" :          # Integrated energy precision D701 06=6 digits
				print(EPC)

			elif EPC == "E0" :          # Integrated energy measurement (positive measurement) E004 00022721=14108.9 0002272A=14109.8kWh
				hexPower = line[-8:]	# The last 4 bytes (base 16, 8 characters) are instantaneous power measurements
				intPower = int(hexPower, 16)
				print(EPC)
				print(u"sekisan{0}[kWh]".format(intPower))
				field = {'E0': float(intPower)}
				influx_json(field, date_time_now)

			elif EPC == "E1" :          # Integrated energy unit (positive, reverse measurement) E10101=0.1kWh
				hexPower = line[-4:]	# last 4 bytes
				intPower = int(hexPower, 16)
				print(EPC)
				print(u"sekisan seigyaku{0}[kWh]".format(intPower))
				field = {'E1': float(intPower)}
				influx_json(field, date_time_now)

			elif EPC == "E2" :          # Accumulated energy measurement history 1 (positive measurement)
				print(EPC)

			elif EPC == "E3" :          # Integrated energy measurement (reverse measurement) E304 FFFFFFFE=4294967294
				hexPower = line[-8:]	# last 4 bytes
				intPower = int(hexPower, 16)
				print(EPC)
				print(u"sekisan Reverse{0}[kWh]".format(intPower))
				field = {'E3': float(intPower)}
				influx_json(field, date_time_now)

			elif EPC == "E4" :          # Total energy measurement history 1 (reverse measurement
				print(EPC)
			elif EPC == "E5" :          # Accumulation history collection date 1:E501FF
				print(EPC)

			elif EPC == "E7" :          # Instantaneous power measured value: E704 000000CE=206W
				hexPower = line[-8:]	# last 4 bytes
				intPower = int(hexPower, 16)
				print(EPC)
				print(u"power:{0}[W]".format(intPower))
				field = {'power': float(intPower) }
				influx_json(field, date_time_now)

				# write the data to a text file
				with open(TXT_FL, 'w') as f:
					print(float(intPower), file=f)

			elif EPC == "E8" :   # Instantaneous current (A) Current458792 Last 2 bytes 2 pieces
				hexCurR = line[-8:-4]
				hexCurT = line[-4:]
				intCurR = int(hexCurR, 16)
				intCurT = int(hexCurT, 16)
				intCurR = float(intCurR)
				intCurT = float(intCurT)
				intCurR = intCurR / 100
				intCurT = intCurT / 100

				print(EPC)
				#print(hexCurR)
				print(u"R:{0}[A]".format(intCurR)) 
				print(u"T:{0}[A]".format(intCurT)) 
				field = {'E8_R': intCurR , 'E8_T': intCurT }
				influx_json(field, date_time_now)

			elif EPC == "EA" :          # Fixed time accumulated energy positive direction meter number EA0B 07E3=2019 year 0C=12 month 06=6 day 0A=10 hour 00=0 minute 00=0 second 0002276C=14116
				hexYearKW = line[-22:-18]
				hexMoonKW = line[-18:-16]
				hexDayKW = line[-16:-1]
				hexTimeKW = line[-8:]	# last 4 bytes
				nowtime = '{} {}:{}:{}'.format(hexYearKW, hexMoonKW, hexDayKW, hexTimeKW)
				print(u"Now {0}".format(nowtime))
				intTimeKW = int(hexTimeKW, 16)  # kWh

				print(EPC)
				print(u"teizi{0}[kWh]".format(intTimeKW))
				field = {'EA': float(intTimeKW) }
				influx_json(field, nowtime)
				#influx_json(field)

			elif EPC == "EB" :  # Accumulation history collection date 1:EB0B07E30C050B1E00FFFFFFFE
				print(EPC)
			elif EPC == "EC" :  # Accumulated energy measurement history 2 (positive direction, reverse measurement) Not available in our smart meter
				print(EPC)
			elif EPC == "ED" :  # Accumulated history collection date 2 Not in our smart meter
				print(EPC)
			else :
				print("Other")
				print(EPC)

time.sleep(1)
