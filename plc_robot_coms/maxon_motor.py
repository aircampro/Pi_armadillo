#
# control maxon epos drive on raspi ref:- https://github.com/mkhorasani/maxon_python_windows_64/blob/main/maxon_python_windows_64_example.py
#
# 1. Download and install EPOS2 libraries in Raspberry PI. Copy *.so in /usr/lib, and replace *.h in /usr/include. Replace the original "Definitions.h" file with the one included in the zip.
# Example of this step in linux:
# cd ./EPOS_Linux_Library_Files/lib/arm/rpi
# sudo mv libEposCmd.so.5.0.1.0 /usr/lib/libEposCmd.so
# sudo mv libftd2xx.so.1.2.8 /usr/lib/libftd2xx.so
# Copy the file "Definitions.h" to your SSD in Raspberry PI and use the following command
# sudo mv ./Definitions.h /usr/include
#
# other examples show the library to copy as /home/alg_sys/python/libEposCmd.so.6.6.1.0
# that depends on where you installed it and the version but please copy as above
#
import serial
import time
from ctypes import *

# EPOS Command Library path if on windows
# path='.\EposCmd64.dll'
# library for linux reaspberry pi
path='/usr/lib/libEposCmd.so'

# Load library
cdll.LoadLibrary(path)
epos = CDLL(path)

# Defining return variables from Library Functions
ret = 0
pErrorCode = c_uint()
pDeviceErrorCode = c_uint()

# Defining a variable NodeID and configuring connection
nodeID = 1
keyhandle=0
baudrate = 1000000
timeout = 500

# Configure desired motion profile
acceleration = 30000                                                                                                         # rpm/s, up to 1e7 would be possible
deceleration = 30000                                                                                                         # rpm/s

# Read Statusword and mask it to bit12
def WaitAcknowledged():
	ObjectIndex=0x6041
	ObjectSubindex=0x0
	NbOfBytesToRead=0x02
	pNbOfBytesRead=c_uint()
	pData=c_uint()
	pErrorCode=c_uint()

	# Setpoint Acknowledged
	Mask_Bit12=0x1000
	Bit12=0
	i=0

	while True:
		# Read Statusword
		ret=epos.VCS_GetObject(keyhandle, NodeID, ObjectIndex, ObjectSubindex, byref(pData), NbOfBytesToRead, byref(pNbOfBytesRead), byref(pErrorCode) )
		Bit12=Mask_Bit12&pData.value

		# Timed out
		if i>20:
			return 0
			break

		if Bit12==Mask_Bit12:
			time.sleep(1)
			i+=1

		# Bit12 reseted = new profile started
		else:
			return 1
			break

# Query motor position
def GetPositionIs():
    pPositionIs=c_long()
    pErrorCode=c_uint()
    ret=epos.VCS_GetPositionIs(keyHandle, nodeID, byref(pPositionIs), byref(pErrorCode))
    return pPositionIs.value                                                                                                  # motor steps

# Move to position at speed
def MoveToPositionSpeed(target_position,target_speed):
    true_position = -99999
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, nodeID, target_speed, acceleration, deceleration, byref(pErrorCode))      # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, nodeID, target_position, True, True, byref(pErrorCode))                       # move to position
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode))                                               # halt motor
        if WaitAcknowledged() == 0:
            print("position set timeout occurred")
        true_position = GetPositionIs()
        if true_position == target_position:
            break
    return true_position 

if __name__ == "__main__":
    global keyhandle
    # Initiating connection and setting motion profile
    keyHandle = epos.VCS_OpenDevice(b'EPOS2', b'MAXON SERIAL V2', b'USB', b'USB0', byref(pErrorCode))                          # specify EPOS version and interface
    epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode))                                         # set baudrate
    epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode))                                                                  # clear all faults
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode))                                                 # activate profile position mode
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode))                                                              # enable device

    tp = MoveToPositionSpeed(20000, 5000)                                                                                      # move to position 20,000 steps at 5000 rpm/s
    if tp != -99999:
        print('Motor position: %s' % (tp))
        time.sleep(1)
    else:
        print("error getting positon")

    tp = MoveToPositionSpeed(0, 2000)                                                                                           # move to position 0 steps at 2000 rpm/s
    if tp != -99999:
        print('Motor position: %s' % (tp))
        time.sleep(1)
    else:
        print("error getting positon")

    epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))                                                              # disable device
    epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))                                                                          # close device

-------------------------------

# -*- coding: utf-8 -*-
"""
Example to command EPOS4 with python
Operation Mode: Profile Position Mode, 10cycles repeated
No Homing

Python Version 2.7
Tested under Ubuntu 16.04 and Raspberry Pi 3

Created on 16.03.2020
Version 1.0
@author: CCMC, maxon motor ag, Switzerland

"""

from ctypes import *

# sleep function
import time

# Folder created for example: /home/pi/src/python/
# Copy maxon motor Linux Library arm v7 into this folder
# Library must match according your cpu, eg. PI3 has arm v7
# EPOS Comand Library can be found here, when EPOS Studio has been installed:
# C:\Program Files (x86)\maxon motor ag\EPOS IDX\EPOS4\04 Programming\Linux Library
path='/home/pi/src/python/libEposCmd.so.6.6.1.0'
# path='/home/alg_sys/python/libEposCmd.so.6.6.1.0'
cdll.LoadLibrary(path)
epos=CDLL(path)

# Node ID must match with Hardware Dip-Switch setting of EPOS4
NodeID=4
keyhandle=0
# return variable from Library Functions
ret=0
pErrorCode=c_uint()
pDeviceErrorCode=c_uint()


## ============================================================================================
# Read Statusword and mask it to bit12
def WaitAcknowledged():
	ObjectIndex=0x6041
	ObjectSubindex=0x0
	NbOfBytesToRead=0x02
	pNbOfBytesRead=c_uint()
	pData=c_uint()
	pErrorCode=c_uint()

	# Setpoint Acknowledged
	Mask_Bit12=0x1000
	Bit12=0
	i=0

	while True:
		# Read Statusword
		ret=epos.VCS_GetObject(keyhandle, NodeID, ObjectIndex, ObjectSubindex, byref(pData), NbOfBytesToRead, byref(pNbOfBytesRead), byref(pErrorCode) )
		Bit12=Mask_Bit12&pData.value

		# Timed out
		if i>20:
			return 0
			break

		if Bit12==Mask_Bit12:
			time.sleep(1)
			i+=1

		# Bit12 reseted = new profile started
		else:
			return 1
			break



def CyclicMode(pErrorCode):

	print('Wait finishing positioning...')

	for x in range(1, 11):
		print('Loop: %d' % x)

		# TargetPosition=20'000qc / AbsolutMovement=0 =>Relative Positioning / StartProfileImmediately=0
		ret=epos.VCS_MoveToPosition(keyhandle, NodeID, 20000, 0, 0, byref(pErrorCode) )

		ret=WaitAcknowledged()

		# Send new profile during execution of previous profile
		ret=epos.VCS_MoveToPosition(keyhandle, NodeID, -20000, 0, 0, byref(pErrorCode) )

		ret=WaitAcknowledged()

	print('Cyclic movemenent finished')

	return 1



# Example of using VCS_GetObject()
# With this function any CANopen Object can be accessed
def GetPosition():
	# CANopen Object: Position Actual Value
	ObjectIndex=0x6064
	ObjectSubIndex=0x00
	NbOfBytesToRead=0x04
	# DWORD
	NbOfBytesRead=c_uint()
	# 0x6064 => INT32
	pData=c_int()
	pErrorCode=c_uint()

	ret=epos.VCS_GetObject(keyhandle, NodeID, ObjectIndex, ObjectSubIndex, byref(pData), NbOfBytesToRead, byref(NbOfBytesRead), byref(pErrorCode) )

	if ret==1:
		print('Position Actual Value: %d [inc]' % pData.value)
		return 1
	else:
		print('GetObject failed')
		return 0



# Get Position direct via function
def GetPositionIs():
	pPositionIs=c_long()
	pErrorCode=c_uint()

	ret=epos.VCS_GetPositionIs(keyhandle, NodeID, byref(pPositionIs), byref(pErrorCode) )

	if ret==1:
		print('Position Actual Value: %d [inc]' % pPositionIs.value)
		return 1
	else:
		print('GetPositionIs failed')
		return 0



## ============================================================================================
# Main
# Open USB Port
print('Opening Port...')
keyhandle=epos.VCS_OpenDevice('EPOS4', 'MAXON SERIAL V2', 'USB', 'USB0', byref(pErrorCode) )

if keyhandle != 0:

	print('Keyhandle: %8d' % keyhandle)

	# Verify Error State of EPOS4
	ret=epos.VCS_GetDeviceErrorCode(keyhandle, NodeID, 1, byref(pDeviceErrorCode), byref(pErrorCode) )
	print('Device Error: %#5.8x' % pDeviceErrorCode.value )

	# Device Error Evaluation
	if pDeviceErrorCode.value==0:

		# Set Operation Mode PPM
		ret=epos.VCS_ActivateProfilePositionMode(keyhandle, NodeID, byref(pErrorCode) )

		# Profile Velocity=500rpm / Acceleration=1000rpm/s / Deceleration=1000rpm/s
		ret=epos.VCS_SetPositionProfile(keyhandle, NodeID, 500, 1000, 1000, byref(pErrorCode) )

		# Read Position Actual Value with VCS_GetObject()
		ret=GetPosition()

		ret=epos.VCS_SetEnableState(keyhandle, NodeID, byref(pErrorCode) )
		print('Device Enabled')

		ret=CyclicMode(pErrorCode)

		ret=epos.VCS_SetDisableState(keyhandle, NodeID, byref(pErrorCode) )
		print('Device Disabled')

		# Other Option to Read Position
		ret=GetPositionIs()

	else:
		print('EPOS4 is in Error State: %#5.8x' % pDeviceErrorCode.value)
		print('EPOS4 Error Description can be found in the EPOS4 Fimware Specification')

	# Close Com-Port
	ret=epos.VCS_CloseDevice(keyhandle, byref(pErrorCode) )
	print('Error Code Closing Port: %#5.8x' % pErrorCode.value)

else:
	print('Could not open Com-Port')
	print('Keyhandle: %8d' % keyhandle)
	print('Error Openening Port: %#5.8x' % pErrorCode.value)
	
==========================================

