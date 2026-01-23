# -*- coding: utf-8 -*-
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
import time
from ctypes import *
import sys

# EPOS Command Library path if on windows
# path='.\EposCmd64.dll'
ps = platform.system()
if ps == "Linux":
    # library for linux reaspberry pi
    path='/usr/lib/libEposCmd.so'
elif ps == 'Windows':
    path='.\EposCmd64.dll'

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
FAILURE=-99999
POS_DB = 0.5

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
		return pData.value
	else:
		print('GetObject failed')
		return FAILURE

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
    if ret == 1:
        return pPositionIs.value                                                                                                  # motor steps
    else:
        return FAILURE

# Move to position at speed
def MoveToPositionSpeed(target_position, target_speed):
    true_position = FAILURE
    while True:
        if target_speed != 0:
            ret=epos.VCS_SetPositionProfile(keyHandle, nodeID, target_speed, acceleration, deceleration, byref(pErrorCode))      # set profile parameters
            ret=epos.VCS_MoveToPosition(keyHandle, nodeID, target_position, True, True, byref(pErrorCode))                       # move to position
        elif target_speed == 0:
            ret=epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode))                                              # halt motor
        if ret != 1:                                                                                                             # error on setting the motor to position
            print("set position command failed %#5.8x" % pErrorCode.value) 
            break
        if WaitAcknowledged() == 0:
            print("position set timeout occurred")
        true_position = GetPositionIs()
        if abs(true_position - target_position) < POS_DB:
            break
    return true_position 

if __name__ == "__main__":
    global keyhandle
    # Initiating connection and setting motion profile
    keyHandle = epos.VCS_OpenDevice(b'EPOS2', b'MAXON SERIAL V2', b'USB', b'USB0', byref(pErrorCode))                          # specify EPOS version and interface
    if keyhandle == 0:
        print("could not open device")
	    print('Error Openening Port: %#5.8x' % pErrorCode.value)
        sys.exit(-1)
    ret=epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode))                                     # set baudrate
    ret=epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode))                                                              # clear all faults
    ret=epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode))                                             # activate profile position mode
    ret=epos.VCS_SetPositionProfile(keyhandle, NodeID, 500, 1000, 1000, byref(pErrorCode) )  		                           # Profile Velocity=500rpm / Acceleration=1000rpm/s / Deceleration=1000rpm/s
    ret=GetPosition()                                                                                   	                   # Read Position Actual Value with VCS_GetObject()
    ret=epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode))                                                          # enable device

    tp = MoveToPositionSpeed(20000, 5000)                                                                                      # move to position 20,000 steps at 5000 rpm/s
    if tp != FAILURE:
        print('Motor position: %s' % (tp))
        time.sleep(1)
    else:
        print("error getting positon")

    tp = MoveToPositionSpeed(0, 2000)                                                                                           # move to position 0 steps at 2000 rpm/s
    if tp != FAILURE:
        print('Motor position: %s' % (tp))
        time.sleep(1)
    else:
        print("error getting positon")

    ret=epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))                                                              # disable device
    ret=epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))                                                                          # close device


