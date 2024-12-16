#!/usr/bin/env python3
#
# Robotiq f2-85: https://robotiq.com/products/2f85-140-adaptive-robot-gripper
#
import cython
import libscrc
import serial
import time

# define the serial link global to the device
#
GRIPPER_PORT='/dev/ttyUSB0'
GRIPPER_BAUD=115200
ser = serial.Serial(port=GRIPPER_PORT, baudrate=GRIPPER_BAUD, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

def grip_check(ser):
    data_raw = list(ser.readline())
    time.sleep(0.001)
    ser.write(b'\x09\x03\x07\xD0\x00\x03\x04\x0E')
    time.sleep(0.001)
    data_raw = list(ser.readline())
    if data_raw[3] == 185:
        grip = 'ok'
    else:
        grip = 'not ok'
    return grip

def gri_set(ser):
    ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
    data_raw = ser.readline()
    time.sleep(0.01)
    ser.write(b'\x09\x03\x07\xD0\x00\x01\x85\xCF')
    data_raw = ser.readline()
    time.sleep(0.01)
    ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19')
    
def gripper_command(Pos, speed, force):
    crc_cal, command, li_s  = [9,16,3,232,0,3,6,9,00,00,00,255,255], [9,16,3,232,0,3,6,9,00,00,00,255,255,114,41],['0','0','0','0']
    command[10], command[11], command[12] = Pos, speed, force
    for j in range(0, len(crc_cal)):
        crc_cal[j] = command[j]
    crc_cal = bytes(crc_cal)
    crc = libscrc.modbus(crc_cal)
    crc = str(hex(crc))
    crc = crc.replace('0x','')
    li = [char for char in  crc ]
    if len(li) == 3:
        li_s[1] = li[0]
        li_s[2] = li[1]
        li_s[3] = li[2]
        li1 = [li_s[0], li_s[1]]
        li2 = [li_s[2], li_s[3]]
    elif len(li) == 2:
        li_s[2] = li[0]
        li_s[3] = li[1]
        li1 = [li_s[0], li_s[1]]
        li2 = [li_s[2], li_s[3]]
    elif len(li) == 1:
        li_s[3] = li[0]
        li1 = [li_s[0], li_s[1]]
        li2 = [li_s[2], li_s[3]]
    else:
       li1 =[li[0],li[1]]
       li2 =[li[2],li[3]]
    hex1 = ''
    hex1 = bytes(hex1.join(li1),'utf-8')
    hex2 = ''
    hex2 = bytes(hex2.join(li2),'utf-8')
    hex1 = int(hex1,16)
    hex2 = int(hex2,16)
    command[13] = hex2
    command[14] = hex1
    final_command = bytes(command)
    return(final_command)

def gripper_open(ser):
    ser.write(gripper_command(220, 250, 250))

def gripper_close(ser):
    ser.write(gripper_command(0, 250, 250))    

if __name__ == "__main__":

    if grip_check(ser) == 'ok':
        gripper_open(ser)
        time.sleep(2.0)
        gripper_close(ser)
    ser.close()

