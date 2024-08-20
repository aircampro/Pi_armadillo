#!/usr/bin/env python
# ref :- https://qiita.com/ToyoshiMorioka/items/8f92121f6bf6b9b6d9a0
# example of oriental drive on modbus serial
# modified from the example shown to allow the changing of the motor address and calculating the crc
# e.g. AZD-KD drive of Oriental Motor
#
import time
import serial

import do_modbus_crc

#drive address here we are set to 01
d_addr = b"\x01"
d_port="COM6"                      # windows port6
# if on raspi linux ---            d_port="/dev/ttyS0"

# open the specified serial port to the drive
client = serial.Serial(d_port, 115200, timeout=0.01, parity=serial.PARITY_EVEN,stopbits=serial.STOPBITS_ONE) 
size = 16                  # Divide the size appropriately
print(client.name)

# ------- run drive at 2000Hz for 10 seconds ----------
#
# set write method ?? - func_code is 0x10 preset multiple registers -- please check the manual for your specific drive
# slave_id\ func_code st_addr     read_num        ?           data               crc
# \x01\     x10\      x18\x04\    x00\x02\        x04\        x00\x00\x07\xd0    \x5b\xf0"
#
# this was d_addr=01 with crc hardcoded -- commando = d_addr + b"\x10\x18\x00\x00\x02\x04\x00\x00\x00\x02\xd8\x6e"
comman = d_addr + b"\x10\x18\x00\x00\x02\x04\x00\x00\x00\x02"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
# Sending continuous commands at short intervals is not allowed, so wait for the PLC to return the command results
result = client.read(size) 
print(result)
# MIN～MAX The value set by the soft limit can be specified from MIN to MAX. 2134=8500
# commando = d_addr + b"\x10\x18\x02\x00\x02\x04\x00\x00\x21\x34\xc1\xf1"
comman = d_addr + b"\x10\x18\x02\x00\x02\x04\x00\x00\x21\x34"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
# Speed ​​(Hz) 2000
# commando = d_addr + b"\x10\x18\x04\x00\x02\x04\x00\x00\x07\xd0\x5b\xf0"
comman = d_addr + b"\x10\x18\x04\x00\x02\x04\x00\x00\x07\xd0"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
# Start rate of change kHz/s 1500
# commando = d_addr + b"\x10\x18\x06\x00\x02\x04\x00\x00\x05\xdc\xdb\x4c"
comman = d_addr + b"\x10\x18\x06\x00\x02\x04\x00\x00\x05\xdc"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
# Specified rate kHz/s 1500
# commando = d_addr + b"\x10\x18\x08\x00\x02\x04\x00\x00\x05\xdc\x5a\xc0"
comman = d_addr + b"\x10\x18\x08\x00\x02\x04\x00\x00\x05\xdc"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
# Current 100
# 
comman = d_addr + b"\x10\x18\x10\x00\x02\x04\x00\x00\x00\x64"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
# drive on data 8
# commando = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x08\xf5\x18"
comman = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x08"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
time.sleep(10)
# drive off data 0
# commando = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x00\xf4\xde"
comman = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x00"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)

# ------- now change the drive speed and run again for 10 seconds ----------
#
# Speed ​​(Hz) 1000 this time its set as configurably written 
# commando = d_addr + b"\x10\x18\x04\x00\x02\x04\x00\x00\x03\xe8\xXX\xYY"
sped = do_modbus_crc.number_to_bytes( 1000 )
comman = d_addr + b"\x10\x18\x04\x00\x02\x04\x00\x00"
comman = comman + sped
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
# drive on data 8
# commando = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x08\xf5\x18"
comman = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x08"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
time.sleep(10)
# drive off data 0
# commando = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x00\xf4\xde"
comman = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x00"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)

# drive on data 8 start the motor in this example we will compute using a 2nd crc function to do the same calc
# commando = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x08\xf5\x18"
comman = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x08"
crc_cal = do_modbus_crc.get_modbus_crc2(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)
for speeds in [10,100,200,500,750,1000,1300,1600,1800,2000]:
    sped = do_modbus_crc.number_to_bytes( speeds )
    comman = d_addr + b"\x10\x18\x04\x00\x02\x04\x00\x00"
    comman = comman + sped
    crc_cal = do_modbus_crc.get_modbus_crc2(comman,1)
    commando = comman + crc_cal
    client.write(commando)
    result = client.read(size)
    print(result)
    time.sleep(2)
# drive off data 0
# commando = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x00\xf4\xde"
comman = d_addr + b"\x10\x00\x7c\x00\x02\x04\x00\x00\x00\x00"
crc_cal = do_modbus_crc.get_modbus_crc2(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result) 

# read data
comman = d_addr + b"\x03\x00\x7f\x00\x01"
crc_cal = do_modbus_crc.get_modbus_crc(comman,1)
commando = comman + crc_cal
client.write(commando)
result = client.read(size)
print(result)

# close the serial port connection to the drive as we have finished
client.close()