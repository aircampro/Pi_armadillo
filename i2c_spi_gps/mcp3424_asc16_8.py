#!/usr/bin/env python
# reading a mcp3424 as 16 or 18 bit input 0-5v
#
import sys
# python 3.6 > supports smbus2 now for i2c communication
if sys.version_info[0] >= 3 and sys.version_info[1] >= 6:
    import smbus2 as smbus
else:
    import smbus
import time

bus_no=1
i2c = smbus.SMBus(bus_no)
addr=0x68
Vref=2.048

def swap16(x):
    return (((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF))

def sign16(x):
    return ( -(x & 0b1000000000000000) | (x & 0b0111111111111111) )

def sign18(x):
    return ( -(x & 0b100000000000000000) | (x & 0b011111111111111111) )

# set to either 16 or 18 bits registers 
conf_bits = 16
if conf_bits == 16:
    config = 0b10011000                         # 16 bit
elif conf_bits == 18:
    config = 0b10011100                         # 18 bit
else:
    print("conf_bits must be either 16 or 18")
    sys.exit(-1)

i2c.write_byte(addr, config) 
time.sleep(0.2)

#main
#
while True:
    if conf_bits == 16:
        data = i2c.read_word_data(addr,0x00)
        raw = swap16(int(hex(data),16))
        raw_s = sign16(int(hex(raw),16))
        volts = round((Vref * raw_s / 32767),5)
    elif conf_bits == 18:
        data = i2c.read_i2c_block_data(addr, config, 3) #18bit
        raw = ((data[0] & 0b00000011) << 16) | (data[1] << 8) | (data[2])
        raw_s = sign18(int(hex(raw),16))
        volts = round((Vref * raw_s / 131071),7)
    print (str(volts) +"V")
    time.sleep(1)	