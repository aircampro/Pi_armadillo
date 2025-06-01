#!/usr/bin/env python
# AdaFruit Stemma LPS22HB pressure and temperature
#
import sys
# python > 3.6 uses smbus2 now for i2c communication
if sys.version_info[0] >= 3 and sys.version_info[1] >= 6:
    import smbus2 as smbus
else:
    import smbus
import time

bus_no = 1
i2c =  smbus.SMBus(bus_no)
addr = 0x5d
i2c.write_byte_data(addr, 0x10, 0x10)
time.sleep(0.1)

while 1:
    i2c.write_byte_data(addr, 0x11, 0x11)
    pdata_XL = i2c.read_byte_data(addr, 0x28)
    pdata_L  = i2c.read_byte_data(addr, 0x29)
    pdata_H  = i2c.read_byte_data(addr, 0x2a)
    press = (pdata_H<<16 | pdata_L << 8 | pdata_XL ) / 4096.0
    print("\nPress = " + str(int(press)) + "hPa")
    tdata_L = i2c.read_byte_data(addr, 0x2b)
    tdata_H = i2c.read_byte_data(addr, 0x2c)
    temp = (tdata_H << 8 | tdata_L ) / 100.0
    print("Temp  = " + str(round(temp,1)) + "`C")
    time.sleep(2)