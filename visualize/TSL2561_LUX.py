#!/usr/bin/env python
# lux sensor TSL2561-FN on i2c
#
import smbus
import time
bus = smbus.SMBus(1)
addr = 0x39                                                         # TSL2561-FN
while 1:
        bus.write_byte_data(addr , 0x80, 0x03)
        bus.write_byte_data(addr , 0x81, 0x12)
        time.sleep(0.5)
        data = bus.read_i2c_block_data(addr , 0x8c, 2)
        data1 = bus.read_i2c_block_data(addr , 0x8e, 2)
        ch0 = data[1] *256 + data[0]
        ch1 = data1[1] *256 + data1[0]
        R = 1.0*ch1/ch0
        if 0 < R <= 0.5:
                Lux = 0.0304*ch0 - 0.062*ch0*(R**1.4)
        elif 0.5 < R <= 0.61:
                Lux = 0.0224*ch0 - 0.031*ch1
        elif 0.61 < R <= 0.8:
                Lux = 0.0128*ch0 - 0.0153*ch1
        elif 0.8 < R <= 1.3:
                Lux = 0.0146*ch0 - 0.00112*ch1
        elif R > 1.3:
                Lux = 0
 
        print(round(Lux,2)," lux ")
        print("Full ",ch0*0.03)
        print("IR ",ch1*0.03)
        time.sleep(1)