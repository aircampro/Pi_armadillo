#!/usr/bin/env python
# lux sensor TSL2561-FN on i2c
#
import smbus
import time
bus = smbus.SMBus(1)
addr = 0x39                                                         # TSL2561-FN
while 1:
    bus.write_byte_data(addr , 0x80, 0x03)                          # power on
    bus.write_byte_data(addr , 0x81, 0x12)                          # timing 16x gain = 0x10, integration=402ms=0x02
    time.sleep(0.5)                                                 # wait at least integration time
    data = bus.read_i2c_block_data(addr , 0xAc, 2)                  # previously was 0x8c needed to include word bit of 0x20
    data1 = bus.read_i2c_block_data(addr , 0xAe, 2)                 # previosuly 0x8e ? now ored with 0x20 word_bit so 0xAe
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
    # luminescence
    x = data
    x <<= 16
    x |= data1
    luminoscity = (x & 0xFFFF) - (x >> 16)

    bus.write_byte_data(addr , 0x80, 0x0)                            # power off
    print(round(Lux,2)," lux ")
    print("Full ",ch0)                                               # ch0*0.03
    print("IR ",ch1)                                                 # ch1*0.03
    print("luminoscity ", luminoscity)
    time.sleep(1)