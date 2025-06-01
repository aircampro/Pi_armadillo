#!/usr/bin/env python
# angle measurement 
# https://ams.com/ja/as5147padapterboard/#tab/description
# AS5147 magnetic encoder
#
import spidev
import time
 
spi = spidev.SpiDev()
spi.open(0,0)                                # CE0
spi.max_speed_hz = 10000000                  # 10MHz
spi.mode = 0b01
 
spi.xfer2([0x00,0x18])
spi.xfer2([0x00,0x00])
spi.xfer2([0x00,0x19])
spi.xfer2([0x00,0x00])
 
while 1:
    angle = spi.xfer2([0x3f,0xff])
    time.sleep(0.001)
    data = ((angle[0] & 0b00111111) << 8) | angle[1]
    print(data, round((data/16384*360),1) , "`")
