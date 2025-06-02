# AQM Series Character Display
#
# Pin 3 (CS) is connected to GND.
# Pin 4 is connected to either VDD or GND.
# The slave address of the I2C changes depending on which connection it is connected to.
# Pin 8 and pin 9 are used together as SDA.
# Since the pull-up of SCL and SDA is done inside RasPi, it is unnecessary.
#
# sudo apt-get install python-smbus
#
import sys
# python 3.6 > supports smbus2 now for i2c communication
if sys.version_info[0] >= 3 and sys.version_info[1] >= 6:
    import smbus2 as smbus
else:
    import smbus
import time

# class for AQM Series Character Display
#
class AQM_SCD():

    def __init__(self, cursor = False, blink = False, busno=1):
        self.bus = smbus.SMBus(busno)
        self.addr = 0x3e
        self.lcdcu_init()
        self.lcdcu_clear()

    # initialize the display
    def lcdcu_init(self):
        self.bus.write_byte_data(self.addr, 0x00, 0x38)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x39)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x14)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x73)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x56)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x6c)
        time.sleep(3.0)
        self.bus.write_byte_data(self.addr, 0x00, 0x38)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x0c)
        time.sleep(0.2)
        self.bus.write_byte_data(self.addr, 0x00, 0x01)
        time.sleep(0.2)

    # set the cursor before the write e.g. x 0 y0 first line x0 y1 next line		
    def lcdcu_set(self, x=0, y=0):
        ca = (x + y * 0x40) | (0x80) 
        self.bus.write_byte_data(self.addr, 0x00, ca)
        time.sleep(0.1)

    # clears lcd
    def lcdcu_clear(self): 
        self.bus.write_byte_data(self.addr, 0x00, 0x01)
        time.sleep(0.1)

    # writes the string shown
    def lcdcu_write(self, stri): 
        for ch in stri:
            self.bus.write_byte_data(self.addr, 0x40, ord(ch))
            time.sleep(0.1)