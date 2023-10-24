# OLED character display, SO1602A series sold in Akizuki
#
# Pin 3 (CS) is connected to GND.
# Pin 4 is connected to either VDD or GND.
# The slave address of the I2C changes depending on which connection it is connected to.
# Pin 8 and pin 9 are used together as SDA.
# Since the pull-up of SCL and SDA is done inside RasPi, it is unnecessary.
#
# sudo apt-get install python-smbus
#
# The slave address of the SO1602A is either 0x3C (SA0=Low) or 0x3D (SA0=High)
#
from smbus2 import SMBus

class SO1602A():

    def __init__(self, sa0 = 0, cursor = False, blink = False):
        self.bus = smbus.SMBus(1)
        if (sa0 == 0):
            self.addr = 0x3c
        else:
            self.addr = 0x3d
        self.clearDisplay()
        self.returnHome()
        self.displayOn(cursor, blink)
        self.clearDisplay()

    # for each value we send the following as per the manual
    # Send data (0x00) that says "I will now send a command".
    # then Send command
    def clearDisplay(self):
        self.bus.write_byte_data(self.addr, 0x00, 0x01)

    def returnHome(self):
        self.bus.write_byte_data(self.addr, 0x00, 0x02)

    def displayOn(self, cursor = False, blink = False):
        cmd = 0x0c
        if (cursor):
            cmd += 0x02
        if (blink):
            cmd += 0x01
        self.bus.write_byte_data(self.addr, 0x00, cmd)

    def displayOff(self):
        self.bus.write_byte_data(self.addr, 0x00, 0x08)

    def writeLine(self, str = '', line = 0, align = 'left', offset=0x00):
        # If string is less than 16 characters, fill with blanks
        while (len(str) < 16):
            if (align == 'right'):
                str = ' ' + str
            else:
                str = str + ' '

        # Align the cursor position line == 1 is the 2nd line
        if (line == 1):
            self.bus.write_byte_data(self.addr, 0x00, (0x80 + 0x20 + offset))
        else:
            self.bus.write_byte_data(self.addr, 0x00, (0x80 + offset))

        # Send one character at a time
        for i in range(len(str)):
            self.bus.write_byte_data(self.addr, 0x40, ord(str[i]))
            
            