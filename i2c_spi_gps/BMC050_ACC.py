# Sample of device communication with i2c 
# Accel Sensor
# Chip: Bosch BMC050 
#	https://www.bosch-sensortec.com/en/homepage/products_3/6_axis_sensors_2/ecompass/bmc050_1/bmc050
#
import os
#
# sudo apt-get install python-smbus
#
import sys
# python 3.6 > supports smbus2 now for i2c communication
if sys.version_info[0] >= 3 and sys.version_info[1] >= 6:
    import smbus2 as smbus
else:
    import smbus
from collections import OrderedDict
from logging import basicConfig, getLogger, DEBUG, FileHandler, Formatter
import sys
import datetime
import time
import ctypes

# convert bytes/words between signed and unsigned
#
def s_u8(byte, mode="U8"):
    if mode == "S8":
        return ctypes.c_int8(byte).value
    elif mode == "U8":
        return ctypes.c_uint8(byte).value

def s_u16(byte, mode="U16"):
    if mode == "S16":
        return ctypes.c_int16(byte).value
    elif mode == "U16":
        return ctypes.c_uint16(byte).value

def s_u32(byte, mode="U32"):
    if mode == "S32":
        return ctypes.c_int32(byte).value
    elif mode == "U32":
        return ctypes.c_uint32(byte).value

BMC050_ADDRESS = 0x18
BUFSIZ = int(0x3f + 2)

class BMC050:
    LOG_FILE = '{script_dir}/logs/adt7410.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, address=BMC050_ADDRESS, bus_no=1):
        self.init_logger()
        self.bmc050_address = address
        self._bus = smbus.SMBus(bus_no)
	    self.temperature = 0
        self.error = 0
        self.accelx = 0
        self.accely = 0
        self.accelz = 0 
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0 
        self.ACCEL_RES = 3.91 * 0.001
        self.g = 9.81

    def init_logger(self):
        self._logger = getLogger(__class__.__name__)
        file_handler = FileHandler(self.LOG_FILE)
        formatter = Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self._logger.addHandler(file_handler)
        self._logger.setLevel(DEBUG)

    def writeReg(self, reg_address=0, data=None):
        if data==None:
            data=(ctypes.c_uint8 * 1)()        
        self._bus.write_byte_data(self.bmc050_addresss, reg_address, data)

    def accel(self, buff, idx): 
        # C code has this (((int16_t)  ((*(buff + 1) << 8) | *(buff))    ) >> 6)
        # translates to this (s_u16(((buff[idx+1] << 8) | buff[idx]),"S16") >> 6)
        #
        # but this seemed more right for python (bit shifts on uints but sign is taken from before >> 6
        if s_u16(((buff[idx+1] << 8) | buff[idx]), "S16") < 0:
            sign = -1
        else:
            sign = 1
        return sign * (((buff[idx+1] << 8) | buff[idx]) >> 6)

    def readReg(self, bsiz=BUFSIZ):
        try:
            cmdbuff = self._bus.read_i2c_block_data(self.bmc050_address, 0x00, bsiz)
            self.accelx = self.accel(cmdbuff, 2)
            self.accely = self.accel(cmdbuff, 4)
            self.accelz = self.accel(cmdbuff, 6) 
            self.accel_x = self.accelx * self.g * self.ACCEL_RES
            self.accel_y = self.accely * self.g * self.ACCEL_RES
            self.accel_z = self.accelz * self.g * self.ACCEL_RES 
            self.temperature = 24.0 + (cmdbuff[8] * 0.5)
            self.error = 0
        except::
            self.error = -100
