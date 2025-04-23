# Sample of device communication with i2c 
# 
# Distance sensor
# Chip: SHARP GP2Y0E03
#	https://jp.sharp/products/device/lineup/selection/opto/haca/diagram2.html
import os
import smbus2 as smbus
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

GP2Y0E03_ADDRESS = 0x40
BUFSIZ = 256

class GP2Y0E03:
    LOG_FILE = '{script_dir}/logs/adt7410.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, address=GP2Y0E03_ADDRESS, bus_no=1):
        self.init_logger()
        self.address = address
        self._bus = smbus.SMBus(bus_no)
	self.temperature = 0
        self.error = 0
        self.distance = 0
        self.return = 0

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
        self._bus.write_byte_data(self.addresss, reg_address, data)

    def readReg(self, bsiz=BUFSIZ):
        try:
            cmdbuff = self._bus.read_i2c_block_data(self.address, 0x00, bsiz)
            if (cmdbuff[int(0x35)] == 0x01 or cmdbuff[int(0x35)] == 0x02):
                bitshift = 1 << cmdbuff[int(0x35)]
                self.distance = (((cmdbuff[int(0x5e)] << 4) + cmdbuff[int(0x5f)]) / 16.0) / bitshift
            else:
                self.return = cmdbuff[0]
            self.error = 0
        except::
            self.error = -100