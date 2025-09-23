# Sample of device communication with i2c 
# Light Sensor
# Chip: Adafruit TSL2561 Digital Luminosity/Lux Light Sensor 
#	
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
from logging import basicConfig, getLogger, DEBUG, FileHandler, Formatter
import sys
import datetime
import time
import ctypes
from enum import Enum

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

# these are the addresses that are possible
#
TSL2561_ADDR_LOW  = 0x29
TSL2561_ADDR_FLOAT = 0x39
TSL2561_ADDR_HIGH = 0x49
BUFSIZ = int(0x3f + 2)

class Tsl2561LuxScale(Enum):
    LUXSCALE      = (14)      # Scale by 2^14
    RATIOSCALE    = (9)       # Scale ratio by 2^9
    CHSCALE       = (10)      # Scale channel values by 2^10
    CHSCALE_TINT0 = (0x7517)  # 322/11 * 2^TSL2561_LUX_CHSCALE
    CHSCALE_TINT1 = (0x0FE7)  # 322/81 * 2^TSL2561_LUX_CHSCALE

class Tsl2561ControlMessage(Enum):
    REGISTER_CONTROL          = 0x00
    REGISTER_TIMING           = 0x01
    REGISTER_THRESHHOLDL_LOW  = 0x02
    REGISTER_THRESHHOLDL_HIGH = 0x03
    REGISTER_THRESHHOLDH_LOW  = 0x04
    REGISTER_THRESHHOLDH_HIGH = 0x05
    REGISTER_INTERRUPT        = 0x06
    REGISTER_CRC              = 0x08
    REGISTER_ID               = 0x0A
    REGISTER_CHAN0_LOW        = 0x0C
    REGISTER_CHAN0_HIGH       = 0x0D
    REGISTER_CHAN1_LOW        = 0x0E
    REGISTER_CHAN1_HIGH       = 0x0F

class Tsl2561CmdAction(Enum):
    READBIT       = 0x01
    COMMAND_BIT   = 0x80    # Must be 1
    CLEAR_BIT     = 0x40    # Clears any pending interrupt (write 1 to clear)
    WORD_BIT      = 0x20    # 1 = read/write word (rather than byte)
    BLOCK_BIT     = 0x10    # 1 = using block read/write

class Tsl2561IntTime(Enum):
    T13      = 0x00     # 13.7ms
    T101     = 0x01     # 101ms
    T402     = 0x02     # 402ms

class Tsl2561Gain(Enum):
    G0X      = 0x00    # No gain
    G16X     = 0x10    # 16x gain

class TSL2561:
    LOG_FILE = '{script_dir}/logs/adt7410.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, address=TSL2561_ADDR_LOW, bus_no=1, typ="CS"):
        self.init_logger()
        self.address = address
        self._bus = smbus.SMBus(bus_no)
	self.lux = 0
	self.luminosity = 0
        self.error = 0
        self.gain = 0
        self.integ = 0
        self.type = typ
        self.it_list = [13.7/100.0, 101/100.0, 402/100.0]              # timings for each integration choice in s
        if self.type == "CS":
            self.TSL2561_LUX_K1C = (0x0043)  # 0.130 * 2^RATIO_SCALE
            self.TSL2561_LUX_B1C = (0x0204)  # 0.0315 * 2^LUX_SCALE
            self.TSL2561_LUX_M1C = (0x01ad)  # 0.0262 * 2^LUX_SCALE
            self.TSL2561_LUX_K2C = (0x0085)  # 0.260 * 2^RATIO_SCALE
            self.TSL2561_LUX_B2C = (0x0228)  # 0.0337 * 2^LUX_SCALE
            self.TSL2561_LUX_M2C = (0x02c1)  # 0.0430 * 2^LUX_SCALE
            self.TSL2561_LUX_K3C = (0x00c8)  # 0.390 * 2^RATIO_SCALE
            self.TSL2561_LUX_B3C = (0x0253)  # 0.0363 * 2^LUX_SCALE
            self.TSL2561_LUX_M3C = (0x0363)  # 0.0529 * 2^LUX_SCALE
            self.TSL2561_LUX_K4C = (0x010a)  # 0.520 * 2^RATIO_SCALE
            self.TSL2561_LUX_B4C = (0x0282)  # 0.0392 * 2^LUX_SCALE
            self.TSL2561_LUX_M4C = (0x03df)  # 0.0605 * 2^LUX_SCALE
            self.TSL2561_LUX_K5C = (0x014d)  # 0.65 * 2^RATIO_SCALE
            self.TSL2561_LUX_B5C = (0x0177)  # 0.0229 * 2^LUX_SCALE
            self.TSL2561_LUX_M5C = (0x01dd)  # 0.0291 * 2^LUX_SCALE
            self.TSL2561_LUX_K6C = (0x019a)  # 0.80 * 2^RATIO_SCALE
            self.TSL2561_LUX_B6C = (0x0101)  # 0.0157 * 2^LUX_SCALE
            self.TSL2561_LUX_M6C = (0x0127)  # 0.0180 * 2^LUX_SCALE
            self.TSL2561_LUX_K7C = (0x029a)  # 1.3 * 2^RATIO_SCALE
            self.TSL2561_LUX_B7C = (0x0037)  # 0.00338 * 2^LUX_SCALE
            self.TSL2561_LUX_M7C = (0x002b)  # 0.00260 * 2^LUX_SCALE
            self.TSL2561_LUX_K8C = (0x029a)  # 1.3 * 2^RATIO_SCALE
            self.TSL2561_LUX_B8C = (0x0000)  # 0.000 * 2^LUX_SCALE
            self.TSL2561_LUX_M8C = (0x0000)  # 0.000 * 2^LUX_SCALE
       else:                                                            # T, FN and CL package values
            self.TSL2561_LUX_K1T = (0x0040)  # 0.125 * 2^RATIO_SCALE
            self.TSL2561_LUX_B1T = (0x01f2)  # 0.0304 * 2^LUX_SCALE
            self.TSL2561_LUX_M1T = (0x01be)  # 0.0272 * 2^LUX_SCALE
            self.TSL2561_LUX_K2T = (0x0080)  # 0.250 * 2^RATIO_SCALE
            self.TSL2561_LUX_B2T = (0x0214)  # 0.0325 * 2^LUX_SCALE
            self.TSL2561_LUX_M2T = (0x02d1)  # 0.0440 * 2^LUX_SCALE
            self.TSL2561_LUX_K3T = (0x00c0)  # 0.375 * 2^RATIO_SCALE
            self.TSL2561_LUX_B3T = (0x023f)  # 0.0351 * 2^LUX_SCALE
            self.TSL2561_LUX_M3T = (0x037b)  # 0.0544 * 2^LUX_SCALE
            self.TSL2561_LUX_K4T = (0x0100)  # 0.50 * 2^RATIO_SCALE
            self.TSL2561_LUX_B4T = (0x0270)  # 0.0381 * 2^LUX_SCALE
            self.TSL2561_LUX_M4T = (0x03fe)  # 0.0624 * 2^LUX_SCALE
            self.TSL2561_LUX_K5T = (0x0138)  # 0.61 * 2^RATIO_SCALE
            self.TSL2561_LUX_B5T = (0x016f)  # 0.0224 * 2^LUX_SCALE
            self.TSL2561_LUX_M5T = (0x01fc)  # 0.0310 * 2^LUX_SCALE
            self.TSL2561_LUX_K6T = (0x019a)  # 0.80 * 2^RATIO_SCALE
            self.TSL2561_LUX_B6T = (0x00d2)  # 0.0128 * 2^LUX_SCALE
            self.TSL2561_LUX_M6T = (0x00fb)  # 0.0153 * 2^LUX_SCALE
            self.TSL2561_LUX_K7T = (0x029a)  # 1.3 * 2^RATIO_SCALE
            self.TSL2561_LUX_B7T = (0x0018)  # 0.00146 * 2^LUX_SCALE
            self.TSL2561_LUX_M7T = (0x0012)  # 0.00112 * 2^LUX_SCALE
            self.TSL2561_LUX_K8T = (0x029a)  # 1.3 * 2^RATIO_SCALE
            self.TSL2561_LUX_B8T = (0x0000)  # 0.000 * 2^LUX_SCALE
            self.TSL2561_LUX_M8T = (0x0000)  # 0.000 * 2^LUX_SCALE

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

    def set_up(self, gain=Tsl2561Gain.G0X.value, integ=Tsl2561IntTime.T13.value):
        self.gain = gain
        self.integ = integ

        timing_cmd = self.gain | self.integ
        ctl_reg = Tsl2561ControlMessage.REGISTER_CONTROL.value | Tsl2561CmdAction.COMMAND_BIT.value
        tim_reg = Tsl2561ControlMessage.REGISTER_TIMING.value | Tsl2561CmdAction.COMMAND_BIT.value
        int_reg = Tsl2561ControlMessage.REGISTER_INTERRUPT.value | Tsl2561CmdAction.COMMAND_BIT.value
 
        writeReg(self.address, ctl_reg, 0x03)                                           # POWER UP
        writeReg(self.address, tim_reg, timing_cmd)                                     # No High Gain (1x), integration time of 13ms
        writeReg(self.address, int_reg, 0x00)
        time.sleep(self.it_list[int(self.integ)]+0.05)                                  # wait longer than integ time

    def power_on(self):
        ctl_reg = Tsl2561ControlMessage.REGISTER_CONTROL.value | Tsl2561CmdAction.COMMAND_BIT.value
        writeReg(self.address, ctl_reg, 0x03)                                           # POWER On

    def power_off(self):
        ctl_reg = Tsl2561ControlMessage.REGISTER_CONTROL.value | Tsl2561CmdAction.COMMAND_BIT.value
        writeReg(self.address, ctl_reg, 0x00)                                           # POWER Down


    def calculateLux(self, ch0, ch1):
        channel1 = 0
        channel0 = 0
        ratio1 = 0
        scale_list = [ Tsl2561LuxScale.CHSCALE_TINT0.value, Tsl2561LuxScale.CHSCALE_TINT1.value, 1 << Tsl2561LuxScale.CHSCALE.value]
        chScale = scale_list[int(self.integ)]
       
        # Scale for gain (1x or 16x)
        if self.gain == 0:
            chScale = chScale << 4

        # scale the channel values
        channel0 = (ch0 * chScale) >> Tsl2561LuxScale.CHSCALE.value
        channel1 = (ch1 * chScale) >> Tsl2561LuxScale.CHSCALE.value

        # find the ratio of the channel values (Channel1/Channel0)
        ratio1 = 0
        if not channel0 == 0: 
            ratio1 = (channel1 << (Tsl2561LuxScale.RATIOSCALE.value+1)) / channel0

        # round the ratio value
        ratio = (ratio1 + 1) >> 1

        b=0
        m=0

        if self.type == "CS":
            if ((ratio >= 0) and (ratio <= self.TSL2561_LUX_K1C))
                b=self.TSL2561_LUX_B1C 
                m=self.TSL2561_LUX_M1C
            elif ratio <= self.TSL2561_LUX_K2C:
                b=self.TSL2561_LUX_B2C
                m=self.TSL2561_LUX_M2C
            elif ratio <= self.TSL2561_LUX_K3C:
                b=self.TSL2561_LUX_B3C 
                m=self.TSL2561_LUX_M3C
            elif ratio <= self.TSL2561_LUX_K4C:
                b=self.TSL2561_LUX_B4C
                m=self.TSL2561_LUX_M4C
            elif ratio <= self.TSL2561_LUX_K5C:
                b=self.TSL2561_LUX_B5C
                m=self.TSL2561_LUX_M5C
            elif ratio <= self.TSL2561_LUX_K6C:
                b=self.TSL2561_LUX_B6C
                m=self.TSL2561_LUX_M6C
            elif ratio <= self.TSL2561_LUX_K7C:
                b=self.TSL2561_LUX_B7C
                m=self.TSL2561_LUX_M7C
            elif ratio > self.TSL2561_LUX_K8C:
                b=self.TSL2561_LUX_B8C
                m=self.TSL2561_LUX_M8C
        else:
            if ((ratio >= 0) and (ratio <= self.TSL2561_LUX_K1T)):
                b=self.TSL2561_LUX_B1T
                m=self.TSL2561_LUX_M1T
            elif ratio <= self.TSL2561_LUX_K2T:
                b=self.TSL2561_LUX_B2T
                m=self.TSL2561_LUX_M2T
            elif ratio <= self.TSL2561_LUX_K3T:
                b=self.TSL2561_LUX_B3T
                m=self.TSL2561_LUX_M3T
            elif ratio <= self.TSL2561_LUX_K4T:
                b=self.TSL2561_LUX_B4T
                m=self.TSL2561_LUX_M4T
            elif ratio <= self.TSL2561_LUX_K5T:
                b=self.TSL2561_LUX_B5T
                m=self.TSL2561_LUX_M5T
            elif ratio <= self.TSL2561_LUX_K6T:
                b=self.TSL2561_LUX_B6T
                m=self.TSL2561_LUX_M6T
            elif ratio <= self.TSL2561_LUX_K7T:
                b=self.TSL2561_LUX_B7T 
                m=self.TSL2561_LUX_M7T
            elif ratio > self.TSL2561_LUX_K8T:
                b=self.TSL2561_LUX_B8T 
                m=self.TSL2561_LUX_M8T

        temp = ((channel0 * b) - (channel1 * m))

        # do not allow negative lux value
        if (temp < 0):
            temp = 0

        # round lsb (2^(LUX_SCALE-1))
        temp += (1 << (Tsl2561LuxScale.LUXSCALE.value-1))

        # strip off fractional portion
        lux = temp >> Tsl2561LuxScale.LUXSCALE.value

        return lux

    def readReg(self): 
        # read write function also includes word bit to return two bytes
        ch0loreg = Tsl2561ControlMessage.REGISTER_CHAN0_LOW.value | Tsl2561CmdAction.COMMAND_BIT.value | Tsl2561CmdAction.WORD_BIT.value
        ch1loreg = Tsl2561ControlMessage.REGISTER_CHAN1_LOW.value | Tsl2561CmdAction.COMMAND_BIT.value | Tsl2561CmdAction.WORD_BIT.value
        try:
            data1ch0 = self._bus.read_i2c_block_data(self.address, ch0loreg, 2)
            data2ch1 = self._bus.read_i2c_block_data(self.address, ch1loreg, 2)
            x = data1ch0
            x <<= 16
            x |= data2ch1
            self.luminoscity = (x & 0xFFFF) - (x >> 16)
            self.lux = self.calculateLux(data1ch0, data2ch1)
            self.error = 0
        except::
            self.error = -100

    def get_lux_lumen(self):
        self.set_up()
        self.readReg()
        self.power_off()
        time.sleep(self.it_list[int(self.integ)]+0.05)                                     # wait longer than integ time
