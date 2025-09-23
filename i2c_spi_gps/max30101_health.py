#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# i2c interface to Maxim Integrated Products, Inc. MAX30101
# High-Sensitivity Pulse Oximeter and Heart-Rate Sensor for Wearable Health
# https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30101.pdf
#

# convert bytes/words between signed and unsigned
#
import ctypes
import math
#
# sudo apt-get install python-smbus
#
import sys
# python 3.6 > supports smbus2 now for i2c communication
if sys.version_info[0] >= 3 and sys.version_info[1] >= 6:
    import smbus2 as smbus
else:
    import smbus

# convert types
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

MAX30101_ADDRESS = 0b1010111                                                                     # default address should be 0x57

class INT_STAT_1:
    RA = 0x0
    A_FULL = int(math.pow(2,0))
    PPG_RDY = int(math.pow(2,5))
    ALC_OVF = int(math.pow(2,6))
    PWR_RDY = int(math.pow(2,7))
    PPG_RDY_SH = 5
    ALC_OVF_SH = 6
    PWR_RDY_SH = 7

class INT_STAT_2:
    RA = 0x01
    DIE_TEMP_RDY = int(math.pow(2,1))
    DIE_TEMP_RDY_SH = 1

class INT_ENAB_1:
    RA = 0x02
    A_FULL = int(math.pow(2,0))
    PPG_RDY = int(math.pow(2,5))
    ALC_OVF = int(math.pow(2,6))
    PWR_RDY = int(math.pow(2,7))
    PPG_RDY_SH = 5
    ALC_OVF_SH = 6
    PWR_RDY_SH = 7

class INT_ENAB_2:
    RA = 0x01
    DIE_TEMP_RDY = int(math.pow(2,1))
    DIE_TEMP_RDY_SH = 1

class FIFO:
    FIFO_WR_PTRA = 0x04
    FIFO_WR_LEN = 0x1F
    OVF_COUNTERA = 0x05
    OVF_COUNTER_LEN = 0x1F
    FIFO_RD_PTRA = 0x06
    FIFO_RD_LEN = 0x1F
    FIFO_DTA_PTRA = 0x07
    FIFO_DTA_LEN = 0xFF

class DIE_TEMP:
    DT_A = 0x1F
    DTF_A = 0x20
    DTF_LEN = 0b00001111
    DTC_A = 0x21
    DTC_LEN = 0b00000001

class CONFIG:
    FIFO_RA = 0x08
    FIFO_A_FULL_LEN = int(math.pow(2,0)) + int(math.pow(2,1)) + int(math.pow(2,2)) + int(math.pow(2,3))
    FIFO_ROLLOVER_EN = 0x01 << 4
    FIFO_ROLLOVER_SH = 4
    SMP_AVE = 0x07 << 5
    SMP_AVE_SH = 5
    MODE_RA = 0x09
    MODE = int(math.pow(2,0)) | int(math.pow(2,1)) | int(math.pow(2,2))  
    RESET = 0x01 << 6
    RESET_SH = 4
    SHDN = 0x01 << 7
    SHDN_SH = 7
    SpO2_RA = 0x0A
    LED_PW = int(math.pow(2,0)) | int(math.pow(2,1))  
    SpO2_SR = int(math.pow(2,0)) | int(math.pow(2,1)) | int(math.pow(2,2)) << 2 
    SpO2_SR_SH = 2
    SpO2_ADC_RGE = 0x03 << 5
    SHDN_SH = 5
    LED1_PA = 0x0C
    LED2_PA = 0x0D
    LED1_PA = 0x0E
    LED2_PA = 0x0F
    MLMCRA1 = 0x11
    MLMCRA1_S1 = 0x07   
    MLMCRA1_S2 = 0x07 << 4 
    MLMCRA1_S2_SH = 4
    MLMCRA2 = 0x12
    MLMCRA2_S1 = 0x07   
    MLMCRA2_S2 = 0x07 << 4 
    MLMCRA2_S2_SH = 4    

class SMP_Average:
    NO_AVG = 0b000 
    A2 = 0b001
    A4 = 0b010
    A8 = 0b011 
    A16 - 0b100 
    A32_1 = 0b101 
    A32_2 = 0b110 
    A32_3 = 0b111 

class Mode:
    HeartRate = 0b010  # mode Red only
    SpO2 = 0b011       # mode Red and IR
    MultiLED = 0b111   # mode Green, Red, and/or IR

# specify address (set to default) and bus number for i2c connection to mcu
class MAX30101:
    LOG_FILE = '{script_dir}/logs/max30101.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, address=MAX30101_ADDRESS, bus_no=1):
        self.init_logger()
        self.max30101_address = address
        self._bus = smbus.SMBus(bus_no)
	    self.pwr_rdy = 0
        self.ppg_rdy = 0
        self.alc_ovf = 0
        self.a_full = 0 
        self.dt_rdy = 0
        self.dt_value = 0
        self.dt_prev = 0
        self.fifo_buf = []
        self.prev_fifo_buf = []
        self.led1 = 0
        self.led2 = 0
        self.led3 = 0 
        self.pled1 = 0
        self.pled2 = 0
        self.pled3 = 0        
        self.error = 0
        self.mode = 0

    def init_logger(self):
        self._logger = getLogger(__class__.__name__)
        file_handler = FileHandler(self.LOG_FILE)
        formatter = Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self._logger.addHandler(file_handler)
        self._logger.setLevel(DEBUG)

    # function which reads interrupt 1 register
    def readInterupt1(self, bsiz=1):
        try:
            cmdbuff = self._bus.read_i2c_block_data(self.max30101_address, INT_STAT_1.RA, 1)
            try:
                self.pwr_rdy = (cmdbuff & INT_STAT_1.PWR_RDY) >> INT_STAT_1.PWR_RDY_SH
                self.ppg_rdy = (cmdbuff & INT_STAT_1.PPG_RDY) >> INT_STAT_1.PPG_RDY_SH
                self.alc_ovf = (cmdbuff & INT_STAT_1.ALC_OVF) >> INT_STAT_1.ALC_OVF_SH
                self.a_full = (cmdbuff & INT_STAT_1.A_FULL) 
            except:
                self.pwr_rdy = (cmdbuff[0] & INT_STAT_1.PWR_RDY) >> INT_STAT_1.PWR_RDY_SH 
                self.ppg_rdy = (cmdbuff[0] & INT_STAT_1.PPG_RDY) >> INT_STAT_1.PPG_RDY_SH
                self.alc_ovf = (cmdbuff[0] & INT_STAT_1.ALC_OVF) >> INT_STAT_1.ALC_OVF_SH
                self.a_full = (cmdbuff[0] & INT_STAT_1.A_FULL)                
        except::
            self.error = -100

    def writeReg(self, reg_address=0, data=None):
        if data==None:
            data=(ctypes.c_uint8 * 1)()        
        self._bus.write_byte_data(self.max30101_addresss, reg_address, data)

    # function which reads interrupt 2 register
    def readInterupt2(self, bsiz=1):
        try:
            cmdbuff = self._bus.read_i2c_block_data(self.max30101_address, INT_STAT_2.RA, 1)
            try:
                self.dt_rdy = (cmdbuff & INT_STAT_2.DIE_TEMP_RDY) >> INT_STAT_2.DIE_TEMP_RDY_SH
            except:
                self.dt_rdy = (cmdbuff[0] & INT_STAT_2.DIE_TEMP_RDY) >> INT_STAT_2.DIE_TEMP_RDY_SH                
        except::
            self.error = -100

    def readData(self, bsiz=1):
        try:
            if self.dt_rdy:
                # ----------- read the die temperature ------------
                cmdbuff = self._bus.read_i2c_block_data(self.max30101_address, DIE_TEMP.DT_A, 2)    # read 2 bytes 
                self.dt_prev = self.dt_value
                self.dt_value = s_u8((cmdbuff[0] + (cmdbuff[1] & DIE_TEMP.DTF_LEN)), mode="S8")     # msp is sign bit type cast to s8 bit:    
            elif self.ppg_rdy and ((self.mode == Mode.HeartRate) or (self.mode == Mode.SpO2)):      # new data and we are in spO2 or HR mode
                # ----------- read the fifo buffer ------------
                self.new_data = True
                self.prev_fifo_buf = self.fifo_buf
                self.fifo_buf = []
                writeReg(reg_address=FIFO.FIFO_WR_PTRA, data=0x0)
                cmdbuff = self._bus.read_i2c_block_data(self.max30101_address, FIFO.FIFO_WR_PTRA, 3)    # read 3 bytes from the FIFO_WR_PTRA address
                fifo_wr_ptr = cmdbuff[0] & FIFO.FIFO_WR_LEN  
                fifo_rd_ptr = cmdbuff[2] & FIFO.FIFO_RD_LEN   
                no_available = fifo_wr_ptr - fifo_rd_ptr                                                # calculate number of buffers available
                while (no_available > 0):                                                               # read the available buffers
                    writeReg(reg_address=FIFO.FIFO_DTA_PTRA, data=0x0)  
                    cmdbuff = self._bus.read_i2c_block_data(self.max30101_address, FIFO.FIFO_WR_PTRA, 1) 
                    self.fifo_buf.append(cmdbuff)
                    no_available -= 1 
                    if len(self.fifo_buf) == 3:
                        self.led1 = (self.fifo_buf[0] << 16) | (self.fifo_buf[0] << 8) | self.fifo_buf[2] 
                        self.pled1 = self.led1 
                    elif len(self.fifo_buf) == 6:
                        self.led2 = (self.fifo_buf[3] << 16) | (self.fifo_buf[4] << 8) | self.fifo_buf[5] 
                        self.pled2 = self.led2 
                    elif len(self.fifo_buf) == 9:
                        self.led3 = (self.fifo_buf[6] << 16) | (self.fifo_buf[7] << 8) | self.fifo_buf[8] 
                        self.pled3 = self.led3                         
                        self.prev_fifo_buf = self.fifo_buf
                        self.fifo_buf = [] 
                writeReg(reg_address=FIFO.FIFO_RD_PTRA, data=0x0)                        
                self.new_data = False                    
        except::
            self.error = -100

    # initialize the unit 
    def i2c_init(self, mde=Mode.HeartRate):
        self.mode = mde
        self.bus.write_byte_data(self.max30101_addresss, INT_ENAB_1.RA, 0x00)           # enable all interrupt 1
        time.sleep(0.1)
        self.bus.write_byte_data(self.max30101_addresss, INT_ENAB_2.RA, 0x00)           # enable all interrupt 2
        time.sleep(0.1)
        smp_avg = (SMP_Average.A2 << CONFIG.SMP_AVE_SH) & CONFIG.SMP_AVE                # configure average 2 samples 
        fifo_a_full = 0x12                                                              # set interrupt when 18 samples empty 14 unread
        self.bus.write_byte_data(self.max30101_addresss, CONFIG.FIFO_RA, (fifo_a_full | smp_avg))
        time.sleep(0.1)
        mode = mde                                                                      # choose the device mode
        self.bus.write_byte_data(self.max30101_addresss, CONFIG.MODE_RA, mode)
        time.sleep(0.1)

TEST_ON=False                                                                           # enable test if True when loading the library
if __name__ == '__main__':
    if TEST_ON == True:
        sensor = MAX30101()                                                             # make device object
        sensor.i2c_init()                                                               # configure device
        for rep in range(0,100):                                                        # test for 100 cycles
            sensor.readInterupt1()                                                      # read interrupt
            sensor.readData()                                                           # check and process data
            sensor.readInterupt2()  
            sensor.readData()     
            print(f" led1 : {self.led1} led2 : {self.led2} led3 : {self.led3}")
            print(f" die temp : {self.dt_value}")            