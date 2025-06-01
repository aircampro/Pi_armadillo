# !/usr/bin python3
#
# Fuzzy logic temperature controller 
# using SparkFun CCS811/BME280 Combo Board on i2c to read data
# can connect using sparkfun HAT on raspi
# added IR control of Mitsubishi AC HVAC system 
#
import os
import smbus2 as smbus
from collections import OrderedDict
from logging import basicConfig, getLogger, DEBUG, FileHandler, Formatter
import sys
import datetime
import time
# https://github.com/r45635/HVAC-IR-Control/tree/master/python/hvac_ircontrol
from hvac_ircontrol.ir_sender import LogLevel
from hvac_ircontrol.mitsubishi import Mitsubishi, ClimateMode, FanMode, VanneVerticalMode, VanneHorizontalMode, ISeeMode, AreaMode, PowerfulMode

mitsi_ir_gpio_pin = 23                             # pin connected for IR controller to AC system
ada_fruit = False                                  # set True to use adafruit lib for bme280 only otherwise sparkfun combo CCS811/BME280
if ada_fruit == True:
    # for bme280 temp, pressure, humidity 
    #
    import board
    from adafruit_bme280 import basic as adafruit_bme280

# Temperature Sensor Module Board MADT7410-MOD
# ADT7410 (manufactured by Analog Devices)
# Chip: Analog Devices ADT7410
#	http://www.analog.com/en/products/analog-to-digital-converters/integrated-special-purpose-converters/integrated-temperature-sensors/adt7410.html
ADT7410 = False
import ctypes

# combined sparkfun CCS811/BME280 Combo Board on i2c
#
# for ccs811 co2 monitor 
#
CCS811_ADDRESS  =  0x5B
BME280_ADDRESS = 0x77                           # sometimes 0x76
ADT7410_ADDRESS = 0x48

CCS811_STATUS = 0x00
CCS811_MEAS_MODE = 0x01
CCS811_ALG_RESULT_DATA = 0x02
CCS811_HW_ID = 0x20

CCS811_DRIVE_MODE_IDLE = 0x00
CCS811_DRIVE_MODE_1SEC = 0x01
CCS811_DRIVE_MODE_10SEC = 0x02
CCS811_DRIVE_MODE_60SEC = 0x03
CCS811_DRIVE_MODE_250MS = 0x04

list_of_upd_cycles = [ 0, 1.0, 10.0, 60.0, 0.25 ]

CCS811_BOOTLOADER_APP_START = 0xF4

CCS811_HW_ID_CODE = 0x81

# for bme280 temp press humid monitor
#
t_fine = 0.0
digT = []
digP = []
digH = []

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

class ADT7410_TEMP:
    LOG_FILE = '{script_dir}/logs/adt7410.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, address=ADT7410_ADDRESS, bus_no=1):
        self.init_logger()
        self.adt7410_address = address
        self._bus = smbus.SMBus(bus_no)
	self.temperature = 0
        self.error = 0
        self.cmdbuff2 = 0
        self.cmdbuff3 = 0 

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
        self._bus.write_byte_data(self.adt7410_address, reg_address, data)

    def readReg(self):
        try:
            cmdbuff = self._bus.read_i2c_block_data(self.adt7410_address, 0x00, 12)
            tmp = (((s_u(cmdbuff[0]) << 8) | s_u(cmdbuff[1])) >> 3)
            if (tmp >= 4096):
                tmp -= 8192
            self.temperature = tmp / 16.0
            self.cmdbuff2 = cmdbuff[2]
            self.cmdbuff3 = cmdbuff[3] 
            self.error = 0
        except::
            self.error = -100

class CCS811_BME280:
    LOG_FILE = '{script_dir}/logs/ccs811_bme280.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, mode=CCS811_DRIVE_MODE_1SEC, address=CCS811_ADDRESS, i2cbme=BME280_ADDRESS, bus_no=1, extra_bus=None):
        self.init_logger()

        if mode not in [CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS]:
            raise ValueError('Unexpected mode value {0}.  Set mode to one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC or CCS811_DRIVE_MODE_250MS'.format(mode))

        self._address = address
        self.bme280_i2c_address = i2cbme
        self._bus = smbus.SMBus(bus_no)
        self.extra_bus = extra_bus
        if not self.extra_bus == None:
            self._exbus = smbus.SMBus(self.extra_bus)

        self._status = Bitfield([('ERROR' , 1), ('unused', 2), ('DATA_READY' , 1), ('APP_VALID', 1), ('unused2' , 2), ('FW_MODE' , 1)])

        self._meas_mode = Bitfield([('unused', 2), ('INT_THRESH', 1), ('INT_DATARDY', 1), ('DRIVE_MODE', 3)])

        self._error_id = Bitfield([('WRITE_REG_INVALID', 1), ('READ_REG_INVALID', 1), ('MEASMODE_INVALID', 1), ('MAX_RESISTANCE', 1), ('HEATER_FAULT', 1), ('HEATER_SUPPLY', 1)])

        self._TVOC = 0
        self._eCO2 = 0

	self.temperature = 0
	self.pressure = 0
	self.humidity = 0

        if self.readU8(CCS811_HW_ID) != CCS811_HW_ID_CODE:
            raise Exception("Device ID returned is not correct! Please check your wiring.")

        self.writeList(CCS811_BOOTLOADER_APP_START, [])
        time.sleep(0.1)

        if self.checkError():
            raise Exception("Device returned an Error! Try removing and reapplying power to the device and running the code again.")
        if not self._status.FW_MODE:
            raise Exception("Device did not enter application mode! If you got here, there may be a problem with the firmware on your sensor.")

        self.disableInterrupt()

        self.setDriveMode(mode)

    def init_logger(self):
        self._logger = getLogger(__class__.__name__)
        file_handler = FileHandler(self.LOG_FILE)
        formatter = Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self._logger.addHandler(file_handler)
        self._logger.setLevel(DEBUG)

    def disableInterrupt(self):
        self._meas_mode.INT_DATARDY = 1
        self.write8(CCS811_MEAS_MODE, self._meas_mode.get())

    def setDriveMode(self, mode):
        self._meas_mode.DRIVE_MODE = mode
        self.write8(CCS811_MEAS_MODE, self._meas_mode.get())

    def available(self):
        self._status.set(self.readU8(CCS811_STATUS))
        if not self._status.DATA_READY:
            return False
        else:
            return True

    def writeReg(self, reg_address, data):
        if not self.extra_bus == None:
            self._exbus.write_byte_data(self.bme280_i2c_address, reg_address, data)
        else:
            self._bus.write_byte_data(self.bme280_i2c_address, reg_address, data)

    def setup(self):
        osrs_t = 1			# Temperature oversampling x 1
        osrs_p = 1			# Pressure oversampling x 1
        osrs_h = 1			# Humidity oversampling x 1
        mode   = 3			# Normal mode
        t_sb   = 5			# Tstandby 1000ms
        filter = 0			# Filter off
        spi3w_en = 0			# 3-wire SPI Disable

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        self.writeReg(0xF2, ctrl_hum_reg)
        self.writeReg(0xF4, ctrl_meas_reg)
        self.writeReg(0xF5, config_reg)

    def get_calib_param(self):
        global digT, digP, digH
        calib = []

        for i in range (0x88,0x88+24):
            if not self.extra_bus == None:
                calib.append(self._exbus.read_byte_data(self.bme280_i2c_address,i))
            else:
                calib.append(self._bus.read_byte_data(self.bme280_i2c_address,i))
        if not self.extra_bus == None:
            calib.append(self._exbus.read_byte_data(self.bme280_i2c_address,0xA1))
        else:
            calib.append(self._bus.read_byte_data(self.bme280_i2c_address,0xA1))
        for i in range (0xE1,0xE1+7):
            if not self.extra_bus == None:
                calib.append(self._exbus.read_byte_data(self.bme280_i2c_address,i))
            else:
                calib.append(self._bus.read_byte_data(self.bme280_i2c_address,i))

        digT.append((calib[1] << 8) | calib[0])
        digT.append((calib[3] << 8) | calib[2])
        digT.append((calib[5] << 8) | calib[4])
        digP.append((calib[7] << 8) | calib[6])
        digP.append((calib[9] << 8) | calib[8])
        digP.append((calib[11]<< 8) | calib[10])
        digP.append((calib[13]<< 8) | calib[12])
        digP.append((calib[15]<< 8) | calib[14])
        digP.append((calib[17]<< 8) | calib[16])
        digP.append((calib[19]<< 8) | calib[18])
        digP.append((calib[21]<< 8) | calib[20])
        digP.append((calib[23]<< 8) | calib[22])
        digH.append( calib[24] )
        digH.append((calib[26]<< 8) | calib[25])
        digH.append( calib[27] )
        digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        digH.append( calib[31] )
	
        for i in range(1,2):
            if digT[i] & 0x8000:
                digT[i] = (-digT[i] ^ 0xFFFF) + 1

        for i in range(1,8):
            if digP[i] & 0x8000:
                digP[i] = (-digP[i] ^ 0xFFFF) + 1

        for i in range(0,6):
            if digH[i] & 0x8000:
                digH[i] = (-digH[i] ^ 0xFFFF) + 1  

    def compensate_P(self, adc_P):
        global  t_fine
        pressure = 0.0
	
        v1 = (t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
        v2 = v2 + ((v1 * digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (digP[3] * 65536.0)
        v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * digP[0]) / 32768
	
        if v1 == 0:
            return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  

        #print "pressure : %7.2f hPa" % (pressure/100)
        return "{:.2f}".format(pressure/100)

    def compensate_T(self, adc_T):
        global t_fine
        v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
        v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
        t_fine = v1 + v2
        temperature = t_fine / 5120.0
        #print "temp : %-6.2f ℃" % (temperature) 
        return "{:.2f}".format(temperature)

    def compensate_H(self, adc_H):
        global t_fine
        var_h = t_fine - 76800.0
        if var_h != 0:
            var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
        else:
            return 0
        var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        #print "hum : %6.2f ％" % (var_h)
        return "{:.2f}".format(var_h)

    def readBME(self):
        try:
            data = []
            for i in range (0xF7, 0xF7+8):
                if not self.extra_bus == None:
                    data.append(self._exbus.read_byte_data(self.bme280_i2c_address, i))
                else:
                    data.append(self._bus.read_byte_data(self.bme280_i2c_address, i))
	    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
	    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
	    hum_raw  = (data[6] << 8)  |  data[7]
	
	    self.temperature = self.compensate_T(temp_raw)
	    self.pressure = self.compensate_P(pres_raw)
	    self.humidity = self.compensate_H(hum_raw)
            error = 0
        except::
            error = -100
        return dict(temperature=self.temperature, pressure=self.pressure, humidity=self.humidity, error=error)

    def readData(self):
        if not self.available():
            return False
        else:
            buf = self.readList(CCS811_ALG_RESULT_DATA, 8)
            self._eCO2 = (buf[0] << 8) | (buf[1])
            self._TVOC = (buf[2] << 8) | (buf[3])
            if self._status.ERROR:
                return buf[5]
            else:
                return 0

    def getTVOC(self):
        return self._TVOC

    def geteCO2(self):
        return self._eCO2

    def checkError(self):
        self._status.set(self.readU8(CCS811_STATUS))
        return self._status.ERROR

    def readU8(self, register):
        result = self._bus.read_byte_data(self._address, register) & 0xFF
        self._logger.debug("Read 0x%02X from register 0x%02X", result, register)
        return result

    def write8(self, register, value):
        value = value & 0xFF
        self._bus.write_byte_data(self._address, register, value)
        self._logger.debug("Wrote 0x%02X to register 0x%02X", value, register)

    def readList(self, register, length):
        results = self._bus.read_i2c_block_data(self._address, register, length)
        self._logger.debug("Read the following from register 0x%02X: %s", register, results)
        return results

    def writeList(self, register, data):
        self._bus.write_i2c_block_data(self._address, register, data)
        self._logger.debug("Wrote to register 0x%02X: %s", register, data)

class Bitfield:
    def __init__(self, _structure):
        self._structure = OrderedDict(_structure)
        for key, value in self._structure.items():
            setattr(self, key, 0)

    def get(self):
        fullreg = 0
        pos = 0
        for key, value in self._structure.items():
            fullreg = fullreg | ( (getattr(self, key) & (2**value - 1)) << pos )
            pos = pos + value

        return fullreg

    def set(self, data):
        pos = 0
        for key, value in self._structure.items():
            setattr(self, key, (data >> pos) & (2**value - 1))
            pos = pos + value

# co2 monitor using the CCS811
#            
class AirConditionMonitor:
    CO2_PPM_THRESHOLD_1 = 1000
    CO2_PPM_THRESHOLD_2 = 2000

    CO2_LOWER_LIMIT  =  400
    CO2_HIGHER_LIMIT = 8192

    CO2_STATUS_CONDITIONING = 'CONDITIONING'
    CO2_STATUS_LOW          = 'LOW'
    CO2_STATUS_HIGH         = 'HIGH'
    CO2_STATUS_TOO_HIGH     = 'TOO HIGH'
    CO2_STATUS_ERROR        = 'ERROR'

    LOG_FILE = '{script_dir}/logs/air_condition_monitor.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, md=CCS811_DRIVE_MODE_1SEC, a=CCS811_ADDRESS, b=BME280_ADDRESS, bn=1, eb=None):               
        self._ccs811 = CCS811_BME280(mode=md, address=a, i2cbme=b, bus_no=bn, extra_bus=eb)
        self.co2_status = self.CO2_STATUS_LOW
        self.init_logger()
        self.co2 = 0
        self.TVOC = 0
        self.temperature = 0
        self.humidity = 0
        self.pressure = 0
        self.update = 0
        self._ccs811.setup()                                       # bme280 set-up
        self._ccs811.get_calib_param()  
      
    def init_logger(self):
        self._logger = getLogger(__class__.__name__)
        file_handler = FileHandler(self.LOG_FILE)
        formatter = Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self._logger.addHandler(file_handler)
        self._logger.setLevel(DEBUG)

    def status(self, co2=self.co2):
        if co2 < self.CO2_LOWER_LIMIT or co2 > self.CO2_HIGHER_LIMIT:
            return self.CO2_STATUS_CONDITIONING
        elif co2 < self.CO2_PPM_THRESHOLD_1:
            return self.CO2_STATUS_LOW
        elif co2 < self.CO2_PPM_THRESHOLD_2:
            return self.CO2_STATUS_HIGH
        else:
            return self.CO2_STATUS_TOO_HIGH

    def execute(self):
        while not self._ccs811.available():
            pass

        while True:
            if not self._ccs811.available():
                time.sleep(1)
                continue

            try:
                if not self._ccs811.readData():
                    co2 = self._ccs811.geteCO2()
                    co2_status = self.status(co2)
                    if co2_status == self.CO2_STATUS_CONDITIONING:
                        self._logger.debug("Under Conditioning...")
                        time.sleep(2)
                        continue

                    if co2_status != self.co2_status:
                        self.co2_status = co2_status

                    self._logger.info("CO2: {0}ppm, TVOC: {1}".format(co2, self._ccs811.getTVOC()))
                else:
                    self._logger.error('ERROR!')
                    while True:
                        pass
            except:
                self._logger.error(sys.exc_info())

            time.sleep(2)

    def get_sparkfun_combo_dat(self, oneShot=True):
        while not self._ccs811.available():
            pass

        run_loop = True
        while run_loop:
            self.update = 0
            if not self._ccs811.available():
                time.sleep(1)
                continue

            try:
                if not self._ccs811.readData():
                    co2_status = self.status(co2)
                    if co2_status == self.CO2_STATUS_CONDITIONING:
                        self._logger.debug("Under Conditioning...")
                        time.sleep(2)
                        continue

                    if co2_status != self.co2_status:
                        self.co2_status = co2_status

                    self.co2 = self._ccs811.geteCO2()
                    self.TVOC = self._ccs811.getTVOC()
                    bme280_dat = self._ccs811.readBME()
                    if bme280_dat['error'] == 0:
                        self.temperature = bme280_dat['temperature']
                        self.pressure = bme280_dat['pressure']
                        self.humidity = bme280_dat['humidity']
                    else:
                        self.update = -300                     
                    if oneShot == True:
                        run_loop = False
                    self.update = 1
                else:
                    self.update = -200
            except:
                self._logger.error(sys.exc_info())
                self.update = -100

# fuzzy logic controller with temperature zones
#
def fuzzy_controller():
	
    #array which contains the room temperature
    temperature = np.arange(15,45)

    #set the input
    temp = ctrl.Antecedent(temperature, ‘temp’)

    #array to represent the possible AC temperatures
    #based on the defined temperature range
    ac_temperature = np.arange(15,31)

    #set the output of the system
    ac_temp = ctrl.Consequent(ac_temperature, ‘ac_temp’)

    #automatically generate the membership functions
    temp.automf(3)
    ac_temp.automf(3)

    #graphical representation
    temp.view()
    ac_temp.view()

    #setting rules
    rule1 = ctrl.Rule(temp[‘poor’], ac_temp[‘good’])
    rule2 = ctrl.Rule(temp[‘average’], ac_temp[‘average’])
    rule3 = ctrl.Rule(temp[‘good’], ac_temp[‘poor’])

    #Creating control system
    temperature_ctrl = ctrl.ControlSystem([rule1,rule2,rule3])
    detect_temp = ctrl.ControlSystemSimulation(temperature_ctrl)

    # testing with various temperature inputs
    detect_temp.input[‘temp’] = 45
    detect_temp.compute()
    temp = detect_temp.output[‘ac_temp’]
    print(“\nWhen room temperature is 45°C :”)
    print(“The AC temperature is adjusted to”,temp, “°C”)
    ac_temp.view(sim = detect_temp)

    detect_temp.input[‘temp’] = 27
    detect_temp.compute()
    temp = detect_temp.output[‘ac_temp’]
    print(“\nWhen room temperature is 27°C :”)
    print(“The AC temperature is adjusted to”,temp, “°C”)

    detect_temp.input[‘temp’] = 15
    detect_temp.compute()
    temp = detect_temp.output[‘ac_temp’]
    print(“\nWhen room temperature is 15°C :”)
    print(“The AC temperature is adjusted to”,temp, “°C”)
    ac_temp.view(sim = detect_temp)
    
    return detect_temp

def c_to_f(temp)
    fahrenheit=temp*9/5+32;
    return fahrenheit

def get_hour():
    return int(str(datetime.datetime.now()).split(" ")[1].split(":")[0])
             
def run():
    if ada_fruit == True:
        i2c = board.I2C()
        bme280_2 = adafruit_bme280.Adafruit_BME280_I2C(i2c,0x77)
        print("bme200 sensor:",int(bme280_2.temperature),"℃  ",int(bme280_2.relative_humidity),"%  ", int(bme280_2.pressure),"hPa")
        air_condition_monitor = AirConditionMonitor()                                   # for logger only
        LOOP_CYCLE = 1.0
    elif ADT7410 == True:
        t_sensor = ADT7410_TEMP()
        t_sensor.readReg()
        print("adt7410 : ",t_sensor.temperature,"℃  ")
    else:	
        chosen_update_rate = CCS811_DRIVE_MODE_1SEC
        air_condition_monitor = AirConditionMonitor(chosen_update_rate)
        air_condition_monitor.get_sparkfun_combo_dat()
        if air_condition_monitor.update == 1:
            print("ccs811 sensor:",int(air_condition_monitor.co2)," ",int(air_condition_monitor.TVOC))      
            print("bme200 sensor:",int(air_condition_monitor.temperature),"℃  ",int(air_condition_monitor.humidity),"%  ", int(air_condition_monitor.pressure),"hPa")  
        LOOP_CYCLE = 1.0 * list_of_upd_cycles[int(str(chosen_update_rate),16)]

    fuzzy_model = fuzzy_controller()
    day = 0
    HVAC = Mitsubishi(mitsi_ir_gpio_pin, LogLevel.ErrorsOnly)            # define object for IR communication
       
    while True:
        if ada_fruit == False:                                           # we are using the sparkfun combo
            air_condition_monitor.get_sparkfun_combo_dat()
            if air_condition_monitor.update == 1:	
                #print("ccs811 sensor: CO2= ",int( air_condition_monitor.co2)," TVOC= ",int( air_condition_monitor.TVOC )) 
                air_condition_monitor._logger.info("CO2: {0}ppm, TVOC: {1}".format(air_condition_monitor.co2, air_condition_monitor.TVOC))     
                air_condition_monitor._logger.info("T: {0}℃, H: {1} P:{2}".format(air_condition_monitor.temperature, air_condition_monitor.humidity, air_condition_monitor.pressure))
                fuzzy_input_temp = air_condition_monitor.temperature
            else:
                fuzzy_input_temp = -100
        elif ADT7410 == True:
            t_sensor.readReg()
            t_sensor._logger.info(("adt7410 : ",t_sensor.temperature,"℃  ")
            fuzzy_input_temp = t_sensor.temperature
        else:
            #print("bme200 sensor: T= ",int(bme280_2.temperature),"℃  H= ",int( bme280_2.relative_humidity),"%  P= ", int(bme280_2.pressure),"hPa")
            air_condition_monitor._logger.info("T: {0}℃, H: {1} P:{2}".format(bme280_2.temperature, bme280_2.relative_humidity, bme280_2.pressure)) 
            fuzzy_input_temp = bme280_2.temperature

        if fuzzy_input_temp >= -90:        
            fuzzy_model.input[‘temp’] = fuzzy_input_temp
            fuzzy_model.compute()
            temp = fuzzy_model.output[‘ac_temp’]	
            #print(" fuzzy model predicts temp set-point of ", temp, "℃  ", c_to_f(temp), "F")
            air_condition_monitor._logger.info("fuzzy model predicts temp set-point of {0}℃ , {1}F".format(temp, c_to_f(temp)))   

        co2_state = air_condition_monitor.status()
        #print("co2 status = ",co2_state)
        air_condition_monitor._logger.info("CO2 status : {0}".format(co2_state))

        hr=get_hour() 
        if hr >= 20 and hr <= 6:
            #print("night time")
            if not day == 1:
                air_condition_monitor._logger.info(" ----- night time -----")
                day = 1
        else:
            #print("day time") 
            if not day == 2:
                air_condition_monitor._logger.info(" ----- day time ------")
                day = 2

        HVAC.send_command(
            climate_mode=ClimateMode.Cold,
            temperature=temp,
            fan_mode=FanMode.Auto,
            vanne_vertical_mode=VanneVerticalMode.Auto,
            vanne_horizontal_mode=VanneHorizontalMode.Swing,
            isee_mode=ISeeMode.ISeeOn,
            area_mode=AreaMode.Full,
            powerful=PowerfulMode.PowerfulOn
            )
        time.sleep(LOOP_CYCLE)

    HVAC.power_off()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        pass
