# !/usr/bin python3
# fuzzy logic temperature controller 
# using SparkFun CCS811/BME280 Combo Board on i2c to read data
#
import os
import smbus2 as smbus
from collections import OrderedDict
from logging import basicConfig, getLogger, DEBUG, FileHandler, Formatter
import sys
import datetime

# for bme280 temp, pressure, humidity 
#
import time
import board
from adafruit_bme280 import basic as adafruit_bme280

# for ccs811 co2 monitor 
#
CCS811_ADDRESS  =  0x5B

CCS811_STATUS = 0x00
CCS811_MEAS_MODE = 0x01
CCS811_ALG_RESULT_DATA = 0x02
CCS811_HW_ID = 0x20

CCS811_DRIVE_MODE_IDLE = 0x00
CCS811_DRIVE_MODE_1SEC = 0x01
CCS811_DRIVE_MODE_10SEC = 0x02
CCS811_DRIVE_MODE_60SEC = 0x03
CCS811_DRIVE_MODE_250MS = 0x04

CCS811_BOOTLOADER_APP_START = 0xF4

CCS811_HW_ID_CODE = 0x81

class CCS811:
    LOG_FILE = '{script_dir}/logs/ccs811.log'.format(
        script_dir = os.path.dirname(os.path.abspath(__file__))
    )

    def __init__(self, mode = CCS811_DRIVE_MODE_1SEC, address = CCS811_ADDRESS):
        self.init_logger()

        if mode not in [CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS]:
            raise ValueError('Unexpected mode value {0}.  Set mode to one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC or CCS811_DRIVE_MODE_250MS'.format(mode))

        self._address = address
        self._bus = smbus.SMBus(1)

        self._status = Bitfield([('ERROR' , 1), ('unused', 2), ('DATA_READY' , 1), ('APP_VALID', 1), ('unused2' , 2), ('FW_MODE' , 1)])

        self._meas_mode = Bitfield([('unused', 2), ('INT_THRESH', 1), ('INT_DATARDY', 1), ('DRIVE_MODE', 3)])

        self._error_id = Bitfield([('WRITE_REG_INVALID', 1), ('READ_REG_INVALID', 1), ('MEASMODE_INVALID', 1), ('MAX_RESISTANCE', 1), ('HEATER_FAULT', 1), ('HEATER_SUPPLY', 1)])

        self._TVOC = 0
        self._eCO2 = 0

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

    def __init__(self):
        self._ccs811 = CCS811()
        self.co2_status = self.CO2_STATUS_LOW
        self.init_logger()
        self.co2 = 0
        self.TVOC = 0
        
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

    def get_co2_dat(self, oneShot=True):
        while not self._ccs811.available():
            pass

        run_loop = True
        while run_loop:
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
                    if oneShot == True:
                        run_loop = False
            except:
                self._logger.error(sys.exc_info())

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
                
if __name__ == '__main__':
    i2c = board.I2C()
    bme280_2 = adafruit_bme280.Adafruit_BME280_I2C(i2c,0x77)
	
    air_condition_monitor = AirConditionMonitor()
    air_condition_monitor.get_co2_dat()
    print("ccs811 sensor:",int(air_condition_monitor.co2)," ",int(air_condition_monitor.TVOC))      
    print("bme200 sensor:",int(bme280_2.temperature),"℃  ",int(bme280_2.relative_humidity),"%  ", int(bme280_2.pressure),"hPa")   

    fuzzy_model = fuzzy_controller()
    day = 0    
    LOOP_CYCLE = 2.0
    while True:
        air_condition_monitor.get_co2_dat()	
        #print("ccs811 sensor: CO2= ",int( air_condition_monitor.co2)," TVOC= ",int( air_condition_monitor.TVOC )) 
        self._logger.info("CO2: {0}ppm, TVOC: {1}".format(air_condition_monitor.co2, air_condition_monitor.TVOC))     
        #print("bme200 sensor: T= ",int(bme280_2.temperature),"℃  H= ",int( bme280_2.relative_humidity),"%  P= ", int(bme280_2.pressure),"hPa") 
        self._logger.info("T: {0}℃, H: {1} P:{2}".format(bme280_2.temperature, bme280_2.relative_humidity, bme280_2.pressure)) 
        
        fuzzy_model.input[‘temp’] = bme280_2.temperature
        fuzzy_model.compute()
        temp = fuzzy_model.output[‘ac_temp’]	
        #print(" fuzzy model predicts temp set-point of ", temp, "℃  ", c_to_f(temp), "F")
        self._logger.info("fuzzy model predicts temp set-point of {0}℃ , {1}F".format(temp, c_to_f(temp)))   

        co2_state = air_condition_monitor.status()
        #print("co2 status = ",co2_state)
        self._logger.info("CO2 status : {0}".format(co2_state))

        hr=get_hour() 
        if hr >= 20 and hr <= 6:
            #print("night time")
            if not day == 1:
                self._logger.info(" ----- night time -----")
                day = 1
        else:
            #print("day time") 
            if not day == 2:
                self._logger.info(" ----- day time ------")
                day = 2
        time.sleep(LOOP_CYCLE)
