#!/usr/bin/python3
# -*- coding: utf-8 -*-

#
# example of use of object classes to read various sensors from spi or i2c
#

#
# with raspberry pi enable as follows
#
# sudo vi /boot/config.txt
# Uncomment some or all of these to enable the optional hardware interfaces
#dtparam=i2c_arm=on
#dtparam=i2s=on
#dtparam=spi=on

from smbus2 import SMBus
import time

# =========================================== i2c =============================================
#
# bosch Bme280
# sudo apt-get install i2c-tools
# check address with sudo i2cdetect -y 1
# i2cdump -y 1 0x76
# sudo pip install smbus2
#
# bosch bme280
# wget https://raw.githubusercontent.com/SWITCHSCIENCE/BME280/master/Python27/bme280_sample.py
#
class Bme280:

    def __init__(self, busNumber=1, i2cAddress=0x76):

        self.bus = SMBus(busNumber)
        self.i2cAddress = i2cAddress
        self.digT = []
        self.digP = []
        self.digH = []
        self.timeFine = 0.0
        self.presRaw  = 0.0
        self.tempRaw  = 0.0
        self.humRaw   = 0.0

        osrsT   = 1         #Temperature oversampling x 1
        osrsP   = 1         #Pressure oversampling x 1
        osrsH   = 1         #Humidity oversampling x 1
        mode    = 3         #Normal mode
        tSb     = 5         #Tstandby 1000ms
        filter  = 0         #Filter off
        spi3wEn = 0         #3-wire SPI Disable

        ctrlMeasReg = (osrsT << 5) | (osrsP << 2) | mode
        configReg   = (tSb << 5) | (filter << 2) | spi3wEn
        ctrlHumReg  = osrsH

        self.writeReg(0xF2,ctrlHumReg)
        self.writeReg(0xF4,ctrlMeasReg)
        self.writeReg(0xF5,configReg)
        self.getCalibParam()

        self.readData()

    def writeReg(self, regAddress, data):
        self.bus.write_byte_data(self.i2cAddress, regAddress, data)

    def getCalibParam(self):
        calib = []

        for i in range (0x88,0x88+24):
            calib.append(self.bus.read_byte_data(self.i2cAddress,i))
        calib.append(self.bus.read_byte_data(self.i2cAddress,0xA1))
        for i in range (0xE1,0xE1+7):
            calib.append(self.bus.read_byte_data(self.i2cAddress,i))

        self.digT.append((calib[1] << 8) | calib[0])
        self.digT.append((calib[3] << 8) | calib[2])
        self.digT.append((calib[5] << 8) | calib[4])
        self.digP.append((calib[7] << 8) | calib[6])
        self.digP.append((calib[9] << 8) | calib[8])
        self.digP.append((calib[11]<< 8) | calib[10])
        self.digP.append((calib[13]<< 8) | calib[12])
        self.digP.append((calib[15]<< 8) | calib[14])
        self.digP.append((calib[17]<< 8) | calib[16])
        self.digP.append((calib[19]<< 8) | calib[18])
        self.digP.append((calib[21]<< 8) | calib[20])
        self.digP.append((calib[23]<< 8) | calib[22])
        self.digH.append( calib[24] )
        self.digH.append((calib[26]<< 8) | calib[25])
        self.digH.append( calib[27] )
        self.digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        self.digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append( calib[31] )

        for i in range(1,2):
            if self.digT[i] & 0x8000:
                self.digT[i] = (-self.digT[i] ^ 0xFFFF) + 1

        for i in range(1,8):
            if self.digP[i] & 0x8000:
                self.digP[i] = (-self.digP[i] ^ 0xFFFF) + 1

        for i in range(0,6):
            if self.digH[i] & 0x8000:
                self.digH[i] = (-self.digH[i] ^ 0xFFFF) + 1  

    def readData(self):
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.i2cAddress,i))
        self.presRaw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        self.tempRaw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        self.humRaw  = (data[6] << 8)  |  data[7]

    def getPressure(self):
        pressure = 0.0

        v1 = (self.timeFine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768

        if v1 == 0:
            return 0
        pressure = ((1048576 - self.presRaw) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)
        return pressure/100

    def getTemperature(self):
        v1 = (self.tempRaw / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (self.tempRaw / 131072.0 - self.digT[0] / 8192.0) * (self.tempRaw / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        self.timeFine = v1 + v2
        temperature = self.timeFine / 5120.0
        return temperature

    def getHumidity(self):
        varH = self.timeFine - 76800.0
        if varH != 0:
            varH = (self.humRaw - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * varH)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * varH * (1.0 + self.digH[2] / 67108864.0 * varH)))
        else:
            return 0
        varH = varH * (1.0 - self.digH[0] * varH / 524288.0)
        if varH > 100.0:
            varH = 100.0
        elif varH < 0.0:
            varH = 0.0
        return varH

"""
Python Aosong DHT12 I2C driver
"""
class DHTBaseI2C:
          
    def __init__(self, busNumber=1, i2cAddress=0x5c):

        self.bus = SMBus(busNumber)
        self.addr = i2cAddress
        self.buf = bytearray(5)
        self.humidity  = 0.0
        self.tempRaw  = 0.0
        
        self.readData()

    def readData(self):
        for i in range (0, 5):
            self.buf.append(self.bus.read_byte_data(self.i2cAddress,i))
        if (self.buf[0] + self.buf[1] + self.buf[2] + self.buf[3]) & 0xff != self.buf[4]:
            raise Exception("checksum error")
        self.humidity = self.buf[0] + self.buf[1] * 0.1
        t = self.buf[2] + (self.buf[3] & 0x7f) * 0.1
        if self.buf[3] & 0x80:
            t = -t
        self.tempRaw = t

    def humidity(self):
        return self.buf[0] + self.buf[1] * 0.1
        
    def temperature(self):
        t = self.buf[2] + (self.buf[3] & 0x7f) * 0.1
        if self.buf[3] & 0x80:
            t = -t
        return t

# BH1750 Lux Sensor IC ported not tested      
class Bh1750:

    def __init__(self, busNumber=1, i2cAddress=0x23):

        self.bus = SMBus(busNumber)
        self.i2cAddress = i2cAddress
        ctrlMeasReg = 0x01 # power on
        self.buf = bytearray(2)
        self.lux_raw = 0.0
        self.writeReg(i2cAddress,ctrlMeasReg)
        self.readData()

    def writeReg(self, regAddress, data):
        self.bus.write_byte_data(self.i2cAddress, regAddress, data)
        
    def readData(self):
        high_res_mode = 0x10
        self.writeReg(self.i2cAddress,high_res_mode)
        for i in range(len(self.buf)):
            self.buf.append(self.bus.read_byte_data(0x0,i))
        self.lux_raw = ((self.buf[0] << 8) | self.buf[1]) / 1.2

# IAQ2000 environment monitor IC ported not tested   
class Iaq2000:

    def __init__(self, busNumber=1, i2cAddress=0x5A):

        self.bus = SMBus(busNumber)
        self.i2cAddress = i2cAddress
        self.buf = bytearray(9)
        self.iaqpred = 0 
        self.iaqstatus = 0
        self.iaqtvoc = 0
        self.readData()

    def writeReg(self, regAddress, data):
        self.bus.write_byte_data(self.i2cAddress, regAddress, data)
        
    def readData(self):
        high_res_mode = 0x10
        self.writeReg(self.i2cAddress,high_res_mode)
        for i in range(len(self.buf)):
            self.buf.append(self.bus.read_byte_data(0x0,i))
        self.iaqpred = ((self.buf[0] << 8) | self.buf[1]) 
        self.iaqstatus = self.buf[2]
        self.iaqtvoc = ((self.buf[7] << 8) | self.buf[8])

# LidarLite IC  ported and not tested       
class LidarLite:

    def __init__(self, busNumber=1, i2cAddress=0x62):

        self.bus = SMBus(busNumber)
        self.i2cAddress = i2cAddress
        ctrlMeasReg = 0x00 # power mode and sample time
        self.writeReg(i2cAddress,ctrlMeasReg)
        self.buf = bytearray(2)
        self.buf1 = bytearray(2)
        self.d1 = 0 
        self.d2 = 0 
        
    def writeReg(self, regAddress, data):
        self.bus.write_byte_data(self.i2cAddress, regAddress, data)
        
    def readData(self):
        high_res_mode = 0x10
        self.writeReg(self.i2cAddress,0x0)
        self.writeReg(self.i2cAddress,0x04)
        self.writeReg(self.i2cAddress,0x0F)
        for i in range(len(self.buf)):
            self.buf.append(self.bus.read_byte_data(0x0,i))
        self.d1 = ((self.buf[0] << 8) | self.buf[1]) 
        self.writeReg(self.i2cAddress,0x10)
        for i in range(len(self.buf1)):
            self.buf1.append(self.bus.read_byte_data(0x0,i))
        self.d2 = ((self.buf1[0] << 8) | self.buf1[1])
        
#
# ============================== spi =============================================================
#
# lsmod | grep spi
# ls -l /dev | grep spi
#
# git clone git://github.com/doceme/py-spidev
# $ cd py-spidev
# $ sudo python setup.py install

import spidev                       
import time                                             
 
# mcp3002 adc for knob or analog signal
class Mcp3002:

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0) 
        self.spi.max_speed_hz = 1000000 
      
    def spi_close(self):
        self.spi.close()
        
    def get_values(self,addr1,addr2):
        resp = self.spi.xfer2([addr1, addr2])              
        #resp = spi.xfer2([0x68, 0x00])                channel 0
        #resp = spi.xfer2([0x78, 0x00])                channel 1
        adc = ((resp[0] << 8) + resp[1]) & 0x3FF       # calc raw adc counts
        volts = (adc*5.0)/1024                         # scale counts value to volts
        return adc,volts

# KY018 light sensor
class Ky018:

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0) 
        self.spi.max_speed_hz = 1000000 
      
    def spi_close(self):
        self.spi.close()
    
    # Channel must be an integer 0-7        
    def read_light(self,channel=0):
        adc = self.spi.xfer2([1,(8+channel)<<4,0])              
        data = ((adc[1]&3) << 8) + adc[2]              # calc light quantity
        volts = (data * 3.3) / float(1023)             # scale light value to volts
        volts = round(volts,2)
        return adc,volts
 
# The MAX6675 is an IC that performs cold-junction compensation and digitizes the signals from type K thermocouples.The data is 
# connected via SPI with a 12-bit resolution (resolution: 0.25°C).The SPI uses a read-only format of 3-wire (CS, SCK, SO) to read data.
class Max6675:

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0) 
        self.spi.max_speed_hz = 1000000 
        self.spi.mode = 3
              
    def spi_close(self):
        self.spi.close()
        
    def get_value(self,addr1,addr2):
        readByteArray = self.spi.xfer2(addr1,addr2)
        temperatureData = ((readByteArray[0] & 0b01111111) << 5) | ((readByteArray[1] & 0b11111000) >> 3)
        temperature = temperatureData * 0.25
        return temperature

# ADXL345 accelorometer
class Adxl345:

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000 
        self.spi.mode = 3
        self.spi.xfer2([0x2D, 0x08]) # Start measurement 
        self.x_acc = 0.0
        self.y_acc  = 0.0
        self.z_acc  = 0.0
     
    def spi_close(self):
        self.spi.close()
        
    def get_values(self):
        x_data_list = self.spi.xfer2([0xc0|0x32, 0x00, 0x00])
        y_data_list = self.spi.xfer2([0xc0|0x34, 0x00, 0x00])
        z_data_list = self.spi.xfer2([0xc0|0x36, 0x00, 0x00])
        x_data = x_data_list[1] | (x_data_list[2] << 8)
        y_data = y_data_list[1] | (y_data_list[2] << 8)
        z_data = z_data_list[1] | (z_data_list[2] << 8)
        # Convert 2 complement to adic 10
        if(x_data & 0x8000):
            x_data = ((~x_data & 0xFFFF) + 1)*-1
        if(y_data & 0x8000): 
            y_data = ((~y_data & 0xFFFF) + 1)*-1
        if(z_data & 0x8000):
            z_data = ((~z_data & 0xFFFF) + 1)*-1
        # Convert to acceleration (D range ±2g)
        x_data = 2 * 9.8 * x_data / 0x7FFF
        y_data = 2 * 9.8 * y_data / 0x7FFF
        z_data = 2 * 9.8 * z_data / 0x7FFF
        print('x: {:4.2f}, y: {:4.2f}, z: {:4.2f} [m/s^2]'.format(x_data, y_data, z_data))
        self.x_acc = x_data
        self.y_acc = y_data
        self.z_acc = z_data        
        return x_data,y_data_z_data

# LSM303 accelorometer ---- check the profile for the write / read message
class Lsm303:

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000 
        self.spi.mode = 3
        self.spi.xfer2([0x20, 0xAF]) # set the accelerometer data rate to 1600 Hz. Do not update until we read values 
        self.spi.xfer2([0x24, 0xF0]) # 50 Hz magnetometer, high resolution, temperature sensor on
        self.spi.xfer2([0x26, 0x00]) # enable continuous reading of the magnetometer 
        self.x_acc = 0.0
        self.y_acc  = 0.0
        self.z_acc  = 0.0
        self.x_mag = 0.0
        self.y_mag  = 0.0
        self.z_mag  = 0.0
        self.temperature  = 0.0
        
    def spi_close(self):
        self.spi.close()
        
    def get_values(self):
        reg = 0x28|0x80
        byt_arr = []
        #for byt in range(0,5):
        #    reg = reg | 0x40
        #    byt_arr.append(self.spi.xfer2([reg, 0]))
        reg = reg | 0x40
        byt_arr = self.spi.xfer2([reg, 0])
        self.x_acc = (byt_arr[1] << 8) | byt_arr[0]
        self.y_acc = (byt_arr[3] << 8) | byt_arr[2]
        self.z_acc = (byt_arr[5] << 8) | byt_arr[4]
        reg = 0x08|0x80
        byt_arr = []
        #for byt in range(0,5):
        #    reg = reg | 0x40
        #    byt_arr.append(self.spi.xfer2([reg, byt]))  
        reg = reg | 0x40
        byt_arr = self.spi.xfer2([reg, 0])
        self.x_mag = (byt_arr[1] << 8) | byt_arr[0]
        self.y_mag = (byt_arr[3] << 8) | byt_arr[2]
        self.z_mag = (byt_arr[5] << 8) | byt_arr[4]            
        reg = 0x05|0x80
        byt_arr = []
        #for byt in range(0,1):
        #    reg = reg | 0x40
        #    byt_arr.append(self.spi.xfer2([reg, byt]))   
        reg = reg | 0x40
        byt_arr = self.spi.xfer2([reg, 0])
        self.temperature = (byt_arr[1] << 8) | byt_arr[0]
            
        
if __name__ == '__main__':
    # timestamp if you want to
    dt_now = datetime.datetime.now()     
    ts = dt_now.strftime('%Y/%m/%d %H:%M:%S')    
    # ============ example bme290 on i2c ==============================
    sensor = Bme280()
    try:
        print("pressure    : %7.2f hPa" % sensor.getPressure())
        print("temperature :  %-6.2f ℃" % sensor.getTemperature())
        print("humidity    : %6.2f ％" % sensor.getHumidity())
    except KeyboardInterrupt:
        pass
    # ============ example mcp3002 adc on spi ==========================
    knobs = Mcp3002()
    try:
        k1adc, k1volts = knobs.get_values(0x68,0x0) # first analog input 
        k2adc, k2volts = knobs.get_values(0x78,0x0) # second analog input  
        print("input 1 adc    : %8.0f , %2.5f" % (k1adc,k1volts))
        print("input 2 adc    : %8.0f , %2.5f" % (k2adc,k2volts))
    except KeyboardInterrupt:
        pass
    knobs.spi_close()
    # ============ example max5575 temp junction on spi ==========================
    temp1 = Max6675()
    try:
        actual_temp1 = temp1.get_value(0x00,0x00)   
        print("temperature    : %7.2f" % actual_temp1)
    except KeyboardInterrupt:
        pass
    temp1.spi_close()    
    # ============ example adxl345 accelerometer on spi ==========================
    accelo = Adxl345()
    try:
        x,y,z = accelo.get_values()   # or print(accelo.x_acc etc)
        print("x y z : %4.5f %4.5f %4.5f" % (x,y,z))
    except KeyboardInterrupt:
        pass
    accelo.spi_close()         
    # ============ example dht12 on i2c ==============================
    dht = DHTBaseI2C()
    try:
        print("humidity    : %7.4f " % dht.humidity)
        print("temperature :  %-6.2f ℃" % dht.tempRaw)
    except KeyboardInterrupt:
        pass            
