# calculates a gas ppm for the sensor shown in the link below 
# https://www.az-delivery.uk/products/mq-135-gas-sensor-modul
#
# read LTC2450 ADC on SPI
import spidev
import time
class LTC2450_ADC_SPI():
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)                                               # port 0,cs 0
        self.spi.max_speed_hz = 1000000                                  # speed 1MHz
        self.adc = 0
        self.dataV = 0
        
    def __init__(self):
        self.spi.close()
        
    def adc_read_u16(self):
        ret = 0
        dataV = 0
        try:
            self.adc = self.spi.xfer2([0x00,0x00])
            self.dataV = (self.adc[0] << 8) | self.adc[1]
        except :
            self.spi.close() 
            time.sleep(0.1)
            self.spi.open(0,0)                                             # port 0,cs 0
            spi.max_speed_hz = 1000000                                     # speed 1MHz  
            try:
                self.adc = spi.xfer2([0x00,0x00])
                self.dataV = (self.adc[0] << 8) | self.adc[1]  
            except : 
                ret = 1            
        return ret, dataV

# calculate the ppm from the voltage measured from the sensor
import math 
class GAS_MQ_135():
    def __init__(self):
        self.ani = LTC2450_ADC_SPI()
        self.ppm = 0
        self.raw_valu = 0
        self.ret = 0
        
    def __init__(self):
        del self.ani

    def get_reading(self):
        self.ret, self.raw_valu = self.ani.adc_read_u16()
       
    def mq135_gas_ppm(analog_v, ref_v=3.3 raw_c=4095.0, RL = 10.0, Ro = 0.54, m = -0.280, b = 0.425):

        sensor_volt = analog_v * (ref_v / raw_c)  

        RS_gas = (RL * (ref_v - sensor_volt) / sensor_volt)
        RS_ratio = RS_gas / Ro
        ppm = math.log10(RS_ratio) * m + b                                        # Calculate PPM (use the correct logarithmic formula)
        return ppm
        
    def read_gas_ppm(self):
        self.get_reading()
        self.ppm = self.mq135_gas_ppm(self.raw_valu) 
        return self.ppm
        