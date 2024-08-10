# Example using ADAFruit libraries for reading temp, humidity and lux for a greenhouse
#
# SPDX-License-Identifier: MIT ADAFruit
#
# This example shows using two TSL2491 light sensors attached to PCA9546A channels 0 and 1.
# with humidity and temperature from DHT and write data to LCD
# Use with other I2C sensors would be similar.
#
import time
import board
import adafruit_tsl2591
import adafruit_tca9548a
import Adafruit_DHT
import Adafruit_CharLCD as LCD
import RPi.GPIO as GPIO
import sys
import PID
from scipy.interpolate import BSpline, make_interp_spline #  Switched to BSpline
import os.path

USE_STEMMA = 0                                # 0=uses board.SCL and board.SDA 1=the built-in STEMMA QT connector
def set_up_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

HEATER = 17   
def set_up_heater():
    GPIO.setup(HEATER, GPIO.OUT)

LIGHTS = 15
LIGHT_ON = 30
LIGHT_OFF = 90
def set_up_lights():
    GPIO.setup(LIGHTS, GPIO.OUT)
    GPIO.output(LIGHTS, False)     
    
def set_up_lcd():
    lcd_rs        = 21  # Note this might need to be changed to 21 for older revision Pi's.
    lcd_en        = 20
    lcd_d4        = 16
    lcd_d5        = 12
    lcd_d6        = 7
    lcd_d7        = 8
    lcd_columns = 16
    lcd_rows    = 2
    lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows)
    return lcd

targetT = 35
P = 10
I = 1
D = 1
SAMPLE_TM = 2.0
HUM_K = 0.9
HUM_LSP = 10
HUM_RSP = 1

# read configuration csv
def readConfig():
    global targetT
    global HUM_RSP
    global HUM_LSP
    global HUM_K
    with open ('/tmp/pid.conf', 'r') as f:
        config = f.readline().split(',')
        pid.SetPoint = float(config[0])
        targetT = pid.SetPoint
        pid.setKp = (float(config[1]))
        pid.setKi = (float(config[2]))
        pid.setKd = (float(config[3]))
        HUM_K = (float(config[1]))
        HUM_LSP = (float(config[2]))
        HUM_RSP = (float(config[3]))
    if HUM_LSP == 0:
        HUM_LSP = 0.1

# use if you want to write the config at the end       
def createConfig():
    if not os.path.isfile('/tmp/pid.conf'):
        with open ('/tmp/pid.conf', 'w') as f:
            f.write('%s,%s,%s,%s,%s,%s,%s'%(targetT,P,I,D,HUM_K,HUM_LSP,HUM_RSP))

def set_up_pid():
    readConfig()
    pid = PID.PID(P, I, D)
    pid.SetPoint = targetT
    pid.setSampleTime(SAMPLE_TM)
    return pid

if __name__ == '__main__':   
    # Create I2C bus 
    if USE_STEMMA == 0:
        i2c = board.I2C()                         # uses board.SCL and board.SDA
    elsa:
        i2c = board.STEMMA_I2C()                  # For using the built-in STEMMA QT connector on a microcontroller

    # Create the PCA9546A object and give it the I2C bus
    mux = adafruit_tca9548a.PCA9546A(i2c)

    # For each sensor, create it using the PCA9546A channel instead of the I2C object
    tsl1 = adafruit_tsl2591.TSL2591(mux[0])
    tsl2 = adafruit_tsl2591.TSL2591(mux[1])

    # set up the i/o interfaces
    set_up_gpio()
    set_up_heater()
    set_up_lights()
    lcd=set_up_lcd()
    pid=set_up_pid()
    
    # After initial setup, can just use sensors as normal.
    while True:
        # get lux and set the light contact
        print(tsl1.lux, " ", tsl2.lux, " ", time.time())
        lcd.clear()
        lcd.set_cursor(0,0)
        lcd.message('TS11 :'+str(int(tsl1.lux))+' LUX')    
        lcd.set_cursor(0,8)
        lcd.message('TS12 :'+str(int(tsl2.lux))+' LUX')
        if (int(ts11.lux) < LIGHT_ON) and (int(ts12.lux) < LIGHT_ON) :
            GPIO.output(LIGHTS, True) 
        elif (int(ts11.lux) > LIGHT_OFF) and (int(ts12.lux) > LIGHT_OFF) :        
            GPIO.output(LIGHTS, False)        
        time.sleep(1)
    
        # get temp and humidity
        humidity, temperature = Adafruit_DHT.read_retry(11, 4)
        print(humidity, " ", temperature, " ", time.time())
        lcd.clear()
        lcd.set_cursor(0,0)
        lcd.message('Humidity :'+str(int(humidity))+' %')    
        lcd.set_cursor(0,8)
        lcd.message('Temp :'+str(int(temperature))+' C')

        # if humid compensation is on add a litle more to the setpoint
        if (HUM_RSP == 1) and (humidity > HUM_LSP):
            pid.SetPoint = targetT + (HUM_K * (1.0/humidity))
        
        # do PID loop
        pid.update(temperature)
        targetPwm = pid.output
        targetPwm = max(min( int(targetPwm), 100 ),0)
        GPIO.output(HEATER, False)
        time.sleep(targetPwm)
        GPIO.output(HEATER, True)  
        time.sleep(1)
        
    createConfig()	
