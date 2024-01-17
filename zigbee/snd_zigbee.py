'''
Reads data and sends over zigbee
'''
from ina219 import INA219                            # used to check battery level
from machine import I2C
import sys
import xbee
import time

TEMP_OFFSET = 14.0                                   # Internal temperature rise
addr = '\x00\x00\x00\x00\x00\x00\xFF\xFF'            # XBee broadcast address
# if you want to send to a specific target rather than broadcast TARGET_64BIT_ADDR = b'\x00\x13\xA2\x00\x41\xA7\xAD\xBC'

# The following commands affect the UART serial interface
# ［BD(UART baud rate) <XBee3><S2C><S2
# This command sets the serial interface baud rate for communication between the device's UART port and the host.
# The device interprets any value between 0x12C and 0x0EC400 as the custom baud rate.
# The custom baud rate is not guaranteed and the device will attempt to find the closest baud rate achievable.
# After setting a non-standard baud rate, query the BD to find the actual operating baud rate before applying the change.
# The following table shows some examples of sent BD parameters versus stored parameters.
# Parameter Description
# 0x0 1200 baud
# 0x1 2400 baud
# 0x2 4800 baud
# 0x3 9600 baud
# 0x4 19200 baud
# 0x5 38400 baud
# 0x6 57600 baud
# 0x7 115200 baud
# 0x8 230,400 baud
# 0x9 460,800 baud
# 0xA 921,600 baud
# Parameter range
# Standard baud rate : 0x0-0x07 <S2
# Standard baud rate : 0x0-0x0A <XBee3><S2C>
# Non-standard baud rate : 0x80 -0x0E1000 <S2>
# Non-standard baud rate : 0x12C-0x0EC400 <XBee3> <S2C>
# Default
# 0x03 (9600 baud)
#
# ［NB (Parity)] <XBee3><S2C><S2
# Sets or reads the serial parity setting for UART communication.
# Parameter Description
#　0 No parity
#　1 Even parity
#　2 Odd parity
#3 Mark parity (always "1") <S2C><S2
#
# ［SB (stop bit)] <XBee3><S2C><S2
# Sets or displays the number of stop bits for UART communication.
# Parameter Description
#　0 One stop bit
#　1 Two stop bits
# Default
#　0
#
def set_comms_params(baud=0xA, par=1, sb=0):
    network_settings = {"BD": baud, "NB": par, "SB": sb}
    for command, value in network_settings.items():
        xbee.atcmd(command, value)
    xbee.atcmd("AC")                                 # write changes
    time.sleep(1) 

# gets the i2c module
def read_i2c_battery():
    try:
        battery_voltage = str(battery_ina.voltage())
        battery_current = str(battery_ina.current())
    except:
        print("INA219:0x44: Battery read failed...")
        battery_voltage = -1
        battery_current = -1
    return battery_voltage, battery_current
        
# gets temp and volts from xbee module        
def getData(): 
    try:                                                                                                                           # define getTemp function
        temp = xbee.atcmd('TP') - TEMP_OFFSET                                                                                      # Get temperature value from XBee module
        volt = xbee.atcmd('%V') / 1000                                                                                             # Get voltage value from XBee module
    except:
        print("ZigBee: Battery/Temp read failed...")
        temp = -1
        volt = -1
    i2cv, i2ca = read_i2c_battery()                                                                                                # read o2c battery
    time_s = str(time.ticks_cpu())                                                                                                 # get timestamp
    return {'xbee_temp': str(temp), 'xbee_volt': str(volt), 'i2c_volt': str(i2cv), 'i2c_current': str(i2ca), ts: str(time_s)}      # Return the obtained result

# Instantiate an I2C peripheral.
i2c = I2C(1)
for address in i2c.scan():
    print("- I2C device found at address: %s" % hex(address))
battery_ina = INA219(0.1, I2C(1), 0x44)
try:
    print("INA219:0x44: Configuring Battery...")
    battery_ina.configure_32v_2a()
except:
    print("INA219:0x44: Battery Missing...")

chosen_baud=0x7
set_comms_params(chosen_baud)                                                    # baud to 115200
 
# wait for zigbee connection 
while True:
    status = xbee.atcmd('AI')                             # Check network participation status
    print('.',end='')
    if status == 0x00:                                    # Exit the loop when in participating state
        break
    xbee.atcmd('CB',0x01)                                 # Commissioning (network participation)
    time.sleep_ms(2000)                                   # 2 seconds wait time processing
print('\nJoined')

# forEver
while True:
    sensor = getData()                                    # Get sensor value
    payload = sensor
    # if you want csv use this payload instead
    # payload = str(sensor['xbee_temp']) + ',' + str(sensor['xbee_volt'+ ',' + str(sensor['i2c_volt'+ ',' + str(sensor['i2c_current'])
    print('sending this to the server :', payload)
    try:
        xbee.transmit(addr, payload)                     # Send the obtained value using XBee
        time.sleep_ms(3000)                              # 3 seconds waiting time processing
    except:
        print("XBee: TX Failed...")        