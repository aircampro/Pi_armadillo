#!/usr/bin/env python
# Example of reading Toledo Scale(s) and load cell and sending the data to a siemens s7 1500 PLC on TIA portal using snap7
#

# Parameters for the Mettler Toledo XS105 scale:
# 9600
# 8/No
# 1 stopbit
# Xon/Xoff
# <CR><LF>
# Ansi/win
# Off

import serial
import time
import re

# raw read of the toledo scales using serial lib on serial port S0
#
def get_mass_from_XS105(port='/dev/ttyS0'):
#    ser = serial.Serial(port='COM1',  ## if using Windows serial
    ser = serial.Serial(port, baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS )

    if not ser.isOpen():
        ser.open()
    
    ser.write(b'\nSI\n')                                     # send SI to scale to send value
    time.sleep(1)                                            # give moment for balance to settle
    
    value = ser.read(ser.in_waiting)                         # changed from (ser.in_waiting()) due to PySerial updates
    value = value.decode('utf-8')
    value = value.split('\n')[1][:-1]

    if value[3] == 'S':
        stability = True
    else:
        stability = False
    weight = value[4:-1].strip(' ')

    if ser.isOpen():
        ser.close()
        
    return((weight, stability))

# using toldeo library reads all the toledo on the usb hub for example
#
#from mettler_toledo_device import MettlerToledoDevice - for single p-p link
from mettler_toledo_device import MettlerToledoDevices
#import mettler_toledo_device
def get_tol_weight(use_ports=['/dev/ttyUSB0','/dev/ttyUSB1']):
    devs = MettlerToledoDevices()                            # Might automatically find all available devices
    if devs == None:
        # if they are not found automatically, specify ports to use
        #devs = MettlerToledoDevices(use_ports=['/dev/ttyUSB0','/dev/ttyUSB1'])                                    # Linux
        #devs = MettlerToledoDevices(use_ports=['/dev/tty.usbmodem262471','/dev/tty.usbmodem262472'])              # Mac OS X
        #devs = MettlerToledoDevices(use_ports=['COM3','COM4'])                                                    # Windows	
        devs = MettlerToledoDevices(use_ports) 
    dev_wts = []
    for i, dev in enumerate(devs):
        print(i, dev)
        ser_no = dev.get_serial_number()
        vv = dev.get_weight() + [i] + [ser_no]
        # [-0.6800, 'g', 'S'] -- if weight is stable
        # [-0.6800, 'g', 'D'] -- if weight is dynamic
        dev_wts.append(vv)
    return dev_wts
	
# siemens s7-1500 PLC using snap7 library
#
def connectS7PLC(host='192.168.0.14'):
    import snap7
    from snap7 import util

    client = snap7.client.Client()
    client.connect(host,0,1)
    client.get_connected()
    return client
	
# write a real 4 bytes long
#
def writeS7Real(client, s7db=2,s7offset=10, s7real=4, val=98.7):
    db = client.db_read(s7db, s7offset, s7real)
    db2 = util.set_real(db,val)
    client.db_write(s7db,s7offset,db2) 

# Load cell amplifier HX711 (Sparkfun), e.g. from mouser.co.uk: 474-SEN-13879
# Load cell (100g and 500g cells used), e.g. from mouser.co.uk: 474-SEN-14727 or 474-SEN-14728
#
def conect_load_cell_scale(serial_port = "/dev/ttyS1"):
    import numpy as np
    from serial_weighing_scale import SerialWeighingScale
    scale = SerialWeighingScale(port=serial_port)
    return scale

def read_load_cell_scale(scale, mode=0):
    if mode == 0:
        v = scale.read_weight()                                        # Take single measurement
    else:
        v = scale.read_weight_reliable(n_readings=5, measure=np.mean)  # Get statistic of n readings  
    return v
    
# loop to read and write
#	
def main():
    cli1 = connectS7PLC('10.0.1.1')                 # first cleint PLC
    cli2 = connectS7PLC('10.0.1.2')                 # second client PLC
    scale1 = conect_load_cell_scale()               # conenct to the weigh scale on serial port 1
    try:
        while True:	
            val = get_mass_from_XS105()            # reads the weigher on the first serial port 0
	        if val[1] == 'S':                      # stable reading else hold last known value on client
                writeS7Real(cl1,0,20,4,val[0])
                writeS7Real(cl2,0,20,4,val[0])
            vals = get_tol_weight()                # reads all the weighers on the usb link
            for i, dev in enumerate(vals):
                if dev[2] == 'S':                  # stable reading else hold last known value on client
                    if dev[1] == 'g':
                        writeS7Real(cl1,4+(4*i),4,dev[0])
                        writeS7Real(cl2,4+(4*i),4,dev[0])	
                    elif dev[1] == 'Kg': 
                        writeS7Real(cl1,4+(4*i),4,dev[0]*1000)
                        writeS7Real(cl2,4+(4*i),4,dev[0]*1000)  
            v = read_load_cell_scale(scale1)    
            i+=1
            writeS7Real(cl1,4+(4*i),4,v)
            writeS7Real(cl2,4+(4*i),4,v)            
    except (KeyboardInterrupt, ValueError, Exception) as e:
        print("---- exited close s7 ports ------")	
        cli1.disconnect()
        cli2.disconnect()
		
if __name__ == '__main__':   

    main()
