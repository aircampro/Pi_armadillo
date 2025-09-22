#!/usr/bin/env python 
#
# example sending power consumption data to kafka 
# reads INA219 chip on the I2C bus use GY-219 module to change the address
# 0×40 (no jumpers installed - default); 0×41 (jumper A0); 0×44 (jumper A1); 0×45 (jumpers A0 and A1)
# ref :- https://www.ti.com/product/INA219#samples
#
# if your not using raspberry pi or beaglebone e.g. Repka Pi 
# edit get_default_bus() for your bus number. you can get it with i2cdetect -l
#
import logging
LIB=0
if LIB == 1:
    # https://github.com/hrshovon/py_ina219_smbus/tree/master address here fixed at 0x40 and default bus = 0 
    from ina_219_smbus import ina219_i2c as INA219
    RANGE_16V = 0     # Range 0-16 volts
    RANGE_32V = 1     # Range 0-32 volts
    GAIN_1_40MV = 0   # Maximum shunt voltage 40mV
    GAIN_2_80MV = 1   # Maximum shunt voltage 80mV
    GAIN_4_160MV = 2  # Maximum shunt voltage 160mV
    GAIN_8_320MV = 3  # Maximum shunt voltage 320mV
    GAIN_AUTO = -1    # Determine gain automatically
else:
    # https://github.com/chrisb2/pi_ina219 
    from ina219 import INA219
# as per GY-219 which measures up to 3.2 A
SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 0.2
# specify the bus number
BNUM=1
# address (with jumper A0 on)
ADDR=0x41

# specify the address 
import json
import random
import time
from confluent_kafka import Producer

# Producer'а configuration
config = {
    'bootstrap.servers': 'localhost:9092',
    'client.id': 'python-producer'
}
producer = Producer(config)
KTOPIC='sensor_data'
	
# generate json message with the data
def generate_data(id=1):
    if LIB == 1:
        ina = INA219(_BUS=BNUM)  
        ina.set_gain(GAIN_AUTO)
        ina.set_brang(RANGE_16V)        
    else:      
        ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, busnum=BNUM, address=ADDR, log_level=logging.INFO)
        ina.configure(ina.RANGE_16V, ina.GAIN_AUTO)
    return {
        'sensor_id': id,
        'bus_voltage': ina.voltage(),
        'bus_current': ina.current(),
        'supply_voltage': ina.supply_voltage(),
        'shunt_voltage': ina.shunt_voltage(),
        'power': ina.power(),		
        'timestamp': int(time.time())
    }

# serialise to в JSON
def serialize_data(data):
    return json.dumps(data)

# send the message to the kafka topic
def send_message(topic, data):
    producer.produce(topic, value=data)
    producer.flush()

# get the data from i2c and publish it to the kafka topic
try:
    while True:
        data = generate_data()       
        serialized_data = serialize_data(data)
        send_message(KTOPIC, serialized_data)
        print(f'Sent data: {serialized_data}')
        time.sleep(1)
except KeyboardInterrupt:
    print('Stopped.')

producer.close()