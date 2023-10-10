#!/usr/bin/env python
 
"""
read the values from the given env sensor defined in the sensor_typ variable
"""
import asyncio
from bleak import BleakClient
import sys

WX_BEACON=0xF8A8
OMRON=0xD502
ENV_SENSOR_MANUFACTURER_CODE = WX_BEACON                        # first part of MAC Address is manufacturer code
sensor_typ="wx"

import requests
from influxdb import InfluxDBClient
client = InfluxDBClient(host='localhost', port=8086, username='root', password='root_passwd', database='env_sensor')
measurement = 'air5'
tags = {'datebase' : 'env_sensor', 'gateway' : 'localhost', 'semsor_type' : 'WxBeacon2'}

# define s class which contains our data from all env monitors
#
class ENV_MON_JSON_T():
    def __init__(self):
        self.temp = 0.0
        self.humi = 0.0 
        self.lux = 0.0
        self.uv = 0.0
        self.hpa = 0.0
        self.noise = 0.0
        self.thi = 0.0
        self.heat = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.etvoc = 0.0
        self.eco2 = 0.0  
        self.si = 0.0 
        self.pga = 0.0 
        self.seismic = 0.0
        self.vib = 0.0
        self.vibinfo = 0.0
        self.battery = 0.0
        self.dia = 0.0
        self.wgbt = 0.0
        self.rssi = 0.0
        self.distance = 0.0
        self.ignore1 = 0
                    
# create database on influxdb
def create_influx_database():
    v = "q=CREATE DATABASE " + 'env_sensor' + "\n"
    uri = ("http://" + 'localhost' + ":" +
           '8086' + "/query")
    r = requests.get(uri, params=v)
        
def MSG(*args):
    msg = " ".join([str(a) for a in args])
    print(msg)
	sys.stdout.flush()

def discomfort_index_approximation(temp, humi):
    return (0.81 * temp) + 0.01 * humi * ((0.99 * temp) - 14.3) + 46.3

def wbgt_approximation(temp, humi, flag_outside=False):
    wbgt = 0
    if (temp < 0):
        temp = 0
    if (humi < 0):
        humi = 0
    if (humi > 100):
        humi = 100
    wbgt = (0.567 * temp) + 0.393 * (
    humi / 100 * 6.105 * math.exp(
        17.27 * temp / (237.7 + temp))) + 3.94
    if not flag_outside:
        wbgt = (wbgt + (1.1 * (1 - (humi / 62) * 1.6)) * (temp - 30) *
        0.17 - abs(temp - 30) * 0.09) / 1.135
    return wbgt

def return_accuracy(self, rssi, power):  # rough distance in meter
    RSSI = abs(rssi)
    if RSSI == 0:
        return -1
    if power == 0:
        return -1

    ratio = RSSI * 1.0 / abs(power)
    if ratio < 1.0:
        return pow(ratio, 8.0)
    accuracy = 0.69976 * pow(ratio, 7.7095) + 0.111
    # accuracy = 0.89976 * pow(ratio, 7.7095) + 0.111
    return accuracy
        
# environmental sensor decoder and database writer WX_BEACON
#
# Handle ENRON Sensor OMRON Environment Sensor (2JCIE-BL01 & BU01)MRON Environment Sensor (2JCIE-BL01 & BU01)
#
def decode_env_data_put_in_db(dataRow, sensor_type):

    # create the holding data structure
    #
    s = ENV_MON_JSON_T()
    
    # unpack the data as returned from the read_gatt_char call on the uuid
    #
    if (sensor_type == "wx"):
        (s.seq, s.temp, s.humid, s.light, s.uv, s.press, s.noise, s.discom, s.heat, s.batt) = struct.unpack('<BhhhhhhhhH', dataRow)
        MSG(sensor_type, float(s.seq), float(s.temp), float(s.humid), float(s.light), float(s.uv), float(s.press), float(s.noise), float(s.discom), float(s.heat), float(s.batt))
        s.batt = float(s.batt) / 100.0
    elif (sensor_type == "ep"):
        (s.seq, s.temp, s.humid, s.light, s.uv, s.press, s.noise, s.discom, s.heat, s.ignore1, s.batt) = struct.unpack('<BhhhhhhhhhB', dataRow)
        MSG(sensor_type, float(s.seq), float(s.temp), float(s.humid), float(s.light), float(s.uv), float(s.press), float(s.noise), float(s.discom), float(s.heat), float(s.batt))
        s.batt = 10.0 * (float(s.batt) + 100.0)
    elif (sensor_type == "im"):
        (s.seq, s.temp, s.humid, s.light, s.uv, s.press, s.noise, s.x, s.y, s.z, s.batt) = struct.unpack('<BhhhhhhhhhB', dataRow)
        MSG(sensor_type, s.seq, s.temp, s.humid, s.light, s.uv, s.press, s.noise, s.x, s.y, s.z, s.batt)
        s.batt = 10.0 * (float(s.batt) + 100.0)
    elif (sensor_type == "rbt01"):
        (s.seq, s.temp, s.humid, s.light, s.uv, s.press, s.noise, s.etvoc, s.eco) = struct.unpack('<Bhhhhhhhh', dataRow)
        MSG(sensor_type, s.seq, s.temp, s.humid, s.light, s.uv, s.press, s.noise, s.etvoc, s.eco)
    elif (sensor_type == "rbt02"):
	(s.seq, s.discom, s.heat, s.vib, s.si, s.pga, s.seismic, s.x, s.y, s.z) = struct.unpack('<BhhBhhhhhh', dataRow)
        MSG(sensor_type, s.seq, s.discom, s.heat, s.vib, s.si, s.pga, s.seismic, s.x, s.y, s.z)
        if (s.vib == 0x00):
            s.vibinfo = "NONE"
        elif (vib == 0x01):
            s.vibinfo = "VIBRATION"
        elif (vib == 0x02):
            s.vibinfo = "EARTHQUAKE"
    s.dia = discomfort_index_approximation((float(s.temp) / 100), (float(s.humid) / 100))
    s.wbgt = wbgt_approximation((float(s.temp) / 100), (float(s.humid) / 100))
    MSG("discomfort and wbgt ", s.dia, s.wbgt)
    
    # scaling conversions
    #
    s.temp = float(s.temp) / 100.0 
    s.humid = float(s.humid) / 100.0 
    s.light = float(s.light) 
    s.uv = float(s.uv) / 100.0
    s.press = float(s.press) / 10.0
    s.noise = float(s.noise) / 100.0
    s.discom = float(s.discom) / 100.0
    s.heat = float(s.heat) / 100.0
    s.x = float(s.x) / 10.0
    s.y = float(s.y) / 10.0
    s.z = float(s.z) / 10.0
    s.si = float(s.si) / 10.0,
    s.pga = float(s.pga) / 10.0,
    s.seismic = float(s.seismic) / 1000.0,
                
    # create json for the database
    #
    json_body = [
        {
	    'measurement': measurement,
	    'tags': tags,
	    'fields': {
                'temp': s.temp,
                'humi': s.humid,
                'lux': s.light,
                'uv': s.uv,
                'hpa': s.press,
                'noise': s.noise,
                'thi': s.discom,
                'heat': s.heat,
                "accel_x": s.x,
                "accel_y": s.y,
                "accel_z": s.z,
                "etvoc": s.etvoc,
                "eco2": s.eco,
                "si": s.si,
                "pga": s.pga,
                "seismic": s.seismic,
                "vibinfo": s.vibinfo,
                'battery': s.batt,
                'dia' : s.dia,
                'wbgt' : s.wbgt,
                "rssi": s.rssi,
                "distance": s.distance
            }
        }
    ]
    
    # upload to the db
    #
	client.write_points(json_body)
    
# makes the UUID from the command bytes passed as arguments
def _MAKE_UUID(cmd):
    return ('%08X-7700-46F4-AA96-D5E974E32A54' % (0x0C4C0000 | (cmd&0xFFFF)))

def _MAKE_SAFE_UUID(cmd):   
    try:
        return ('%08X-7700-46F4-AA96-D5E974E32A54' % (0x0C4C0000 | (cmd&0xFFFF)))
    except Exception as e:
        print("you must pass a integer number in hex or decimal to the function _MAKE_UUID set to zero in returned value")
        return ('%08X-7700-46F4-AA96-D5E974E32A54' % (0x0C4C0000))   
        
# define the main thread    
async def main():
    try:
        create_influx_database()
    except Exception as e:
        print(f"unable to create database {e}")
        
    devices = await discover()
    for d in devices:
        if ENV_SENSOR_MANUFACTURER_CODE in d.metadata.get('manufacturer_data', {}):
            user_input = input('Device {!r} found. Is this your env sensor? Y/N [Y]: '.format(d.name)) or 'y'
            if user_input.lower() == 'y':
                env_sensor = d
                break
    else:
        print('env_sensor mug is not found. Exiting...')
        return

    #-#-#-# Now get and set the requested data #-#-#-#
    async with BleakClient(env_sensor.address) as client:
        #values = await client.read_gatt_char('0C4C3001-7700-46F4-AA96-D5E974E32A54')   # read env sensor with hard coded uid
        uuid = _MAKE_SAFE_UUID(0x3001)                                                       # command 0x3001
        values = await client.read_gatt_char(uuid)
        decode_env_data_put_in_db(values, sensor_typ)
        await client.disconnect()                                                       # disconnect the central from the periferal env_sensor mug

# run the main thread
if __name__ == '__main__':
    asyncio.run(main())
