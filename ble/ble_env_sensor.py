#!/usr/bin/env python
 
"""
read the values from the env sensor 
"""
import asyncio
from bleak import BleakClient
import sys

WX_BEACON=0xF8A8
OMRON=0xD502
ENV_SENSOR_MANUFACTURER_CODE = OMRON                        # first part of MAC Address is manufacturer code

import requests
from influxdb import InfluxDBClient
client = InfluxDBClient(host='localhost', port=8086, username='root', password='root_passwd', database='env_sensor')
measurement = 'air5'
tags = {'place': 'leaf','host': 'WxBeacon2'}

# create database on influxdb
def create_influx_database():
    v = "q=CREATE DATABASE " + 'env_sensor' + "\n"
    uri = ("http://" + 'localhost' + ":" +
           '8086' + "/query")
    r = requests.get(uri, params=v)
    if debug:
        print "-- created database : " + str(r.status_code)
        
def MSG(*args):
    msg = " ".join([str(a) for a in args])
    print(msg)
	sys.stdout.flush()
		
# environmental sensor decoder and database writer
def decode_env_data_put_in_db(dataRow):
	(seq, temp, humid, light, uv, press, noise, discom, heat, batt) = struct.unpack('<BhhhhhhhhH', dataRow)
	MSG(float(seq), float(temp) / 100, float(humid) / 100, float(light), float(uv) / 100, float(press) / 10, float(noise) / 100, float(discom) / 100, float(heat) / 100, float(batt) / 100)
	# create json for the database
	json_body = [
		{
			'measurement': measurement,
			'tags': tags,
			'fields': {
				'temp': float(temp) / 100 ,
				'humi': float(humid) / 100 ,
				'lux': float(light) ,
				'uv': float(uv / 100) ,
				'hpa': float(press / 10) ,
				'noise': float(noise / 100) ,
				'thi': float(discom / 100) ,
				'heat': float(heat / 100) ,
				'battery': float(batt / 100) ,
			}
		}
	]
	client.write_points(json_body)
    
# makes the UUID from the command bytes passed as arguments
def _MAKE_UUID(cmd):
    return ('%08X-7700-46F4-AA96-D5E974E32A54' % (0x0C4C0000 | (cmd&0xFFFF)))

def _MAKE_SAFE_UUID(cmd):   
    try:
        return ('%08X-7700-46F4-AA96-D5E974E32A54' % (0x0C4C0000 | (cmd&0xFFFF)))
    except Exception as e:
        print("you must pass a integer number in hex or decimal to the function _MAKE_UUID")
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
        uuid = _MAKE_UUID(0x3001)                                                       # command 0x3001
        values = await client.read_gatt_char(uuid)
        decode_env_data_put_in_db(values)
        await client.disconnect()                                                       # disconnect the central from the periferal env_sensor mug

# run the main thread
if __name__ == '__main__':
    asyncio.run(main())