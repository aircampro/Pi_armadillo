#
# client is publishing enocean sensors to moquito broker, 
# subscriber sister application is to read this from the mosquito broker.
#
import paho.mqtt.client as mqtt

MQTT_PORT = 1883
KEEP_ALIVE = 60

TOPIC = "topic/B/"

import requests
import json
import serial
from sys import exit
from datetime import datetime
from flask import *

PORT = '/dev/ttyUSB400J'              # USB400J The name of the device to which it is connected
SENSORID1 = '04:01:53:e1'             # ID of the 1st STM-431J
SENSORID2 = '04:00:6f:e5'             # ID of the 2nd STM-431J
s_port = 0                            # serial port number handle
BAUD_RT = 57600                       # serial baud rate

URL = 'http://harvest.soracom.io/'    # SORACOM HarvestのURL

# This function reads 1 byte of data from the serial port and parses the EnOcean telegram. After analyzing 
# Telegram, data is sent to SORACOM Harvest only if the device ID matches, those selected at the top of code
def EnoceanAll(s_port):
    sensor1_rdy = False
    sensor2_rdy = False
    cnt = 0
    dataLen = 0
    optLen = 0
    telegraph,headList,dataList,optList = [],[],[],[]
    ready = True

    while True:
        if sensor1_rdy and sensor2_rdy:
            break

        # 1byte Reads data from the serial port one by one.
        s_data = s_port.read().encode('hex') # read 1byte

        # Sync-Recognizes the start of Telegram data from Byte 0x55.。
        if s_data == '55' and ready: # Telegram start
            # valuable reset
            cnt = 0
            dataLen = 0
            optLen = 0
            ready = False
            telegraph,headList,dataList,optList = [],[],[],[]

        cnt += 1
        telegraph.append(s_data)

        # We are analyzing Telegram data.
        if 2 <= cnt <= 5: # header
            headList.append(s_data)
        if cnt == 5: # header end, get data length
            dataLen = int(headList[1],16)
            optLen  = int(headList[2],16)
        if 7 <= cnt <= (6+dataLen): # data
            dataList.append(s_data)
        if (7+dataLen) <= cnt <= (6+dataLen+optLen): # optional data
            optList.append(s_data)
        if cnt == (6+dataLen+optLen+1): # Telegram end
            ready = True
            sensorId = ':'.join(dataList[1:5]) # Sensor ID
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # The data is displayed for debugging purposes.
            print "========"
            print timestamp
            print 'telegraph...:', ':'.join(telegraph)
            print 'head...', ':'.join(headList)
            print 'data...', ':'.join(dataList), '(length=%d)' % dataLen
            print 'opt ...', ':'.join(optList),  '(length=%d)' % optLen
            print 'sensorID...', sensorId


            # Temp sensor1 Send data when the sensor ID matches.
            if sensorId == SENSORID1 and sensor1_rdy == False:
                val = int(dataList[7],16)
                sensor1_temp = round((255.0-val)/255.0*40.0, 2)
                # ------ sendSoraComData('temperature01', sensor1_temp)
                sensor1_rdy = True

            # Temp sensor1 Send data when the sensor ID matches.
            elif sensorId == SENSORID2 and sensor2_rdy == False:
                val = int(dataList[7],16)
                sensor2_temp = round((255.0-val)/255.0*40.0, 2)
                # ------ sendSoraComData('temperature02', sensor2_temp)
                sensor2_rdy = True

            # Other sensors, ignore ID
            else:
                continue
                
    # return this data for webpage display via flask->uwgsi->nginx         
    if sensor1_rdy and sensor2_rdy:
        return str(sensor1_temp),str(sensor2_temp)
        
# Broker processes when connected to
def on_connect(client, userdata, flag, rc):
  print("Connect Broker:" + str(rc))

# Broker processes when disconnected 
def on_disconnect(client, userdata, rc):
  if rc != 0:
     print("disconnect broker")

# publish processes when publish done
def on_publish(client, userdata, mid):
  print("publish Done")

if __name__ == '__main__':

    client = mqtt.Client()

    # now connect the 
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish

    client.connect("localhost", MQTT_PORT, KEEP_ALIVE)

    # Open serial port
    try:
        s_port = serial.Serial(PORT, BAUD_RT)
        print('open serial port: %s' % PORT)
    except:
        print('cannot open serial port: %s' % PORT)
        exit(1)
                
    enO_temp_value1, enO_temp_value2 = EnoceanAll(s_port)
    msg = "sensor 1 " + enO_temp_value1
    client.publish(TOPIC,msg)
    msg = "sensor 2 " + enO_temp_value2
    client.publish(TOPIC,msg)
    client.disconnect()