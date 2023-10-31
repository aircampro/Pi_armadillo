# LoRa Gateway. Use Link Labs BS-8, an industrial-grade IoT/M2M gateway 
# for use with LoRa networks. It’s capable of supporting thousands of endpoints with 8 simultaneous receive channels.
#
# Get and plot data from TTN Console using Python

import paho.mqtt.client as mqtt
import json
import base64

APPEUI = 'YOURAPPEUI'
APPID  = 'YOUAPPID'
PSW    = 'YOURPASSWORD'

import matplotlib.pyplot as plt
#import DataPlot and RealtimePlot from the file plot_data.py
from plot_data import DataPlot, RealtimePlot

fig, axes = plt.subplots()
plt.title('Data from TTN console')

data = DataPlot()
dataPlotting= RealtimePlot(axes)

count=0

def bytes_to_decimal(i,d):
    xx = i - 127
    dec = (-d if xx < 0 else d)/100
    return xx + dec

def on_connect(client, userdata, flags, rc):
    client.subscribe('+/devices/+/up'.format(APPEUI))

def on_message(client, userdata, msg):
    j_msg = json.loads(msg.payload.decode('utf-8'))
    dev_eui = j_msg['hardware_serial']

    tmp_hum = base64.b64decode(j_msg['payload_raw'])
    tmp = bytes_to_decimal(*tmp_hum[0:2])
    hum = bytes_to_decimal(*tmp_hum[2:4])
    slot = chr(bytes_to_decimal(*tmp_hum[4:5]))
	
    # print data
    print('---')
    print('tmp:', tmp, ' hum:', hum, 'slot:', slot)
    print('dev eui: ', dev_eui)

    # plot data
    global count
    count+=1
    data.add(count, tmp , hum)
    dataPlotting.plot(data)
    plt.pause(0.001)

# set paho.mqtt callback
ttn_client = mqtt.Client()
ttn_client.on_connect = on_connect
ttn_client.on_message = on_message
ttn_client.username_pw_set(APPID, PSW)
ttn_client.connect("eu.thethings.network", 1883, 60)                    #MQTT port over TLS

try:
    ttn_client.loop_forever()
except KeyboardInterrupt:
    print('disconnect')
    ttn_client.disconnect()