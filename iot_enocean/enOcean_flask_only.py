# =============================================================================================================================
# enOcean wifi sensor connected to iOt systems and webserver example, of course this example only sends to iOt 
# when a user calls the url as this is a test server.
#
# this would be the dockerfile for cloud deployment
# FROM python:3.5.2
# RUN pip install Flask
# RUN pip install serial
# RUN pip install json
# RUN pip install requests
# COPY enocean_flask.py .
# COPY ./app/ ./app
# COPY ./public/ ./public
# CMD ["python", "enocean_flask.py"]
#
# or for production deployment using uWGSI and NGinX on dor example raspberry pi/
# read https://serip39.hatenablog.com/entry/2020/07/06/070000
#
# using pipenv virtual environment
#
# pip pip3 install pipupgrade pip # Upgrade pip to the latest version
# pip pip3 install pipenv # Install pipenv
# export PATH=$PATH:/home/pi/.Add local/bin # PATH
#
# $ mkdir ./flask-test # Create flask-test directory
# $ cd ./flask-test # Move to flask-test directory
# $ pipenv install --python 3.7.3  
#
# pip pipenv install uwsgi # Install uwsgi in a virtual environment
# pip pipenv install flask # Install flask in a virtual environment
# pip pipenv install requests # Install requests in a virtual environment
# pip pipenv install json # Install json in a virtual environment
# pip pipenv install serial # Install serial in a virtual environment
# pip pipenv shell # Run the virtual environment
# 
# to run
# uwsgi --http :8000 --wsgi-file enocean_flask.py --callable app
#
#
# uwsgi configuration file myapp.ini
# [uwsgi]
# module = enocean_flask
# callable = app
# master = true
# processes = 1
# socket = /tmp/uwsgi.sock
# chmod-socket = 666
# vacuum = true
# die-on-term = true
# chdir = /home/pi/flask-test # Specify the directory of the folder
#
# start-up 
# via sysctrl /etc/systemd/system/uwsgi.service
#
# [Unit]
# Description = uWSGI
# After = syslog.target
# [Service]
# ExecStart = /home/pi/.local/bin/uwsgi --ini /home/pi/flask-test/myapp.ini # Specify the path to the file
# Restart=always
# KillSignal=SIGQUIT
# Type=notify
# StandardError=syslog
# NotifyAccess=all
# [Install]
# WantedBy=multi-user.target
#
# =============================================================================================================================
import requests
import json
import serial
from sys import exit
from datetime import datetime
from flask import *

PORT = '/dev/ttyUSB400J'              # USB400J The name of the device to which it is connected
URL = 'http://harvest.soracom.io/'    # SORACOM HarvestのURL
SENSORID1 = '04:01:53:e1'             # ID of the 1st STM-431J
SENSORID2 = '04:00:6f:e5'             # ID of the 2nd STM-431J
s_port = 0                            # serial port number handle
BAUD_RT = 57600                       # serial baud rate

app = Flask(__name__)
#app.register_blueprint(static.app)
        
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
                sensor1_rdy = True

            # Temp sensor1 Send data when the sensor ID matches.
            elif sensorId == SENSORID2 and sensor2_rdy == False:
                val = int(dataList[7],16)
                sensor2_temp = round((255.0-val)/255.0*40.0, 2)
                sensor2_rdy = True

            # Other sensors, ignore ID
            else:
                continue
                
    # return this data for webpage display via flask->uwgsi->nginx         
    if sensor1_rdy and sensor2_rdy:
        return sensor1_temp,sensor2_temp

# This function reads 1 byte of data from the serial port and parses the EnOcean telegram. After analyzing 
# Telegram, data is sent to SORACOM Harvest only if the device ID matches.
def EnoceanSensor(s_port,sens_num):
    sensor1_rdy = False
    cnt = 0
    dataLen = 0
    optLen = 0
    telegraph,headList,dataList,optList = [],[],[],[]
    ready = True

    while True:
        if sensor1_rdy:
            break

        # 1byte read from serial usb
        s_data = s_port.read().encode('hex') # read 1byte

        # Sync-Byte 0x55
        if s_data == '55' and ready: # Telegram start
            # valuable reset
            cnt = 0
            dataLen = 0
            optLen = 0
            ready = False
            telegraph,headList,dataList,optList = [],[],[],[]

        cnt += 1
        telegraph.append(s_data)

        # parse the enocean message
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

            # print the data
            print "========"
            print timestamp
            print 'telegraph...:', ':'.join(telegraph)
            print 'head...', ':'.join(headList)
            print 'data...', ':'.join(dataList), '(length=%d)' % dataLen
            print 'opt ...', ':'.join(optList),  '(length=%d)' % optLen
            print 'sensorID...', sensorId


            # Temp sensor1 calculate the value
            if sensorId == sens_num and sensor1_rdy == False:
                val = int(dataList[7],16)
                sensor1_temp = round((255.0-val)/255.0*40.0, 2)
                # ------ sendSoraComData('temperature01', sensor1_temp)
                sensor1_rdy = True

            # Other sensors, ignore ID
            else:
                continue
                
    if sensor1_rdy:
        return sensor1_temp
        
@app.route("/")
def main():
    return render_template("index.html")
    
@app.route("/get_enocean")
# http://localhost:8000/get_enocean
def get_enocean():
    
    temp1,temp2 = EnoceanAll(s_port) 
#      <!--
#         showvalues.html - simple html representation of two real time values
#      --> 
#    <html>
#    <body>
#    <h1>SENSOR {{ s1 }}!</h1>
#    <h1>TEMPERATURE {{ t1 }}!</h1>
#    <h1>SENSOR {{ s2 }}!</h1>
#    <h1>TEMPERATURE {{ t2 }}!</h1>
#    </body>
#    </html>
    return render_template("showvalues.html", s1=SENSORID1, t1=temp1, s2=SENSORID2, t2=temp2)

@app.route("/get_enocean")
# http://localhost:8000/get_enocean2
def get_enocean2():
    
    temp1,temp2 = EnoceanAll(s_port) 
#      <!--
#         showvalues.html
#      --> 
#    <html>
#    <body>
#    <h1>SENSOR {{ s1 }}!</h1>
#    <h1>TEMPERATURE {{ t1 }}!</h1>
#    <h1>SENSOR {{ s2 }}!</h1>
#    <h1>TEMPERATURE {{ t2 }}!</h1>
#    </body>
#    </html>
    return render_template("i_example.html", t1=temp1, t2=temp2)

@app.route("/sensor_json/<sensor_number>")
# http://localhost:8000/sensor_json/<sensor_id>
def get_enocean_sensor_json(s_port, sensor_id):
    
    temp1 = EnoceanSensor(s_port, sensor_id) 
    # Json
    results = {'sensor_id': sensor_id, 'temperature': temp1}
    # If it is output as is, non-ASCII characters are escaped, so it is false.
    results = json.dumps(results, indent=2, ensure_ascii=False)
    return results
    
if __name__ == "__main__":

    # Open serial port
    try:
        s_port = serial.Serial(PORT, BAUD_RT)
        print('open serial port: %s' % PORT)
    except:
        print('cannot open serial port: %s' % PORT)
        exit(1)
    # run the webserver via flask            
    app.run(debug=True, host='0.0.0.0', port=8000, threaded=True)