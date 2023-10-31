#!/usr/bin/python3
#
# client is publishing enocean sensors to various listed iOt telemetry or email or sms text message or to databases
# subscriber sister application is to read this from the mosquito broker for example if you are doing the mqtt internally on raspberry pi 4
# or you can use one of the SSL server examples to communicate internally or load a database to a server, or choose one from cloud
#
# import neccessary libraries 
import paho.mqtt.client as mqtt
import requests
import json
import base64
import serial
import time
from sys import exit
from datetime import datetime

# enocean
PORT = '/dev/ttyUSB400J'              # The name of the device/port to which the USB400J is connected
SENSORID1 = '04:01:53:e1'             # ID of the 1st STM-431J
SENSORID2 = '04:00:6f:e5'             # ID of the 2nd STM-431J
s_port = 0                            # serial port number handle
BAUD_RT = 57600                       # serial baud rate

# soracom
URL = 'http://harvest.soracom.io/'    # SORACOM HarvestのURL

# mosquito
MTOPIC = "topic/B/"
MQTT_PORT = 1883
KEEP_ALIVE = 60

# beebotte
TOKEN = "[BeebotteTKName]"
HOSTNAME = "mqtt.beebotte.com"
BPORT = 8883
BTOPIC = "MyEnOceanSensors/temperature"                    # beebotte channel/resource
CACERT = "mqtt.beebotte.com.pem"

# ubidots
# pip install ubidots==1.6.6
# sudo apt-get install python-setuptools
UBI_URL=https://industrial.api.ubidots.com/api/v1.6

# machinist
MAPIKEY = "xxxxxx"
MUrl = "https://gw.machinist.iij.jp/endpoint"

# aws
AWS_TOPIC_NAME = "my/temperatures"
AWS_CLIENT="iot-data"

# azure
AZURE_TYPE = "connection_string"          # choose connection_string or provisioning_host
AZURE_CONN_STR="yourConnectionString"
# ensure environment variables are set for your device and IoT Central application credentials
PROVISIONING_HOST = "your host"
ID_SCOPE = "your_scope"
DEVICE_ID = "your_device"
DEVICE_KEY = "your_key"
provisioning_host = PROVISIONING_HOST
id_scope = ID_SCOPE
registration_id = DEVICE_ID
symmetric_key = DEVICE_KEY

# yandex
YPORT=8443
YHOST="rc1c-xxxx123456.mdb.yandexcloud.net"
YCERTNAME="your_cert_name"	
YUrl=r'https://api.iot.yandex.net/v1.0/devices/actions'	
YAPI_KEY="xxxxxx"

# time
import datetime
import pytz
MY_TZ='Europe/Moscow'

# smtp
SMTP_SERVER = "smtp.gmail.com"
port = 587
SMTP_NO_SSL=1 # 1 use above data without SSL otherwise use gmail and SSL

# ssl server with tls_set
SSLHOST, SSLPORT = "127.0.0.1", 12345
# ssl23
SSL_URL = '127.0.0.1'
SSL_PORT = 10023

# cloud MQTT
CHOSTNAME = "driver.cloudmqtt.com"
CPORT = 28607
CTOPIC = "pi/sub2"                    
CCACERT = "/etc/ssl/certs/ca-certificates.crt"

#define a global MQTT Topic which will be set upon choices
GTOPIC=" "

# GCS Spreadsheet
# set when you set-up sheet
GSS_TEMP_KEY = os.environ['GSS_TEMP_KEY']

# ambient
#
channelID = 100
writeKey = 'writeKey'
        
# ------------ here list the choices and options for iOt or monitoring -----------------      
TELEM_CHOICES=[ "soracom", "beebotte", "mosquito", "ubidots", "machinist", "aws", "azure", "yandex", "twillio", "smtp_email", "ssl_tls_server", "ssl_23_server", "cloud_mqtt", "gcs_blob", "splunk", "gcs_spread", "ambient", "influxdb", "redis", "mongo", "mysql", "sybase" ]
SORACOM=0
BEEBOTTE=1
MOSQUITO=2
UBIDOTS=3
MACHINIST=4
AWS=5
AZURE=6
YANDEX=7
TWILLIO=8
SMTP_EMAIL=9
SSL_TLS_SERVER=10
SSL_23_SERVER=11
CLOUD_MQTT=12
BLOB_GCS=13
SPLUNK=14
SPREAD_GCS=15
AMBIENT=16
INFLUXDB=17
REDIS=18
MONGO=19
MYSQL=20
SYBASE=21
# ============= make your choice of cloud service here from list above ================== 
MY_CURRENT_TELEM=TELEM_CHOICES[SORACOM]

# if you want to use AES with SSLv23
AES_ENCRYPT=0

# Broker processes when connected to
def on_connect(client, userdata, flag, rc):
    print("Connect Broker:" + str(rc))
    client.subscribe(GTOPIC)

# Broker processes when disconnected 
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("disconnect broker")

# publish processes when publish done
def on_publish(client, userdata, mid):
    print("publish Done")

# ============================== mongo DB Classes *perhaps make seperate include and import when option active) =======================================
import pandas as pd
import pymongo

# define globals here you could default them as parameters to class if preferred

# remote / online
USER_NAME = "1_MongoDB Atlas"
CLUSTER_NAME = "1_MongoDB Atlas Cluster"
PASSWORD = "DB_pass1"
# database structure
DB_NAME = "1_MongoDB Atlas DB"                      # online or offline
COLLECTION_NAME="enocean_temps"                     # name of your collection
   
class Mongo_Database_Class(object):

    def __init__(self, dbName, collectionName, rem="local"):
        self.clint = self.connect_to_mongodb(rem)
        self.db = self.clint[dbName]
        self.collection = self.db.get_collection(collectionName)

    def connect_to_mongodb(rem="online"):
        """ connect to the mongo db either online or local """
        if rem == "online":
            client = pymongo.MongoClient(f"mongodb+srv://{USER_NAME}:{PASSWORD}@{CLUSTER_NAME}.jipvx.mongodb.net/{DB_NAME}?retryWrites=true&w=majority")
        else:
            client = pymongo.MongoClient()	
        return client
    
    def add_record(self,sd1,st1,sd2,st2):
        """ adds the record to the open mongo database """
        post = {
            'sensor1_desc1': sd1,
            'sensor1_temp1': st1,
            'sensor1_desc2': sd2,
            'sensor1_temp2': st2,
            'created_at': datetime.datetime.now()
        }
        return self.collection.insert_one(post)
		
    def get_collection_to_df(self, filter=None, projection=None):
        """ get the collection and return a pandas data frame """
        #  collection = self.clint[dbName][collectionName] 
        cursor = self.collection.find(filter=None, projection=None)
        df = pd.DataFrame(list(cursor))	
	    return df
		
    def insert_many(self, documents):
        """ insert many records example :- mongo.insert_many([{'name':'mark','salary':40},{'name':'andy','salary':500000}]) """
        return self.collection.insert_many(documents)
		
    def find(self, projection=None,filter=None, sort=None):
        """ print collection """
        return self.collection.find(projection=projection,filter=filter,sort=sort)

# =============================================  use mySQL database  ============================================================
#
MY_SQL_DB='python_db'
MY_SQL_TAB="Enocean_Temperatures"
MYSQLLIB = "MYSQLDB"

    
# This function reads 1 byte of data from the serial port and parses the EnOcean telegram. After analyzing 
# Telegram, data is sent to the chosen iOt (telemetry/database) system
def Enocean2Telemetry(s_port, telem_opt):

    # SORACOM Harvest。
    def sendDataSoraCom(string1, temp_data1, string2, temp_data2):
        headers = {'content-type': 'application/json'}
        payload = {string1:temp_data1}
        print payload
        try:
            req = requests.post(URL, data=json.dumps(payload), headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
        headers = {'content-type': 'application/json'}
        payload = {string2:temp_data2}
        print payload
        try:
            req = requests.post(URL, data=json.dumps(payload), headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err) 

    # SPLUNK
    def sendDataSplunk(string1, temp_data1, string2, temp_data2):
        splunk_config = {
            "wifi": {
                "ssid": "WIFI SID",
                "password": "WIFI YOUR_PASSWD"
            },
            "hec": {
                "url": "http://SplunkMY_IDIP:8088/services/collector/event",
                "token": "HTTP Event Collector YOUR_TOKEN",
                "hostname": "MySplunkServer"
            }
        }
        url = splunk_config['hec']['url']
        token = splunk_config['hec']['token']
        host = splunk_config['hec']['hostname']
        data_val = { "values" : [ {"metric_name" : string1, "sensor" : SENSORID1, "_value" : temp_data1}, {"metric_name" : string2 , "sensor" : SENSORID2, "_value" : temp_data2} ] }
        source=None
        index=None
        timestamp=round(datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())
        
        u'Send measured data to Splunk HEC'
        headers = {'Authorization': 'Splunk {}'.format(token)}
        payload = []
        #for data in values:
        for cnt in range(len(data_val['values'])):
            p = {
                "event": "metric",
                "source": source if source is not None else data_val['values'][int(cnt)]['sensor'],
                "host": host,
                "fields": {
                    "metric_name": data_val['values'][int(cnt)]['metric_name'],
                    "sensor": data_val['values'][int(cnt)]['sensor'],
                    "_value": data_val['values'][int(cnt)]['_value'],
                }
            }
            if index is not None:
                p['index'] = index
            if timestamp is not None:
                p['time'] = timestamp
            payload.append(p)

        print("[DEBUG] HEC payload", payload)
        try:
            res = requests.post(url, headers=headers, data=json.dumps(payload))
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
    
    # MOSQUITO
    def sendDataMosquito(string1, enO_temp_value1, string2, enO_temp_value2):
        msg = string1 + enO_temp_value1
        client.publish(MTOPIC,msg)
        msg = string2 + enO_temp_value2
        client.publish(MTOPIC,msg)
        client.disconnect()
        
    # BEEBOTTE
    def sendDataBeebotte(string1, enO_temp_value1, string2, enO_temp_value2):
        msg = string1 + enO_temp_value1
        client.publish(BTOPIC,msg)
        msg = string2 + enO_temp_value2
        client.publish(BTOPIC,msg)
        client.disconnect()

    # CLOUD MQTT
    def sendDataCloudMqtt(string1, enO_temp_value1, string2, enO_temp_value2):
        msg = string1 + enO_temp_value1
        client.publish(CTOPIC,msg,0)
        msg = string2 + enO_temp_value2
        client.publish(CTOPIC,msg,0)
        client.disconnect()

    # MONGO DATABASE
    def addMongoRecord(string1, enO_temp_value1, string2, enO_temp_value2):
        rest = mongo_obj.add_record(string1, enO_temp_value1, string2, enO_temp_value2)
        print(rest)
    
    # AMBIENT
    def sendDataAmbient(string1, enO_temp_value1, string2, enO_temp_value2):
        ret = am.send({'created': datetime.datetime.now(), string1: enO_temp_value1, string2: enO_temp_value2})
        # not sure if we have it ?? am.disconnect()
        
    # AWS
    def sendDataAws(string1, enO_temp_value1, string2, enO_temp_value2):
        data = {
            "temp1": enO_temp_value1,
            "temp1_desc": string1,
            "temp2": enO_temp_value2,
            "temp2_desc": string2,
        }
        json_data = bytes(json.dumps(data),"utf-8")
        print(f"sending to aws -> {sys.getsizeof(json_data)} bytes") 
        client = boto3.client(AWS_CLIENT)
        client.publish(topic=AWS_TOPIC_NAME, payload=json_data) 
        
    # UBIDOTS
    def sendDataUbiDots(string1, temp_data1, string2, temp_data2): 
       try:
           my_variable = api.get_variable('56799cf1231b28459f976417')
       except UbidotsError400 as e:
           print("General Description: %s; and the detail: %s" % (e.message, e.detail))
       except UbidotsForbiddenError as e:
           print("For some reason my account does not have permission to read this variable")
           print("General Description: %s; and the detail: %s" % (e.message, e.detail))
        payload = {string1:int(temp_data1)}
        print payload
        new_value = my_variable.save_value(payload)
        try:
            my_variable2 = api.get_variable('56799cf1031b28459f976718')
        except UbidotsError400 as e:
            print("General Description: %s; and the detail: %s" % (e.message, e.detail))
        except UbidotsForbiddenError as e:
            print("For some reason my account does not have permission to read this variable")
            print("General Description: %s; and the detail: %s" % (e.message, e.detail))
        payload = {string2:int(temp_data2)}
        print payload
        new_value = my_variable2.save_value(payload)

    # AZURE
    def sendDataAzure(descrip1,temp_data1,descrip2,temp_data2):
        print("Sending telemetry for temperature to azure")
        temperature_msg = {descrip1 : temp_data1}
        msg = Message(json.dumps(temperature_msg))
        msg.content_encoding = "utf-8"
        msg.content_type = "application/json"
        print("Send message : ",msg)
        device_client.send_message(msg)
        temperature_msg1 = {descrip2 : temp_data2}
        msg1 = Message(json.dumps(temperature_msg1))
        msg1.content_encoding = "utf-8"
        msg1.content_type = "application/json"
        print("Send message : ",msg1)
        device_client.send_message(msg1)
        device_client.disconnect()
        device_client.shutdown()
        
    # sub functions to store certs in azure key vaalts and may be useful for management of certs
    #
    def createCertAzure():
        from azure.identity import DefaultAzureCredential
        from azure.keyvault.certificates import CertificateClient, CertificatePolicy
        credential = DefaultAzureCredential()
        certificate_client = CertificateClient(vault_url="https://my-key-vault.vault.azure.net/", credential=credential)
        create_certificate_poller = certificate_client.begin_create_certificate(certificate_name="cert-name", policy=CertificatePolicy.get_default())
        print(create_certificate_poller.result())
    def getLatestCertAzure():
        from azure.identity import DefaultAzureCredential
        from azure.keyvault.certificates import CertificateClient
        credential = DefaultAzureCredential()
        certificate_client = CertificateClient(vault_url="https://my-key-vault.vault.azure.net/", credential=credential)
        certificate = certificate_client.get_certificate("cert-name")
        print(certificate.name)
        print(certificate.properties.version)
        print(certificate.policy.issuer_name)
    def updateCertAzure():
        from azure.identity import DefaultAzureCredential
        from azure.keyvault.certificates import CertificateClient
        credential = DefaultAzureCredential()
        certificate_client = CertificateClient(vault_url="https://my-key-vault.vault.azure.net/", credential=credential)
        # we will now disable the certificate for further use
        updated_certificate= certificate_client.update_certificate_properties(certificate_name="cert-name", enabled=False)
    def deleteCertAzure():
        from azure.identity import DefaultAzureCredential
        from azure.keyvault.certificates import CertificateClient
        credential = DefaultAzureCredential()
        certificate_client = CertificateClient(vault_url="https://my-key-vault.vault.azure.net/", credential=credential)
        deleted_certificate_poller = certificate_client.begin_delete_certificate("cert-name")
        deleted_certificate = deleted_certificate_poller.result()
        print(deleted_certificate.name)
        print(deleted_certificate.deleted_on)
        print(updated_certificate.name)
        print(updated_certificate.properties.enabled)
    def listCertAzure():
        from azure.identity import DefaultAzureCredential
        from azure.keyvault.certificates import CertificateClient
        credential = DefaultAzureCredential()
        certificate_client = CertificateClient(vault_url="https://my-key-vault.vault.azure.net/", credential=credential)
        certificates = certificate_client.list_properties_of_certificates()

        for certificate in certificates:
        # this list doesn't include versions of the certificates
            print(certificate.name)
    
    # YANDEX
    def makeYandexTable(table_name):

        statement = f"""CREATE TABLE {table_name} (
        ( telemetry_timestamp timestamp,
          device_nm varchar(200),
          payload varchar(2000)
        );
        """
        return statement

    def sendCreateYandexTable(my_query):
	
	timestamp = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp()  # set the timezone as you wish for your location
        event_ts=round(timestamp)
        insert=  my_query	
	
        data = {
            'ca': YCERTNAME,
            'path': '/myHome/', {
                'database': 'pxc_cloud_db',
                'query': insert,
		},
            'port': YPORT,
            'hostname': YHOST,
        }
		
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Api-Key {YAPI_KEY}",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post(YUrl, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
 
    # YANDEX 
    def sendDataYandex(descrip1,temp_data1,descrip2,temp_data2):
	
        #msg = json.loads(payload_json)
        #msg_str = json.dumps(msg)
	msg_str = temp_data1
	timestamp = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp()  # set the timezone as you wish for your location
        event_ts=round(timestamp)
        insert=  f"""INSERT INTO pxc_cloud_db.timeseries_example (telemetry_timestamp , device_nm , payload) VALUES ('{event_ts}','{descrip1}', '{msg_str}')"""	
	
        data = {
            'ca': YCERTNAME,

            'path': '/myHome/', {
                'database': 'pxc_cloud_db',
                'query': insert,
		},
            'port': YPORT,
            'hostname': YHOST,
        }
		
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Api-Key {YAPI_KEY}",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post(YUrl, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

	msg_str = temp_data2
	timestamp = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp()  # set the timezone as you wish for your location
        event_ts=round(timestamp)
        insert=  f"""INSERT INTO pxc_cloud_db.timeseries_example (telemetry_timestamp , device_nm , payload) VALUES ('{event_ts}','{descrip2}', '{msg_str}')"""	
	
        data = {
            'ca': YCERTNAME,

            'path': '/myHome/', {
                'database': 'pxc_cloud_db',
                'query': insert,
		},
            'port': YPORT,
            'hostname': YHOST,
        }
		
        headers = {
            // 'X-ClickHouse-User': user,
            // 'X-ClickHouse-Key': password,
            "Content-Type": "application/json",
            "Authorization": f"Api-Key {YAPI_KEY}",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post(YUrl, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)	

    # GCS BLOB UPLOAD
    def uploadBlobGcs(descrip1,temp_data1,descrip2,temp_data2):
        """Writes json to file then uploads csv to the bucket on GCS """
        import csv
        from google.cloud import storage
        destination_blob_name = "your_data_1"
        source_file_name = f'/tmp/{destination_blob_name}'
        bucket_name = 'yourdataalarms'

        fieldnames = ['sensor1_desc', 'sensor1_temp', 'sensor2_desc', 'sensor2_temp', 'timestamp']
        json_obj = {}
        json_obj['sensor1_desc'] = descrip1
        json_obj['sensor1_temp'] = temp_data1
        json_obj['sensor2_desc'] = descrip2
        json_obj['sensor2_temp'] = temp_data2
        json_obj['timestamp'] = round(timestamp = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())
        json_obj['timestamp'] = datetime.datetime.utcfromtimestamp(int(json_obj['timestamp'])//1000).strftime('%Y-%m-%d %H:%M:%S')
        with open(source_file_name, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(json_obj)
        storage_client = storage.Client()
        bucket = storage_client.bucket(bucket_name)
        blob = bucket.blob(destination_blob_name)
        blob.upload_from_filename(source_file_name)
        print("File {} uploaded to {}.".format(source_file_name, destination_blob_name))

    # GCS Spreadsheet
    # get the worksheet from the cloud service
    def get_gss_worksheet(gss_name, gss_sheet_name):
        # If you do not write two APIs, you must continue to issue refresh tokens every 3600 seconds
        #scope = ['https://spreadsheets.google.com/feeds','https://www.googleapis.com/auth/drive']
        scope = ['https://www.googleapis.com/auth/spreadsheets','https://www.googleapis.com/auth/drive']
        c = ServiceAccountCredentials.from_json_keyfile_name('../gss_credential.json', scope)

        # authorise using vred file
        gs = gspread.authorize(c)
    
        # set the auth key
        if gss_name == "Temp_Data":
            spreadsheet_key = GSS_TEMP_KEY

        # open the worksheet and return its handle for further use
        worksheet = gs.open_by_key(spreadsheet_key).worksheet(gss_sheet_name)

        return worksheet

    def init_gcs_worksheet():
        import os
        import gspread
        from oauth2client.service_account import ServiceAccountCredentials
        # get the worksheet from google cloud service
        #scope = ['https://spreadsheets.google.com/feeds','https://www.googleapis.com/auth/drive']
        scope = ['https://www.googleapis.com/auth/spreadsheets','https://www.googleapis.com/auth/drive']
        c = ServiceAccountCredentials.from_json_keyfile_name('../gss_credential.json', scope)

        # authorise using vred file
        gs = gspread.authorize(c)
    
        # set the auth key
        spreadsheet_key = GSS_TEMP_KEY

        # open the worksheet and return its handle for further use
        worksheets = gs.open_by_key(spreadsheet_key)
        wks = worksheets.add_worksheet(title="sheet_1", rows='200', cols='8')

        # cell A1 contains the current row being used
        # update the worksheet
        wks.update_acell("A1", 3)           # default starts at row 3
        wks.update_acell("A2", "sensor 1 description")           
        wks.update_acell("B2", "sensor 1 temp DegC")   
        wks.update_acell("C2", "sensor 1 description")   
        wks.update_acell("D2", "sensor 2 temp DegC")   
        wks.update_acell("E2", "timestamp") 

    
    def put_data_on_gcs_worksheet(desc1,v1,desc2,v2):
        # get the worksheet from google cloud service
        worksheet = get_gss_worksheet(gss_name='Temp_Data', gss_sheet_name='sheet_1')

        # cell A1 contains the current row being used
        value = worksheet.acell("A1").value
        print(value)
        if ((value <= 0) || (value >= 200)):
            value = 3                              # start minus the row 1 = saved sursor and 2 which is = titles with overwrite at max row 200
        else:
            value = int(value) + 1

        # update the worksheet
        worksheet.update_acell("A1", value)
    
        # now make the new cells the A1 value contains the last write cursor
        Begin_End=str("A"+value+":"+"E"+value)
        ts = datetime.datetime.now(pytz.timezone(MY_TZ))
        list_data = [desc1, v1, desc2, v2, ts]
        worksheet.update(Begin_End, list_data)

    # INFLUX DATABASE
    def influx_json(field,nowtime=''):
	    json_body = [
		    {
			    'measurement': measurement,
			    'tags': tags,
			    'fields': field
			    'time': nowtime
		    }
	    ]
	    client.write_points(json_body)
	    print(json_body)

    def put_data_influx(desc1,v1,desc2,v2):
        ts = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp()
        field = {desc1: v1}
        influx_json(field,ts) 
        field = {desc2: v2}
        influx_json(field,ts)  

    # REDIS
    def put_data_redis(desc1,v1,desc2,v2):
        ts = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp()
        redis_client.hmset('enOcean Temperatures', { desc1 : v1, desc2 : v2, 'time' :  ts }) 

    def get_data_redis(hashkey='enOcean Temperatures'):
        return redis_client.hgetall(hashkey) 
        
    # MACHINIST
    def sendDataMachinist(descrip1,temp_data1,descrip2,temp_data2):
        data = {
            "agent": "GPIO",
            "metrics": [
                {
                    "name": descrip1,
                    "namespace": "STM-431J Temp Sensor",
                    "data_point": {
                        "value": temp_data1
                    }
                }
            ]
        }
 
        headers = {
            "Content-Type": "application/json",
            "Authorization": "Bearer " + MAPIKEY,
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post(MUrl, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

        data2 = {
            "agent": "GPIO",
            "metrics": [
                {
                    "name": descrip2,
                    "namespace": "STM-431J Temp Sensor",
                    "data_point": {
                        "value": temp_data2
                    }
                }
            ]
        }
        senddatajson = json.dumps(data2).encode("ascii")
        try:
            req = requests.post(MUrl, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    # TWILIO SMS 
    def sendTwilioSMS(descrip1,temp_data1,descrip2,temp_data2):
        from twilio.rest import Client
        tw_account_sid = "Account SID"
        tw_auth_teoken = "AUTHTOKEN"
        tw_to_number = "Destination phone number"
        tw_from_number = "Twilio phone number for the trial obtained"
        client = Client(account_sid, auth_teoken)
	ts = round(datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())  # set the timezone as you wish for your location
        bodytext=" got the data at " + ts + " " + descrip1 + " : " + str(temp_data1) + " " + descrip2 + " : " + str(temp_data2)
        message = client.messages.create(body=bodytext,from_=from_number,to=to_number)
        print(message.sid)   
        #return message.sid        

    # SMTP EMAIL SERVER 
    # to check library classes import inspect then
    # inspect.getmro(MIMEText)
    def sendSMTPemail(descrip1,temp_data1,descrip2,temp_data2):
        import smtplib
        #from email.mime.multipart import MIMEMultipart ---> you can uncomment insted of message = MIMEText(bodytext, "plain", 'utf-8') but i think its okay
        from email.mime.text import MIMEText
        try:
            if SMTP_NO_SSL == 1:
                server = smtplib.SMTP(SMTP_SERVER, port)
            else:
                server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
            server.starttls()
            login_address ="Enter the sender's email address"
            login_password ="Enter password"
            server.login(login_address, login_password)
            #message = MIMEMultipart()
            ts = round(datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())  # set the timezone as you wish for your location
            bodytext=" got the data at " + ts + " " + descrip1 + " : " + str(temp_data1) + " " + descrip2 + " : " + str(temp_data2)
            message = MIMEText(bodytext, "plain", 'utf-8')
            #text = MIMEText(bodytext)
            #message.attach(text)
            #
            # alternative
            # from email.message import EmailMessage
            # message = EmailMessage()
            # message.set_content(bodytext)
            message['Subject']="Email Subject"
            message['From']="your eamil address@gmail.com"
            message['To']="recipient email address@gmail.com"
            server.send_message(message) 
            server.quit()
        except requests.exceptions.RequestException as err:
            print ("SMTP connect error : ",err)

    # ------------- Ciphers / Cryptography -----------------
    # AES_GCM
    def aes_gcm_encrypt(key,iv,text):
        cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
        ciphertext, mac = cipher.encrypt_and_digest(text)
        return ciphertext, mac
    def aes_gcm_decrypt(key,iv,ciphertext,mac):
        plaintext = 0
        cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
        try:
            plaintext = cipher.decrypt_and_verify(ciphertext,mac)
        except (ValueError, KeyError):
            print("Incorrect decryption")
        return plaintext

    # CHACHA - POLY1305
    def chacha20_poly1305_encrypt(key,byte_data):
        from Crypto.Cipher import ChaCha20_Poly1305 as cha
        chacha20 = cha.new(key=key)
        cipher_text, mac = chacha20.encrypt_and_digest(byte_data)
        nonce = chacha20.nonce
        print('cipher_text: ', cipher_text)
        print('mac: ', mac) 
        return cipher_text,nonce
    def chacha20_poly1305_decrypt(key,nonce,mac,cipher_text):
        from Crypto.Cipher import ChaCha20_Poly1305 as cha
        decrypt_data="no data"
        try:
            chacha_poly = cha.new(key=key, nonce=nonce)
            decrypt_data = chacha_poly.decrypt_and_verify(cipher_text, mac)
            print('The message is: ', decrypt_data)
        except:
            print("The message couldn't be verified")  
        return decrypt_data       
        
    # RSA_cryptography (Rivest-Shamir-Adleman)
    def RSA_encrypt(message, ):
        from Crypto.Cipher import PKCS1_OAEP
        from Crypto.PublicKey import RSA
        from binascii import hexlify
        #Generating private key (RsaKey object) of key length of 1024 bits
        private_key = RSA.generate(1024)
        #Generating the public key (RsaKey object) from the private key
        public_key = private_key.publickey()
        print(type(private_key), type(public_key))
        #Converting the RsaKey objects to string 
        private_pem = private_key.export_key().decode()
        public_pem = public_key.export_key().decode()
        print(type(private_pem), type(public_pem))
        #Writing down the private and public keys to 'pem' files
        with open('private_pem.pem', 'w') as pr:
            pr.write(private_pem)
        with open('public_pem.pem', 'w') as pu:
            pu.write(public_pem)
        #Importing keys from files, converting it into the RsaKey object   
        pr_key = RSA.import_key(open('private_pem.pem', 'r').read())
        pu_key = RSA.import_key(open('public_pem.pem', 'r').read())
        print(type(pr_key), type(pu_key))
        #Instantiating PKCS1_OAEP object with the public key for encryption
        cipher = PKCS1_OAEP.new(key=pu_key)
        #Encrypting the message with the PKCS1_OAEP object
        cipher_text = cipher.encrypt(message.encode('UTF-8'))
        print(cipher_text)
    def RSA_decrypt(cipher_text,pr_key):
        from Crypto.Cipher import PKCS1_OAEP
        #Instantiating PKCS1_OAEP object with the private key for decryption
        decrypt = PKCS1_OAEP.new(key=pr_key)
        #Decrypting the message with the PKCS1_OAEP object
        decrypted_message = decrypt.decrypt(cipher_text)
        print(decrypted_message)        
    # common packers
    #
    # messagepack
    def msgPackPack(descrip1,temp_data1,descrip2,temp_data2):
        import msgpack
        json_obj = {}
        json_obj['sensor1_desc'] = descrip1
        json_obj['sensor1_temp'] = temp_data1
        json_obj['sensor2_desc'] = descrip2
        json_obj['sensor2_temp'] = temp_data2
        json_obj['timestamp'] = round(timestamp = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())
        msgpack_data = msgpack.packb(json.dumps(json_obj, ensure_ascii=False))       
        return msgpack_data
    def msgPackUnPack(msgpck_str):
        import msgpack
        msgs=[]
        for msg in msgpack.Unpacker(msgpck_str):
            print msg 
            msgs.append(msg)
        return msgs            
    # cbor
    def cborPack(descrip1,temp_data1,descrip2,temp_data2):
        import cbor
        json_obj = {}
        json_obj['sensor1_desc'] = descrip1
        json_obj['sensor1_temp'] = temp_data1
        json_obj['sensor2_desc'] = descrip2
        json_obj['sensor2_temp'] = temp_data2
        json_obj['timestamp'] = round(timestamp = datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())
        cbor_data = cbor.load(json.dumps(json_obj, ensure_ascii=False))       
        return cbor_data
    def cborUnPack(cbor_str):
        import cbor
        msgs=[]
        for msg in cbor.dump(cbor_str):
            print msg 
            msgs.append(msg)
        return msgs 
        
    # SSL TLS Client connection to your own SSL server which echos the response back in reply 
    def sendTLSClient(descrip1,temp_data1,descrip2,temp_data2):
        import socket, ssl
        # SSL with TLS
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)   # refer to the options https://www.pyopenssl.org/en/21.0.0/api/ssl.html
        context.options |= ssl.OP_NO_TLSv1_3        # OP_NO_TLSv1_3 = Disable TLS1.3 PROTOCOL_TLS_CLIENT = tls client, PROTOCOL_TLS_SERVER = TLS Server
        #context.options |= ssl.OP_SINGLE_ECDH_USE    example a new key will always be created when using ephemeral (Elliptic curve) Diffie-Hellman. 
        context.check_hostname = False              # do not check hostname
        context.load_verify_locations('cert.pem')   # specify the certificate
        context.set_ciphers('kRSA')                 # specify the cipher suite
 
        # Create a socket and wrap it in an SSL socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ssock = context.wrap_socket(sock)
 
        # Connect to the server and send data
        ssock.connect((SSLHOST,SSLPORT))
		ts = round(datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())  # set the timezone as you wish for your location
        msgdata = " got the data at " + ts + " " + descrip1 + " : " + str(temp_data1) + " " + descrip2 + " : " + str(temp_data2)
        ssock.sendall(bytes(msgdata  + "\n", "utf-8"))
 
        # Receive data from the server and exit
        received = str(ssock.recv(1024), "utf-8")
        ssock.close()
 
        print("Sent:     " + msgdata)
        print("Received: " + received)

    # SSL TLS Client connection to your own SSL server which echos the response back in reply    
    def sendSSL23Client(descrip1,temp_data1,descrip2,temp_data2):
        import ssl
        import socket
        import pprint       
        context = ssl.create_default_context()
        context = ssl.SSLContext(ssl.PROTOCOL_SSLv23)
        context.verify_mode = ssl.CERT_NONE 
        context.check_hostname = False
        conn = context.wrap_socket(socket.socket(socket.AF_INET), server_hostname=URL)
        conn.connect((SSL_URL, SSL_PORT))
        cert = conn.getpeercert()
        pprint.pprint(cert)
        ts = round(datetime.datetime.now(pytz.timezone(MY_TZ)).timestamp())  # set the timezone as you wish for your location
        msgdata = b" got the data at " + str(ts).encode('UTF-8') + b" " + descrip1.encode('UTF-8') + b" : " + str(temp_data1).encode('UTF-8') + b" " + descrip2.encode('UTF-8') + b" : " + str(temp_data2).encode('UTF-8') + b"\r\n\r\n"
        #conn.sendall(b"HEAD / HTTP/1.0\r\nHost: linuxfr.org\r\n\r\n")
        if AES_ENCRYPT == 1:
            from Crypto.Cipher import AES
            from Crypto.Random import get_random_bytes
            key = get_random_bytes(16)
            nonce = get_random_bytes(12)
            ciphertext, mac = aes_gcm_encrypt(key,nonce,msgdata)
            conn.sendall(ciphertext)
            indata = conn.recv(1024).split(b"\r\n"))
            pprint.pprint(aes_gcm_decrypt(key,nonce,indata,mac))
        else:
            conn.sendall(msgdata)
            indat = conn.recv(1024).split(b"\r\n"))
            pprint.pprint(indat)
        conn.close()

    # MySQL database
    def putValueMySQL(descrip1, temp_data1, descrip2, temp_data2):

        # insert values
        cursor.execute("""INSERT INTO MY_SQL_TAB (description, value, timestamp)
            VALUES ('{d1}', '{t1}', '{ts}' ),
            ('{d2}', '{t2}', '{ts}' ),
            """.format(d1=descrip1,t1=temp_data1,d2=descrip2,t2=temp_data2, ts=datetime.datetime.now()))

        # fetch the data
        query="SELECT * FROM "+MY_SQL_TAB
        cursor.execute(query)
        for row in cursor:
            print(row)

        # commit the changes
        connection.commit()

        # connection close
        connection.close()

        # set pointers to nulls
        cursor = None
        connection = None

    def mySQLConnect():
        # choose the library not sure which is best !
        if MYSQLLIB == "MYSQLDB":
            # MySQLdb
            # sudo apt-get install python-dev default-libmysqlclient-dev ; sudo apt-get install python3-dev ; pip install mysqlclient
            import MySQLdb
    
            # connect using the MySQLdb library
            connection = MySQLdb.connect(
                host='localhost',
                user='root',
                passwd='mypasswd',
                db=MY_SQL_DB,
                charset='utf8'
            )
        else:
            # alternatively pip install mysql-connector-python
            import mysql.connector

            # connect using mysql.connector
            connection=mysql.connector.connect(host="192.168.56.102", user="user", password="xxxxxxxxxx", database=MY_SQL_DB, charset='utf8' )

        if not connection.is_connected():
            raise Exception("MySQL- database will not connect")
            sys.exit(6) 
    
        # now get the cursor
        cursor = connection.cursor()

        # select the db
        query="USE "+MY_SQL_DB
        cursor.execute(query)

        # drop table if already exist
        query="DROP TABLE IF EXISTS "+MY_SQL_TAB
        cursor.execute(query)

        # create table
        cursor.execute("""CREATE TABLE {table_name}(
            id INT(11) AUTO_INCREMENT NOT NULL, 
            description VARCHAR(30) NOT NULL COLLATE utf8mb4_unicode_ci, 
            value FLOAT NOT NULL,
            timestamp VARCHAR(30) NOT NULL COLLATE utf8mb4_unicode_ci,
            PRIMARY KEY (id)
            )""".format(table_name=MY_SQL_TAB))

        # commit the changes
        connection.commit()


    # sybase database
    #
    SY_SQL_TAB="Temperatures"
    def putValueSybase(descrip1, temp_data1, descrip2, temp_data2):

        # insert values
        sql = """INSERT INTO SY_SQL_TAB (description, value, timestamp)
            VALUES ('{d1}', '{t1}', '{ts}' ),
            ('{d2}', '{t2}', '{ts}' ),
            """.format(d1=descrip1,t1=temp_data1,d2=descrip2,t2=temp_data2, ts=datetime.datetime.now())
        table1 = etl.fromdb(cnxn,sql)
        
        # fetch the data
        query="SELECT * FROM "+SY_SQL_TAB
        table2 = etl.fromdb(cnxn,query)
        df = pd.DataFrame(table2)
        print(df)

        # connection close
        cnxn.close()


    def sySQLConnect():

        import petl as etl 
        import pandas as pd
        import cdata.sybase as mod 
        cnxn = mod.connect("User=myuser;Password=mypassword;Server=localhost;Database=mydatabase;Charset=iso_1;")
        return cnxn
        
    # Choose the iOt you want to use according to the define in top section        
    if telem_opt == "soracom":
        sendData=sendDataSoraCom
    elif telem_opt == "beebotte":
        client = mqtt.Client()
        client.username_pw_set("token:%s"%TOKEN)
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_publish = on_publish
        client.tls_set(CACERT)
        client.connect(HOSTNAME, port=BPORT, keepalive=KEEP_ALIVE)
        sendData=sendDataBeebotte
        GTOPIC=BTOPIC
    elif telem_opt == "ubidots":
        from ubidots import ApiClient      
        api = ApiClient(token="4b00-xxxxxxxxxxxxxxxxxxx", base_url="http://yourcompanyname.api.ubidots.com/api/v1.6/")       
        sendData=sendDataUbiDots
    elif telem_opt == "machinist":
        sendData=sendDataMachinist   
    elif telem_opt == "aws":
        import boto3
        sendData=sendDataAws    
    elif telem_opt == "yandex":
        # un comment if you didnt already make the table 
        # makeTableQuery=makeYandexTable("pxc_cloud_db.timeseries_example")
        # sendCreateYandexTable(makeTableQuery)
        sendData=sendDataYandex         
    elif telem_opt == "azure":
        if AZURE_TYPE == "connection_string" :
            from azure.iot.device.aio import IoTHubDeviceClient
            from azure.iot.device import Message
            device_client = IoTHubDeviceClient.create_from_connection_string(AZURE_CONN_STR)
        else:
            from azure.iot.device.aio import ProvisioningDeviceClient
            from azure.iot.device.aio import IoTHubDeviceClient
            from azure.iot.device import Message
            provisioning_device_client = ProvisioningDeviceClient.create_from_symmetric_key(
                provisioning_host=provisioning_host,
                registration_id=registration_id,
                id_scope=id_scope,
                symmetric_key=symmetric_key,
            )
            registration_result = provisioning_device_client.register()
            print("The complete registration result is")
            print(registration_result.registration_state)
            if registration_result.status == "assigned":
               print("Your device has been provisioned. It will now begin sending telemetry.")
               device_client = IoTHubDeviceClient.create_from_symmetric_key(
                   symmetric_key=symmetric_key,
                   hostname=registration_result.registration_state.assigned_hub,
                   device_id=registration_result.registration_state.device_id,
               )            
        device_client.connect()
        sendData=sendDataAzure  
    elif telem_opt == "twillio":
        sendData=sendTwilioSMS
    elif telem_opt == "smtp_email":
        sendData=sendSMTPemail
    elif telem_opt == "ssl_tls_server":
        sendData=sendTLSClient
    elif telem_opt == "ssl_23_server":
        sendData=sendSSL23Client
    elif telem_opt == "gcs_blob":
        sendData=uploadBlobGcs
    elif telem_opt == "mongo":
        mongo_obj = Mongo_Database_Class(DB_NAME, COLLECTION_NAME, "online")     # make database class instance in this example it is online
        sendData=addMongoRecord
    elif telem_opt == "sybase":
        cnxn = sySQLConnect()
        sendData=putValueSybase
    elif telem_opt = "cloud_mqtt":            
        client = mqtt.Client(protocol=mqtt.MQTTv311)
        client.tls_set(CCACERT)
        client.username_pw_set("aircampro", "air987")
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_publish = on_publish
        client.connect(CHOSTNAME, CPORT, KEEP_ALIVE)    
        sendData=sendDataCloudMqtt
        GTOPIC=CTOPIC
    elif telem_opt == "splunk":
        sendData=sendDataSplunk
    elif telem_opt == "ambient":
        import ambient # this is how to install it --> sudo pip install git+https://github.com/AmbientDataInc/ambient-python-lib.git
        am = ambient.Ambient(channelID, writeKey)  
        sendData=sendDataAmbient
    elif telem_opt == "gcs_spread":
        init_gcs_worksheet()
        sendData=put_data_on_gcs_worksheet   
    elif telem_opt == "influxdb":
        from influxdb import InfluxDBClient
        client = InfluxDBClient(host='192.168.31.53', port=8086, username='root', password='password', database='sensor')
        measurement = 'temperature'
        tags = {'place': 'umi','host': 'enOcean Temperature Sensors'}
        sendData=put_data_influx  
    elif telem_opt == "redis":
        import redis
        redis_client = redis.Redis(host = '172.17.0.2', port = 6379, decode_responses = True)
        sendData=put_data_redis
    elif telem_opt == "mysql":
        mySQLConnect()                                       # connects and creates clean database
        sendData=putValueMySQL
    else:                                                    # we assume its for mosquito broker internally running on host e.g. raspberry pi 4
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_publish = on_publish
        client.connect("localhost", MQTT_PORT, KEEP_ALIVE)    
        sendData=sendDataMosquito
        GTOPIC=MTOPIC
        
    # read the enOcean sensor         
    sensor1_rdy = False
    sensor2_rdy = False
    cnt = 0
    dataLen = 0
    optLen = 0
    telegraph,headList,dataList,optList = [],[],[],[]
    ready = True
    
    # look for the sensors then break out and send to the telemetry    
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
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # The data is displayed for debugging purposes.
            print "========"
            print timestamp
            print("telegraph...:", ":".join(telegraph))
            print( "head...", ":".join(headList))
            print( "data...", ":".join(dataList), "(length=%d)" % dataLen)
            print( "opt ...", ":".join(optList),  "(length=%d)" % optLen)
            print( "sensorID...", sensorId)

            # Temp sensor1 Send data when the sensor ID matches.
            if sensorId == SENSORID1 and sensor1_rdy == False:
                val = int(dataList[7],16)
                sensor1_temp = round((255.0-val)/255.0*40.0, 2)
                sensor_desc1 = 'temperature01 ' + SENSORID1 + " : "
                sensor1_rdy = True

            # Temp sensor1 Send data when the sensor ID matches.
            elif sensorId == SENSORID2 and sensor2_rdy == False:
                val = int(dataList[7],16)
                sensor2_temp = round((255.0-val)/255.0*40.0, 2)
                sensor_desc1 = 'temperature02 ' + SENSORID2 + " : "
                sensor2_rdy = True

            # Other sensors, ignore ID
            else:
                continue
                
    # return this data for printing to the stdout console         
    if sensor1_rdy and sensor2_rdy:
        sendData(sensor_desc1, sensor1_temp, sensor_desc2, sensor2_temp)
        return str(sensor1_temp),str(sensor2_temp)

if __name__ == '__main__':

    # Open serial port for communication with enOcean sensors
    try:
        s_port = serial.Serial(PORT, BAUD_RT)
        print("open serial port: %s" % PORT)
    except:
        print("cannot open serial port: %s" % PORT)
        exit(1)
                
    t1,t2 = Enocean2Telemetry(s_port, MY_CURRENT_TELEM)
    print("sensorID..." % SENSORID1)
    print("temperature" % t1)
    print("sensorID..." % SENSORID2)
    print("temperature" % t2)  
