# mqtt to AWS sending zigbee slave with temp and volts with threshold set from AWS cloud
# this is a slave version which sends from the field radio to the cloud
#
from umqtt.simple import MQTTClient
import network
import time
import ujson
import xbee
import time                          

# AWS endpoint parameters.
HOST = b'FILL_ME_IN'    # ex: b'abcdefg1234567'
REGION = b'FILL_ME_IN'  # ex: b'us-east-1'

CLIENT_ID = "clientID"  # Should be unique for each device connected.
AWS_ENDPOINT = b'%s.iot.%s.amazonaws.com' % (HOST, REGION)

# SSL certificates.
SSL_PARAMS = {'keyfile': "/flash/cert/aws.key",
              'certfile': "/flash/cert/aws.crt",
              'ca_certs': "/flash/cert/aws.ca"}

TOPIC_UPDATE = "sample/update"
TOPIC_TEMP = "sample/temp"
TOPIC_V = "sample/volts"
TOPIC_ALARM = "sample/alarm"

VAR_THRESHOLD_TEMP = "threshold_temp"
VAR_WAIT_TIMER = "wait_timer"

ZBTEMP_OFFSET=14.0 

def get_temp_volts(): 
    """
     read the temp and volts from the sensor over zigbee
    """                             
    temp = xbee.atcmd('TP') - ZBTEMP_OFFSET   
    volt = xbee.atcmd('%V') / 1000          
    return temp, volt       
    
def cb_func(topic, msg):
    """
     Callback executed when messages from subscriptions are received. Updates
     the temperature threshold or wait timer accordingly.

    :param topic: Topic of the message.
    :param msg: Received message.
    """

    global threshold_temp
    global wait_timer

    print("- Message received!")
    d = ujson.loads(msg.decode("utf-8"))
    if VAR_THRESHOLD_TEMP in d.keys():
        try:
            threshold_temp = float(d[VAR_THRESHOLD_TEMP])
            print("   * Updated threshold to '%s'" % threshold_temp)
        except ValueError:
            print("   * [%s] Expected <class 'float'>, received %s"
                  % (VAR_THRESHOLD_TEMP, type(d[VAR_THRESHOLD_TEMP])))
    if VAR_WAIT_TIMER in d.keys():
        try:
            wait_timer = int(d[VAR_WAIT_TIMER])
            print("   * Updated wait timer to '%s'" % wait_timer)
        except ValueError:
            print("   * [%s] Expected <class 'int'>, received %s"
                  % (VAR_WAIT_TIMER, type(d[VAR_WAIT_TIMER])))


print(" +------------------------------------------------+")
print(" | XBee MicroPython AWS Temperature Sensor Sample |")
print(" +------------------------------------------------+\n")

threshold_temp = 80.0      # default can be set from AWS
wait_timer = 10
conn = network.Cellular()

print("- Waiting for the module to be connected to the cellular network... ",
      end="")
while not conn.isconnected():
    time.sleep(5)
print("[OK]")

# Connect to AWS.
client = MQTTClient("clientId", AWS_ENDPOINT, ssl=True, ssl_params=SSL_PARAMS)

client.set_callback(cb_func)
print("- Connecting to AWS... ", end="")
client.connect()
print("[OK]")
# Subscribe to topic.
print("- Subscribing to topic '%s'... " % TOPIC_UPDATE, end="")
client.subscribe(TOPIC_UPDATE)
print("[OK]")

# Start taking temperature samples read over the zigbee link
above = False
timer = 0
while True:
    client.check_msg()
    cur_temp, cur_v = get_temp_volts()
    if timer >= wait_timer:
        # If the wait timer expires, update the value of the temperature.
        print("- Publishing temperature... ", end="")
        client.publish(TOPIC_TEMP, '{"temp": "%s"}' % cur_temp)
        client.publish(TOPIC_V, '{"volts": "%s"}' % cur_v)
        print("[OK]")
        timer = 0
    if cur_temp >= threshold_temp:
        # If the temperature exceeds the threshold and an alarm was not set,
        # send an alarm.
        if not above:
            print("- Publishing alarm (high temperature)... ", end="")
            client.publish(TOPIC_ALARM,
                           '{"message": "Exceeded set threshold of %s"}'
                           % threshold_temp)
            print("[OK]")
            above = True
    else:
        # If the temperature is below the threshold and an alarm was not set,
        # send an alarm.
        if above:
            print("- Publishing alarm (low temperature)... ", end="")
            client.publish(TOPIC_ALARM,
                           '{"message": "Below set threshold of %s"}'
                           % threshold_temp)
            print("[OK]")
            above = False
    timer += 1
    time.sleep(1)