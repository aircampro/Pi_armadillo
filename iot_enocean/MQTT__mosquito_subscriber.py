import paho.mqtt.client as mqtt

MQTT_PORT = 1883
KEEP_ALIVE = 60
TOPIC = "topic/B/"

# Broker Processing when connected to
def on_connect(client, userdata, flag, rc):
  print("connect broker:" + str(rc))
  client.subscribe(TOPIC)

# Broker Processing when disconnected
def on_disconnect(client, userdata, rc):
  if  rc != 0:
    print("disconnect broker")

# do when the broker (e.g. mosquito) recieves a message
def on_message(client, userdata, msg):
  print("Received message '" + str(msg.payload) + "' on topic '" + msg.topic + "' with QoS " + str(msg.qos))

client = mqtt.Client()
# Register a callback function
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

client.connect("localhost", MQTT_PORT, KEEP_ALIVE)

# メッセージを待ち受ける
client.loop_forever()