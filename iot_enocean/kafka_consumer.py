import json
from confluent_kafka import Consumer

KTOPIC = 'sensor_data'

consumer.subscribe([KTOPIC])

for msg in consumer:
    if msg.value():
        data = json.loads(msg.value().decode('utf-8'))
        if data is not None:
            print(f"rcv from {KTOPIC} {data}")