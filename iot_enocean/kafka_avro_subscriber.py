# example of kafka avro topic sunscriber
#
from confluent_kafka import DeserializingConsumer
from confluent_kafka.avro import AvroDeserializer
from confluent_kafka.avro.serializer import SerializerError
from confluent_kafka.schema_registry import SchemaRegistryClient

schema_registry_conf = {'url': 'http://myschemaregistry.com'}
schema_registry_client = SchemaRegistryClient(schema_registry_conf)
KATOPIC='avro_topic'
KAGROUP='avro_group'
avro_deserializer = AvroDeserializer(schema_registry_client)

consumer_conf = {
    'bootstrap.servers': 'localhost:9092',
    'group.id': KAGROUP,
    'auto.offset.reset': 'earliest',
    'key.deserializer': avro_deserializer,
    'value.deserializer': avro_deserializer
}

if __name__ == "__main__":
    consumer = DeserializingConsumer(consumer_conf)
    consumer.subscribe([KATOPIC])
    try:
        while True:
            try:
                msg = consumer.poll(1.0)
                if msg is None:
                    continue
                data = msg.value()
                if data is not None:
                    print("data ", data)
            except SerializerError as e:
                print(f"Error deserializing Avro message: {e}")
                continue
    finally:
        consumer.close()