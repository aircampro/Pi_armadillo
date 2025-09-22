# Example of kafka avro topic producer
# AVRO is very fasr compared to json
#
# ref:- https://github.com/rtarar/AvroProducer/tree/master
#
from confluent_kafka import avro
from confluent_kafka.avro import AvroProducer
import csv
import sys
KATOPIC='avro_topic'

# usage : <this_prog> config_file.avsc data_file.csv
if len(sys.argv) >= 2:
    av_con_fl=str(sys.argv[1])
    data_fl=str(sys.argv[2])
else:
    print("usage : <this_prog> config_file.avsc data_file.csv im using the default files")
    av_con_fl='/home/work/src/avroproducer/arlncsvsource.avsc'
    data_fl='/home/work/src/avroproducer/test.csv'

if __name__ == "__main__":

    # this is the path name to your avro configuration
    value_schema = avro.load(av_con_fl)

    AvroProducerConf = {'bootstrap.servers': 'localhost:9092', 'schema.registry.url': 'http://localhost:8081',                                  }
    avroProducer = AvroProducer(AvroProducerConf, default_value_schema=value_schema)

    with open(data_fl) as file:
        reader = csv.DictReader(file)
        for row in reader:
            print(row)
            avroProducer.produce(topic=KATOPIC, value=row)
            avroProducer.flush()