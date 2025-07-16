# DHT11 using Adafruit library https://github.com/adafruit/Adafruit_Python_DHT
#
import Adafruit_DHT
# GPIO 
DHT_GPIO = 18
# make DHT11 object
sensor = Adafruit_DHT.DHT11
MAX_TRIAL = 3
for _ in range(self.MAX_TRIAL):
    humidity, temperature =  Adafruit_DHT.read_retry(sensor, DHT_GPIO)
    if humidity is None or temperature is None:
        continue
    return [humidity, temperature]
