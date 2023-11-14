# STM32 .Net MF Learning-19 DHT11 Temperature and Humidity Sensor Communication (Part I)
# https://blog.csdn.net/yfiot/article/details/5996524
#
# this is untested 
dht_pin =16                 # Since the data pin of DHT11 is connected to GPIO16

# The code that encapsulates the data in a function is as follows:
#
def GetDTH(): 
    data = []
    j = 0 
    GPIO.setup(dht_pin, GPIO.OUT)
    GPIO.output(dht_pin, GPIO.LOW)
    time.sleep(0.02)
    GPIO.output(dht_pin, GPIO.HIGH)
    GPIO.setup(dht_pin, GPIO.IN)

    while GPIO.input(dht_pin) == GPIO.LOW:
        continue
    while GPIO.input(dht_pin) == GPIO.HIGH:
        continue

    while j < 40:
        k = 0
        while GPIO.input(dht_pin) == GPIO.LOW:
        continue
        while GPIO.input(dht_pin) == GPIO.HIGH:
        k += 1
        if k > 100:
        break
        if k < 8:
        data.append(0)
        else:
        data.append(1)  
        j += 1

    humidity_bit = data[0:8]
    humidity_point_bit = data[8:16]
    temperature_bit = data[16:24]
    temperature_point_bit = data[24:32]
    check_bit = data[32:40]

    humidity = 0
    humidity_point = 0
    temperature = 0
    temperature_point = 0
    check = 0

    for i in range(8):
        humidity += humidity_bit[i] * 2 ** (7-i)
        humidity_point += humidity_point_bit[i] * 2 ** (7-i)
        temperature += temperature_bit[i] * 2 ** (7-i)
        temperature_point += temperature_point_bit[i] * 2 ** (7-i)
        check += check_bit[i] * 2 ** (7-i)
        
    tmp = humidity + humidity_point + temperature + temperature_point
    if check == tmp:
       return temperature,humidity
    else:
       print "wrong"
       return -99,-99

if __name__ == '__main__':
    
    t,h = GetDTH()    
    if (t == 99 ) && (h == 99):
        print("---------- error with STM DTH11 data -----------")
    else:
        print("temp = {} hum = {}",t,h)