#  ================================================ LoRa Device ===============================================================
#  Flip&Click as MCU board, a LoRa Click and a Temp&Hum Click, that carries ST’s HTS221 temperature and relative humidity sensor
#
#  https://zerynth.com/blog/flip_click-officially-supported-by-zerynth/
#
import streams

# https://www.mikroe.com/lr-click
#
from microchip rn2483 import rn2483

# https://docs.zerynth.com/latest/#module-hts221
# https://www.mikroe.com/temp-hum-click
#
from stm.hts221 import hts221

# also define LED outputs for alarms
#
PIN_LED1=38
pinMode(PIN_LED1, OUTPUT)
PIN_LED2=39
pinMode(PIN_LED2, OUTPUT)

# ------ get data from the HTS221 sensor ------
def get_data_from_hts221(hts_obj, slot):
    temp, hum = hts_obj.get_temp_humidity()                                       # calc and get the temperature
    print("HTS221-Slot ",slot,"\ntemperature : ",temp,"humidity : ",hum)
    data = bytearray(5)
    data[0:2] = bytearray([ int(temp) + 127, int((temp - int(temp)) * 100) ])
    data[2:4] = bytearray([ int(hum) + 127, int((hum - int(hum)) * 100) ])        # fill the send bytearray with data
	data[4:5] = bytearray([ord(slot)])                                            # finalize it with the slot number
	return data,temp,hum

TMP_LIMIT=34.5
H_LIMIT=57                                                                    # set the alarm linit for the LED
def drive_led(t1, t2, t3, v_lim, pinled):
    if (((t1 > v_lim) and (t2 > v_lim)) and (t3 > v_lim)):
        digitalWrite(pinled, 1)
    elif (((t1 < v_lim) and (t2 < v_lim)) and (t3 < v_lim)):	
        digitalWrite(pinled, 0)
    print("LED ",pinled, " : ", digitalRead(pinled))
    
streams.serial()

try:
    rst = D16                                                                         # reset pin
    appeui = "YOURAPPEUI"
    appkey = "YOURAPPKEY"
    print("joining...")

    if not rn2483.init(SERIAL1, appeui, appkey, rst):                                 # LoRa Click on SlotA
        print("denied.....")
        raise Exception

    # send 1st msg
    print(rn2483.tx_uncnf('TTN'))

    # get the temperature and humidity from the HTS221 on Slot B refer to flip&click sam3s diagram for pins
    temp_hum_b = hts221.HTS221( I2C1, D26 )	
    # get the temperature and humidity from the HTS221 on Slot C
    temp_hum_c = hts221.HTS221( I2C1, D31 )
    # get the temperature and humidity from the HTS221 on Slot D
    temp_hum_d = hts221.HTS221( I2C1, D38 )
	
    while True:
        data,t1,h1 = get_data_from_hts221(temp_hum_b, "B")
        r = rn2483.tx_uncnf(data)                                                     # send data to TTN from slot B
        sleep(1000)
        data,t2.h2 = get_data_from_hts221(temp_hum_c, "C")
        r = rn2483.tx_uncnf(data)                                                     # send data to TTN from slot C
        sleep(1000)
        data,t3,h3 = get_data_from_hts221(temp_hum_d, "D")
        r = rn2483.tx_uncnf(data)                                                     # send data to TTN from slot D
        thread(drive_led(t1, t2, t3, TMP_LIM, PIN_LED1))                              # drive local temperature LED
        thread(drive_led(h1, h2, h3, H_LIM, PIN_LED2))                                # drive local humidity LED       
        sleep(1000)
		
except Exception as e:
    print("exception : ",e)