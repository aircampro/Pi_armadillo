# Simple LoRa Receiver (single pass)
# Raspberry Pi with RFM95W LoRa module can try RFMW95W SX1278 SX1276
# 433/868MHz
# uses :- https://github.com/wdomski/LoRa-RaspberryPi
#
import loralib

def bytes_to_decimal(i,d):
    xx = i - 127
    dec = (-d if xx < 0 else d)/100
    return xx + dec
    
loralib.init(1, 868000000, 7)                 # when set at 868MHz
#loralib.init(1, 433000000, 7)                  when set at 433MHz
data=loralib.recv();
# >>> (b'your message', 5, -25, -94, 9, 0)
if (data[1] >= 5) and (data[5] == 0):         # buffer length is more than or = to 5 bytes and no CRC error
    tmp = bytes_to_decimal(*data[0][0:2])
    hum = bytes_to_decimal(*data[0][2:4])
    slot = chr(data[0][4:5])
    print("temp : ",temp,"humidity : ",hum,"slot : ",slot)
print("prrsi : ",data[2],"rrsi : ",data[3],"snr : ",data[4],"error : ",data[5])