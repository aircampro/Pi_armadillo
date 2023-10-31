# Raspberry Pi Receiver using Semtech SX126x on SPI
# for more info this github is the master https://github.com/chandrawi/LoRaRF-Python/wiki
# requires that library to be installed
#
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x, LoRaSpi, LoRaGpio
import time

def bytes_to_decimal(i,d):
    xx = i - 127
    dec = (-d if xx < 0 else d)/100
    return xx + dec
	
# Begin LoRa radio with connected SPI bus and IO pins (cs and reset) on GPIO
# SPI is defined by bus ID and cs ID and IO pins defined by chip and offset number
spi = LoRaSpi(0, 0)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 24)
busy = LoRaGpio(0, 23)
LoRa = SX126x(spi, cs, reset, busy)
print("Begin LoRa radio")
if not LoRa.begin() :
    raise Exception("Something wrong, can't begin LoRa radio")

# Configure LoRa to use TCXO with DIO3 as control
print("Set RF module to use TCXO as clock reference")
LoRa.setDio3TcxoCtrl(LoRa.DIO3_OUTPUT_1_8, LoRa.TCXO_DELAY_10)

# Set frequency to 868 Mhz
print("Set frequency to 868 Mhz")
LoRa.setFrequency(868000000)
#print("Set frequency to 433 Mhz")
#LoRa.setFrequency(433000000)

# Set RX gain to boosted gain
print("Set RX gain to boosted gain")
LoRa.setRxGain(LoRa.RX_GAIN_BOOSTED)

# Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
print("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5")
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)

# Configure packet parameter including header type, preamble length, payload length, and CRC type
print("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on")
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 15
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)

# Set syncronize word for public network (0x3444)
print("Set syncronize word to 0x3444")
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Receiver Listen --\n")

# IDs and message format from received message
gatewayId = 0xCC                                # if you want to check the gateway id matches
format = 'BBHI5p'                               # 5p is char array 5 long = temp/hum/slot
length = struct.calcsize(format)

# Receive message continuously
while True :

    # Listen for a LoRa packet in 10 ms and sleep in 10 ms
    rxPeriod = 20
    sleepPeriod = 10
    LoRa.listen(rxPeriod, sleepPeriod)
    # Wait for incoming LoRa packet
    LoRa.wait()
    
    # Check for incoming LoRa packet
    if LoRa.available() :

        # Put received packet to message and counter variable
        message = ""
        # Get received structured message
        message = LoRa.get(length)
        structure = struct.unpack(format, message)

        # Print received message and counter in serial
        print("raw : ", message)
        tmp = bytes_to_decimal(*structure[4].decode('utf-8')[0:2])
        hum = bytes_to_decimal(*structure[4].decode('utf-8')[2:4])
        slot = chr(structure[4].decode('utf-8')[4:5])
        print("temp : ",temp,"humidity : ",hum,"slot : ",slot)

        gw = structure[0]
        nid = structure[1]
        msgid = structure[2]
        timemsg = structure[3]
        print("gateway : ",gw,"network id : ",nid,"message id : ",msgid, "time ",timemsg)
        
        # Print packet/signal status including RSSI, SNR, and signalRSSI
        print("Packet status: RSSI = {0:0.2f} dBm | SNR = {1:0.2f} dB".format(LoRa.packetRssi(), LoRa.snr()))

        # Show received status in case CRC or header error occur
        status = LoRa.status()
        if status == LoRa.STATUS_CRC_ERR : print("CRC error")
        elif status == LoRa.STATUS_HEADER_ERR : print("Packet header error")