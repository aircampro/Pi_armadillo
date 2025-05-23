# this works with sony RC-S380 
# uses swig so please install first https://www.swig.org/
# $ pip install pyscard
# $ sudo apt install libusb-dev libpcsclite-dev opensc pcscd pcsc-tools
#
# this program demostrates some methods using the pyscard library
#
from smartcard.util import toHexString
from smartcard.System import readers as get_readers
import time

readers = get_readers()
print(readers)

while True:
    try:
        conn = readers[0].createConnection()                            # first reader is connection
        conn.connect()

        send_data = [0xFF, 0xCA, 0x00, 0x00, 0x00]                      # send bytes
        recv_data, sw1, sw2 = conn.transmit(send_data)                  # send the data and recieve back the id
        
        print(toHexString(recv_data))                                   # print the RFID that comes back
        break
    except KeyboardInterrupt:
        break
    except:
        time.sleep(0.5)

conn.disconnect()                                                       # disconnect

# works with this reader ACR1251CL-NTT Com Main
# https://www.ntt.com/business/services/application/authentication/jpki/download7.html
#
from smartcard.CardType import AnyCardType
from smartcard.CardConnection import CardConnection
from smartcard.CardRequest import CardRequest
from smartcard.util import toHexString

cardtype = AnyCardType()
cardrequest = CardRequest( timeout=1, cardType=cardtype )
cardservice = cardrequest.waitforcard()
cardservice.connection.connect( CardConnection.T1_protocol )
print ("ATR:", toHexString( cardservice.connection.getATR() ))

# AID
SELECT = [0x00, 0xA4, 0x04, 0x00]
CARD_AID = [0xF2, 0x22, 0x22, 0x22, 0x22 ]
LEN = [len(CARD_AID)]
# APDU transmision string
apdu = SELECT + LEN + CARD_AID
print("send apdu:", toHexString(apdu))
# rx.tx the data
data, sw1, sw2 = cardservice.connection.transmit(apdu)
print("SW1SW2: %x %x" % (sw1, sw2))
print("received:", toHexString(data))

# disconnect
cardservice.connection.disconnect()
print("...disconnected.")

# Another card reader example
#
from smartcard.CardMonitoring import CardMonitor, CardObserver
from smartcard.util import toHexString

from time import sleep

# print card data
class PrintObserver(CardObserver):
    def update(self, observable, actions):
        (addedcards, removedcards) = actions
        for card in addedcards:
            print("+Inserted: ", toHexString(card.atr))
        for card in removedcards:
            print("-Removed: ", toHexString(card.atr))

if __name__ == '__main__':
    print("Please put smartcard on reader.")
    # Ctrl + C will exit
    try:
            cardmonitor = CardMonitor()
            cardobserver = PrintObserver()
            cardmonitor.addObserver(cardobserver)
            while True:
                sleep(10)
        
    except KeyboardInterrupt:
        cardmonitor.deleteObserver(cardobserver)