#!/usr/bin/env python
#
# example using pay pay QR Code Payment Protocol
# https://github.com/nfcpy/nfcpy
# https://nfcpy.readthedocs.io/en/latest/
#
import webbrowser
import nfc
from nfc.tag import Tag

# Extract the necessary parts from the information written in NFC.
# The information written in NFC is `https://<site name>?id=1001`.
# We want the last 4 digits of the ID.
def get_id(tag: Tag) -> str:
    return tag.ndef.records[0].uri[-4:]

# show the url
def send_url(client_id) -> None:
    url = 'http://localhost:5000/pay'                                        # replace wuth <your server URL>
    url = url + "?id=" + client_id
    print(url)
    webbrowser.open(url, new=0, autoraise=True)

# get the url for the tag
def on_connect(tag: Tag) -> bool:
    send_url(get_id(tag))
    return True

if __name__ == "__main__":
    print("===== using PayPay system ====")
    with nfc.ContactlessFrontend('usb') as clf:
        target_req = nfc.clf.RemoteTarget("212F")
        target_req.sensf_req = bytearray.fromhex("0000030000")
        while 1:
            target_res = clf.sense(target_req,iterations=10,interval=0.01)
            if target_res != None:
                tag = nfc.tag.activate(clf,target_res)
                tag.sys = 3
                idm = binascii.hexlify(tag.idm)
                print(idm)
                break
    while True:
        with nfc.ContactlessFrontend("usb") as clf:
        	print(clf)                                                       # print device id e.g. SONY RC-S380/P on usb:001:005	
            clf.connect(rdwr={"on-connect": on_connect})
						