#!/usr/bin/env python
#
# Use nfcpy to handle Express Transit cards for Apple Pay
#
# NFC is a payment method that uses short-range wireless communication 
# technology to exchange data by simply holding an IC card or smartphone over a terminal. 
# NFC is widely adopted for mobile payments such as Apple Pay and Google Pay
#
import nfc

# 0003(Suica)
def on_startup(targets):
    for target in targets:
        # 0003(Suica)
        target.sensf_req = bytearray.fromhex("0000030000")
    return targets

def on_connect(tag):
    print(tag)

def main():
    clf = nfc.ContactlessFrontend('usb')
    # 212F(FeliCa)
    target_req = nfc.clf.RemoteTarget("212F")
    # 0003(Suica)
    target_req.sensf_req = bytearray.fromhex("0000030000")
    while True:
        target_res = clf.sense(target_req, iterations=10, interval=0.01)
        if target_res != None:
            print(binascii.hexlify(target_res.sensf_res))
            tag = nfc.tag.tt3.Type3Tag(clf, target_res)
            tag.sys = 3
            print(tag)
            break
    # 212F(FeliCa) with 0003(Suica)
    clf.connect(rdwr={'targets': ['212F'], 'on-startup': on_startup, 'on-connect': on_connect})
    clf.close()

if __name__ == '__main__':
    main()
  

