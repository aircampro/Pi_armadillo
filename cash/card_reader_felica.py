#!/usr/bin/env python
#
# python -m pip install nfcpy
# sudo pip3 install nfcpy
# git clone https://github.com/nfcpy/nfcpy.git
#
# sudo nano /etc/udev/rules.d/nfcdev.rules
# for sont S-380 add this line SUBSYSTEM=="usb", ACTION=="add", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="06c3", GROUP="plugdev"         # Sony RC-S380/P
#
# $ lsusb
# Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# Bus 001 Device 003: ID 054c:06c1 Sony Corp.  <=NFC
#
# works with either Sony PaSoRi：RC-S370 or RC-S380
# or https://www.amazon.co.jp/%E3%82%BD%E3%83%8B%E3%83%BC-NFC%E9%80%9A%E4%BF%A1%E3%83%AA%E3%83%BC%E3%83%80%E3%83%BC-PaSoRi-RC-S380-S/dp/B00VR1WARC
# https://watanabe-ichiro-nikki.hatenablog.com/entry/nfcpy-pasori/
#
# nfc tag sticker for test www.amazon.co.jp/dp/B00GXSGL5G
# 
# Type3Tag	Indicates that the Type of NDEF is 3 (≈ FeliCa)
# RC-SA00/1	It is possible to check the model number of the IC chip of the NFC card from
# the technical information and the list of target products.
# ID	IDmThe
# top 2 bytes of the manufacturing ID used by the NFC reader/writer to identify the card with which the NFC reader / writer communicates,
# followed by the 6 bytes for the card identification number.
# PMM	PMmParameters that
# allow the NFC reader/writer to identify the type and performance of the card with which it communicates.
# SYS	System Code:
# A value to identify the system, assigned
# to each operator/use
# System / Area	System > area > nested in a structure called a service > block
# Random Service	Services that users can freely block and access
#
from typing import cast

import nfc
from nfc.tag import Tag
from nfc.tag.tt3 import BlockCode, ServiceCode, Type3Tag
from nfc.tag.tt3_sony import FelicaStandard
import binascii

# make this suit your application
FIRST_NUMBER = {
    "American Express" : "3",
    "Visa" : "4",
    "MasterCard" : "5",
    "Discover Card" : "6"}
    
SYSTEM_CODE = FIRST_NUMBER["Visa"]                                                         # system code for determining card type
service_number_id = 3
block_number_id = 9
service_number_nm = 2
block_number_nm = 1

def read_data_block(tag: Type3Tag, service_code_number: int, block_code_number: int, service_attribute: int) -> bytearray:
    service_code = ServiceCode(service_code_number, service_attribute)
    block_code = BlockCode(block_code_number)
    read_bytearray = cast(bytearray, tag.read_without_encryption([service_code], [block_code]))
    return read_bytearray

def get_student_id(tag: Type3Tag) -> str:
    student_id_bytearray = read_data_block(tag, service_number_id, block_number_id, service_attribute)
    return student_id_bytearray.decode("utf-8")                                             # Extract only the necessary parts with slicing

def get_student_name(tag: Type3Tag) -> str:
    student_name_bytearray = read_data_block(tag, service_number_nm, block_number_nm, service_attribute)
    return student_name_bytearray.decode("utf-8")                                          # Extract only the necessary parts with slicing

def on_connect(tag: Tag) -> bool:
    print("connected")
    print(tag)
    idm = binascii.hexlify(tag._nfcid).decode("utf-8")
    print("IDm : " + str(idm))

    if tag.ndef:
        print("NDEF Capabilities:")
        print("  readable  = %s" % ("no", "yes")[tag.ndef.is_readable])
        print("  writeable = %s" % ("no", "yes")[tag.ndef.is_writeable])
        print("  capacity  = %d byte" % tag.ndef.capacity)
        print("  message   = %d byte" % tag.ndef.length)
        if tag.ndef.length > 0:
            print("NDEF Message:")
            for i, record in enumerate(tag.ndef.records):
                print("record", i + 1)
                print("  type =", repr(record.type))
                print("  name =", repr(record.name))
                print("  data =", repr(record.data))
                    
    if isinstance(tag, FelicaStandard) and SYSTEM_CODE in tag.request_system_code():        # If the card is FeliCa and a system code exists for the id and name card
        tag.idm, tag.pmm, *_ = tag.polling(SYSTEM_CODE)
        print(get_student_id(tag))
        print(get_student_name(tag))
    elif tag.TYPE == "Type4Tag":
        id_info = binascii.hexlify(tag.identifier).decode("utf-8")
        print(id_info)
    else:
        id_info = binascii.hexlify(tag.idm).decode("utf-8")
        print(id_info)
    return True  

def on_release(tag: Tag) -> None:
    print("released")

if __name__ == '__main__':
    with nfc.ContactlessFrontend("usb") as clf:
        while True:
            try:
                clf.connect(rdwr={"on-connect": on_connect, "on-release": on_release})
            except:
                pass 

