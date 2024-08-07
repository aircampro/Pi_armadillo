#!/usr/bin/env python
# simple CAN IsoTP sender
#
import isotp
import time
import can

#define the interface
bus = can.interface.Bus(bustype='vector', channel=0, bitrate=500000, app_name='python-can')

# receive ID 0xF1、transmit ID 0x10、Normal
addr = isotp.Address(isotp.AddressingMode.Normal_11bits, rxid=0xF1, txid=0x10)
stack = isotp.CanStack(bus, address=addr)

#send 11 bytes
stack.send(b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b')
time.sleep(2)

while stack.transmitting():
    stack.process()
    time.sleep(stack.sleep_time())

bus.shutdown()