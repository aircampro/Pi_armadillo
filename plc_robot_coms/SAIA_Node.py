#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# pip install -U digimat.saia
#
# use of this library for SAIA Burgess SBUS communication https://github.com/digimat/digimat-saia?tab=readme-ov-file
# EtherSBus Node (Server)
#
from digimat.saia import SAIANode
import numpy as np
import time

def mkNode(id=253):
    return SAIANode(id)
	
def doDemo(node):
    myflag=node.memory.flags[18]
    myflag.value=True
    print("flag value is ",myflag.value)
    myflag.value=False
    print("flag value is ",myflag.value)

    myregister=node.memory.registers[0]
    myregister.value=100
    print("flag value is ",myregister.value)
    # Helpers are available to set/get the register value with common encodings
    #.float32	IEEE float32 encoding (big-endian)
    #.sfloat32	Swapped IEEE float32 encoding (little-endian)
    #.ffp	Motorola Fast Floating Point encoding (SAIA Float)
    #.float	Alias for FFP encodings (easier to remember)
    #.int10	x10 rounded value (21.5175 is encoded as 215)
    #.formatedvalue	Reuse the last used formater
    #
    myregister.float32=21.5
    print("float as int is ",myregister.value)
    print("float as big-endian float is ",myregister.float32)
    print("float as ffp is ",myregister.ffp)
    print("float as little-endian float is ",myregister.sfloat32)
    print("float as int10 is ",myregister.int10)
    time.sleep(2)
    rand_real = np.random.rand() * 22.0
    print("random generated is ",rand_real)
    myregister.ffp=-rand_real
    print("float as int is ",myregister.value)
    print("float as big-endian float is ",myregister.float32)
    print("float as ffp is ",myregister.ffp)
    print("float as little-endian float is ",myregister.sfloat32)
    print("float as int10 is ",myregister.int10)

if __name__ == '__main__':

    nd=mkNode()
    while True:
        doDemo(nd)
        time.sleep(10)