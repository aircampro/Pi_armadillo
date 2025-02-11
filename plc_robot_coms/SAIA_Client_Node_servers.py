#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# pip install -U digimat.saia
#
# use of this library for SAIA Burgess SBUS communication https://github.com/digimat/digimat-saia?tab=readme-ov-file
# Shows how to connect to remote EtherSBus node servers
#
from digimat.saia import SAIANode
import numpy as np

if __name__ == '__main__':

    node=SAIANode(250)

    server1=node.servers.declare('192.168.0.100')       # read a flag and chack its running status
    myRemoteFlag=server1.memory.flags[5]
    print(myRemoteFlag.read())

    print(server1.status)
    print(server1.isRunning())
    print(server1.isStopped())
    print(server1.isHalted())

    server2=node.servers.declare(253)                   # read our example Server
    myregister=server2.registers[0]
    print(myregister.read())

    # connect to a server with a map file
    map_file_name='xxxxx.map'
    server=node.servers.declare('192.168.0.48', mapfile=map_file_name)
    print(server.symbols.count())

    print(server.status)
    print(server.isRunning())
    print(server.isStopped())
    print(server.isHalted())
    
    # get a registers tag name 
    symbol=server.symbols.register(2295)
    print(symbol.tag)

    # integer
    register=server.registers[50]
    register.value=100
    print(register.value)
    print(register.hex)
    print(register.bin)

    # SAIA float
    register_ffp=server.registers[10]
    register_ffp.ffp=np.random.rand() * 100.0
    print(register_ffp.ffp)
    print(register_ffp.read())