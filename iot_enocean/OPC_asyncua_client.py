#!/usr/bin/python3
# OPC Client which updates a random variable to the server 

import asyncio
import random
from asyncua import Client

# setup our server
URL = 'opc.tcp://localhost:4840/freeopcua/server/'
# setup our own namespace, not really necessary but should as spec
URI = 'http://examples.freeopcua.github.io'

async def main():
    async with Client(url=URL) as client:
        idx = await client.get_namespace_index(URI)
        var = await client.nodes.root.get_child(['0:Objects',
                                                 f'{idx}:MyObject',
                                                 f'{idx}:MyVariable'])
        print(var.__class__.__name__)

        print('Read  MyObject/MyVariable =', await var.read_value())
        # here we are just setting it to a random integer you would replace this with the data to send which you got from onme of the methods
        val = random.randint(1, 127)
        print('Write MyObject/MyVariable =', val)
        # now update the OPC server with the value
        await var.write_value(val)
        print('Read  MyObject/MyVariable =', await var.read_value())

if __name__ == '__main__':
    asyncio.run(main())