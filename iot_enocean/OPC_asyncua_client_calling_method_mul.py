#!/usr/bin/python3
# OPC Client is calling the servers multiply method with 2 variables method.py

import asyncio
import random
from asyncua import Client

# setup our server
URL = 'opc.tcp://localhost:4840/freeopcua/server/'
# setup our own namespace, not really necessary but should as spec
NS_URI = 'http://examples.freeopcua.github.io'

async def main():
    async with Client(url=URL) as client:
        idx = await client.get_namespace_index(NS_URI)
        obj = await client.nodes.root.get_child(['0:Objects', '2:MyObject'])

        # Create 2 random variables and call the servers multiply method
        x = random.randint(2, 10)
        y = random.randint(2, 10)
        res = await obj.call_method(f'{idx}:multiply', x, y)
        print(f'method multiply: {x} x {y} = {res}')

if __name__ == "__main__":
    asyncio.run(main())