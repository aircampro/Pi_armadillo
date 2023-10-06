#!/usr/bin/python3	
# this shows how to create a OPC Server with a method which is to multiply the 2 variables method.py

import asyncio
from asyncua import ua, uamethod, Server

# setup our server
ENDPOINT = 'opc.tcp://0.0.0.0:4840/freeopcua/server/'
# setup our own namespace, not really necessary but should as spec
NS_URI = 'http://examples.freeopcua.github.io'

@uamethod
def multiply(parent, x, y):
    print('multiply method call with parameters: ', x, y)
    return x * y

async def main():
    server = Server()
    await server.init()
    server.set_endpoint(ENDPOINT)

    idx = await server.register_namespace(NS_URI)
    objects = server.nodes.objects
    obj = await objects.add_object(idx, 'MyObject')

    arg_x = ua.Argument()
    arg_y = ua.Argument()
    arg_out = ua.Argument()

    await obj.add_method(idx, 'multiply', multiply, [arg_x, arg_y], [arg_out])

    async with server:
        while True:
            await asyncio.sleep(1)

if __name__ == '__main__':
    asyncio.run(main())