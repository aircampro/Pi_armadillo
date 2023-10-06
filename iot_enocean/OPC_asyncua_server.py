#!/usr/bin/python3
# OPC Server which prints the value from the client when it changes value 

import asyncio
from asyncua import Server

# setup our server
END_POINT = 'opc.tcp://0.0.0.0:4840/freeopcua/server/'
# setup our own namespace, not really necessary but should as spec
URI = 'http://examples.freeopcua.github.io'

async def main():
    server = Server()
    await server.init()
    server.set_endpoint(END_POINT)
    idx = await server.register_namespace(URI)
    myobj = await server.nodes.objects.add_object(idx, 'MyObject')
    myvar = await myobj.add_variable(idx, 'MyVariable', 0)
    await myvar.set_writable()

    pre_val = 0
    async with server:
        while True:
            await asyncio.sleep(1)
            val = await myvar.get_value()
            if pre_val != val:
                print('MyObject/MyVariable', val)
                pre_val = val

if __name__ == '__main__':
    asyncio.run(main())