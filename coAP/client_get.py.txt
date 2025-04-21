# ref:- https://github.com/chrysn/aiocoap?tab=readme-ov-file
#
import logging
import asyncio

from aiocoap import *

logging.basicConfig(level=logging.INFO)

async def main():
    protocol = await Context.create_client_context()
    # GET, POST, PUT, DELETE, DISCOVER, OBSERVE
    request = Message(code=GET, uri="coap://localhost/time")

    try:
        response = await protocol.request(request).response
    except Exception as e:
        print("Failed to fetch resource:")
        print(e)
    else:
        print("Result: %s\n%r" % (response.code, response.payload))

async def post_file(fp='/home/drive/foo.bin'):
    file_path = 
    with open(file_path, 'rb') as f:
        data = f.read()
    print('Request: \n%s'%(data))

    context = await Context.create_client_context()
    await asyncio.sleep(2)

    request = Message(code=POST, payload=data, uri="coap://[HOST_NAME]/[PATH]")
    response = await context.request(request).response
    print('Result: %s\n%r'%(response.code, response.payload))

if __name__ == "__main__":
    asyncio.run(main())
    asyncio.run(post_file())
