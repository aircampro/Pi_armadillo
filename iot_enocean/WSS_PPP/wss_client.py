#!/usr/bin/env python3
# wss_client.py Websockets SSL Client
# $ python3 -m venv .venv
# $ . .venv/bin/activate
# $ pip install websockets
# $ python3 client.py -l 10022 wss://example.com 
# $ ssh -p 10022 example.com
#
import asyncio
import ssl
from argparse import ArgumentParser
from functools import partial
from websockets.asyncio.client import connect

size = 1024

async def ssh_client_to_ws(reader, websocket):
    try:
        while data := await reader.read(size):
            await websocket.send(data)
    except OSError as e:
        if not e.winerror or e.winerror != 64:
            raise e

async def ws_to_ssh_client(writer, websocket):
    async for message in websocket:
        writer.write(message)
        await writer.drain()
    writer.close()
    await writer.wait_closed()

async def tcp_server(url, cafile, reader, writer):
    ssl_context = ssl.create_default_context(cafile=cafile)
    async with connect(url, ssl=ssl_context) as websocket:
        print(f"connected: ${websocket.remote_address}")
        tasks = []
        tasks.append(asyncio.create_task(ssh_client_to_ws(reader, websocket)))
        tasks.append(asyncio.create_task(ws_to_ssh_client(writer, websocket)))
        _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        await websocket.close()
        await websocket.wait_closed()
        print("connection closed")

async def main(args):
    if args.listen_port:
        l_port = args.listen_port
    else:
        l_port = 22
    server = await asyncio.start_server(
        partial(tcp_server, args.url, args.cafile),
        port=l_port
    )
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    print(f"Serving on {addrs}")

    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-listen_port", type=int, default=22, metavar="<listening port>")
    parser.add_argument("--cafile", metavar="<CA cert file>")
    parser.add_argument("url", help="a URL like wss://example.com")
    args = parser.parse_args()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print()