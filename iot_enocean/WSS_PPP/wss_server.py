#!/usr/bin/env python3
# wss_server.py Websockets SSL Server
#$ python3 -m venv .venv
#$ . .venv/bin/activate
#$ pip install websockets
#$ sudo .venv/bin/python3 server.py --certfile /etc/ssl/chained.crt --keyfile /etc/ssl/private/private.key
#
import asyncio
import ssl
from argparse import ArgumentParser
from websockets.asyncio.server import serve

wss_port = 443
ssh_port = 22
size = 1024

async def ssh_to_ws(reader, websocket):
    while data := await reader.read(size):
        await websocket.send(data)

async def ws_to_ssh(writer, websocket):
    async for message in websocket:
        writer.write(message)
        await writer.drain()
    writer.close()
    await writer.wait_closed()

async def ws_server(websocket):
    reader, writer = await asyncio.open_connection(port=ssh_port)
    remote_address = websocket.remote_address
    print(f"connected from: {remote_address}")
    tasks = []
    tasks.append(asyncio.create_task(ssh_to_ws(reader, websocket)))
    tasks.append(asyncio.create_task(ws_to_ssh(writer, websocket)))
    _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()
    await websocket.close()
    await websocket.wait_closed()
    print(f"connection closed: {remote_address}")

async def main(args):
    global wss_port, ssh_port
    if args.certfile:
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(args.certfile, keyfile=args.keyfile, password=args.password)
    else:
        ssl_context = ssl.create_default_context()
    if args.ssh_port:
        ssh_port = int(args.ssh_port)
    if args.wss_port:
        wss_port = int(args.wss_port)
    server = await serve(ws_server, port=wss_port, ssl=ssl_context)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    print(f"Serving on {addrs}")
    await server.serve_forever()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--certfile", metavar="<PEM certificates for the server and the CAs")
    parser.add_argument("--keyfile", metavar="<private key file>")
    parser.add_argument("--password", metavar="<private key password>")
    parser.add_argument("--ssh_port", metavar="ssh port (default 22)")
    parser.add_argument("--wss_port", metavar="wss port (default 443)")
    args = parser.parse_args()

    try:
        # asyncio.run() is used when running this example with Python 3.7 and higher.
        asyncio.run(main(args))
    except AttributeError:
        # For Python 3.6 a bit more code is required to run the main() task on
        # an event loop.
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main(args))
        loop.close()
    except KeyboardInterrupt:
        print()