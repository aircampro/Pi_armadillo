#!/usr/bin/env python
# run this websocket_server.py to allow remote execution of commands via a websocket connection to this server by remote clients
#
import asyncio
import websockets
import subprocess

async def server(websocket):
    command = await websocket.recv()
    print(f'Executing command \'{command}\'')
    (exit_code, output) = subprocess.getstatusoutput(command)
    if len(output.strip()) > 0:
        print(output, flush=True)
    print(f'Executed command\'s exit code: {exit_code}')
    await websocket.send(str(exit_code))

async def main():
    async with websockets.serve(server, port= 1463):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())