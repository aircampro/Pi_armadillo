# example of OPC DA ref:- https://github.com/iterativ/openopc2
#
# pip install openopc2
#
import os
from openopc2.config import OpenOpcConfig

def opc_config():
    open_opc_config = OpenOpcConfig()
    open_opc_config.OPC_SERVER = "Matrikon.OPC.Simulation.1"
    open_opc_config.OPC_GATEWAY_HOST = "192.168.0.115"
    open_opc_config.OPC_CLASS = "Graybox.OPC.DAWrapper"
    open_opc_config.OPC_MODE = 'com'
    return open_opc_config

from utils import get_opc_da_client
import time

def main():
    """
    This example show a few simple commands how to configure and connect an OPC server.
    for the ease of use print() is used extensively
    """
    open_opc_config = opc_config()
    paths = "*"
    #open_opc_config.OPC_SERVER = "Matrikon.OPC.Simulation"
    #open_opc_config.OPC_GATEWAY_HOST = "192.168.0.115"
    #open_opc_config.OPC_CLASS = "Graybox.OPC.DAWrapper"
    #open_opc_config.OPC_MODE = 'com'

    limit = 10
    n_reads = 1
    sync = False

    opc_client = get_opc_da_client(open_opc_config)
    tags = opc_client.list(paths=paths, recursive=False, include_type=False, flat=True)

    tags = [tag for tag in tags if "@" not in tag]
    if limit:
        tags = tags[:limit]
    print("TAGS:")
    for n, tag in enumerate(tags):
        print(f"{n:3} {tag}")
    #
    print("READ:")
    for n, tag in enumerate(tags):
        start = time.time()
        read = opc_client.read(tag, sync=sync)
        print(f'{n:3} {time.time()-start:.3f}s {tag} {read}')

    print("READ: LIST")
    for n in range(n_reads):
        start = time.time()
        read = opc_client.read(tags, sync=sync)
        print(f'{n:3} {time.time()-start:.3f}s {read}')

    print("PROPERTIES:")
    for n, tag in enumerate(tags):
        properties = opc_client.properties(tag)
        print(f'{n:3} {time.time()-start:.3f}s {tag} {properties}')

    print("PROPERTIES LIST:")
    for n in range(n_reads):
        start = time.time()
        properties = opc_client.properties(tags)
        print(f'{n} {time.time()-start:.3f}s {properties}')

    print("WRITE:")
    for n, tag in enumerate(tags):
        start = time.time()
        wrt = opc_client.write(tag, 10)
        read = opc_client.read(tag, sync=sync)
        print(f'{n:3} {time.time()-start:.3f}s {tag} {read}')
		
if __name__ == '__main__':
    main()