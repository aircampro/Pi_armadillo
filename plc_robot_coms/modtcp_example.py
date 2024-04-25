#!/usr/bin/env python3
# -*- coding: utf-8 -+-
# modtcp_server_to_client.py
#

# import the class files
from modTCP_server import ModbusTCPServer
from modTCP_client import ModbusTCPClient
import time
import sys

ipaddress = "127.0.0.1"                     # using 127.0.0.1 to test this with a loopback address (it would be the address for the server [slave dev] e.g. 192.168.1.123)
port = 502                                  # port designated to modbus TCP operations

# instantiate 

# Server (slave) this is on the machine which contains the data
modbus_server = ModbusTCPServer(ipaddress=ipaddress, port=port)                # listens to socket and replies

# Client (master) this is the machine requesting the data
modbus_client = ModbusTCPClient(ipaddress=ipaddress, port=port)                # initiates socket connection and asks for the data

#
# function is example of setting values in the server and confirming it by reading from that server with a client
#
def server_to_client(start_addr=0):
    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")
    # Start up modbusTCP server
    modbus_server.start_server()
    time.sleep(0.3)
    # try to connect to the server
    modbus_client.connect_to_server()
    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")

    # Check that the value of address=start_addr is the initial value of [0] on the client side
    modbus_client.read_holding_registers(address=start_addr)
    # Set the value of address=start_addr to 23 on the server side.
    modbus_server.set_holding_registers(address=start_addr, value=23)
    # Wait until the data is fully reflected on the client side
    time.sleep(2)
    # Check that the value of address=start_addr is updated to [23] on the client side
    modbus_client.read_holding_registers(address=start_addr)

    # Check that the value of address=start_addr+1 is the initial value of [0] on the client side
    modbus_client.read_holding_registers(address=start_addr+1)
    # Set the value of address=start_addr+1 to 25 on the server side.
    modbus_server.set_holding_registers(address=start_addr+1, value=25)
    # Wait until the data is fully reflected on the client side
    time.sleep(2)
    # Check that the value of address=start_addr+1 is updated to [25] on the client side by reading from the server
    modbus_client.read_holding_registers(address=start_addr+1)

    # Overwrite the value of address=start_addr+1 with 76 on the server side.
    modbus_server.set_holding_registers(address=start_addr+1, value=76)
    # Wait until the data is fully reflected on the client side
    time.sleep(2)
    # Check that the value of address=start_addr+1 is updated to [76] on the client side
    modbus_client.read_holding_registers(address=start_addr+1)

    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")
    # try to disconnect from the server
    modbus_client.disconnect_to_server()
    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")

#
# function is example of setting values in the server from a client and reading back the servers value
#
def client_to_server(start_addr):
    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")
    # Start up modbusTCP server
    modbus_server.start_server()
    # try to connect to the server
    modbus_client.connect_to_server()
    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")

    # Check that the value of address=start_addr is the initial value of [0] on the server side
    modbus_server.get_holding_registers(address=start_addr)
    # Set the value of address=start_addr to 167 on the client side
    modbus_client.write_single_register(address=start_addr, value=167)
    # Wait until the data is fully reflected on the server side
    time.sleep(2)
    # Check that the value of address=start_addr is [167] on the server side
    modbus_server.get_holding_registers(address=start_addr)

    # Check that the value of address=start_addr+1 is the initial value of [0] on the server side
    modbus_server.get_holding_registers(address=start_addr+1)
    # Set the value of address=start_addr+1 to [12, 28, 13] on the client side
    modbus_client.write_multiple_registers(address=start_addr+1, value=[12, 28, 13])
    # Wait until the data is fully reflected on the server side
    time.sleep(2)
    # Check that the value of address=start_addr+1 is [12, 28, 13] on the server side
    modbus_server.get_holding_registers(address=start_addr+1, num=3)

    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")
    # try to disconnect from the server
    modbus_client.disconnect_to_server()
    print(f"{modbus_client.get_nowtime} from client to server connected state:{modbus_client.get_connect_state}")
#
# terminate modbusTCP server
#
def close_down():
    modbus_server.stop_server()

if __name__ == '__main__':

    start_add = 0                                      # start address in modbus  
    argc = len(sys.argv)
    if (argc >= 1) :
        try:
            start_add = int(sys.argv[1])
        except:
            print("usage : ",sys.argv[0]," start_address_integer ")
    server_to_client(start_add)
    client_to_server(start_add)
    close_down()
