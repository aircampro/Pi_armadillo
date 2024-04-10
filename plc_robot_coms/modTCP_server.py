#!/usr/bin/env python3
# -*- coding: utf-8 -+-
# modbus tcp server using pyModbusTCP library
#
# pip install pyModbusTCP
#
from pyModbusTCP.server import ModbusServer
from datetime import datetime

class ModbusTCPServer():

    def __init__(self, ipaddress="192.168.1.123", port=502):
        self.__ipaddress = ipaddress
        self.__port = port
        self.__server = ModbusServer(ipaddress, port, no_block=True)
        pass

    @property
    def get_nowtime(self):
        """ fetch the operating system time """
        return datetime.now()

    @property
    def get_ipaddress(self):
        """ get the ip address """
        return self.__ipaddress

    @property
    def get_port(self):
        """ fetch the port number used """
        return self.__port

    def start_server(self):
        """ start the server """
        print(f"{self.get_nowtime} Start ModbusTCP Server")
        return self.__server.start()

    def stop_server(self):
        """ stop the server """
        print(f"{self.get_nowtime} Stop ModbusTCP Server")
        return self.__server.stop()

    def get_holding_registers(self, address, num=1):
        """ get holding registers """
        value = self.__server.data_bank.get_holding_registers(address, number=num)
        print(f"{self.get_nowtime} [server] get_holding_registers[address:{address}]: {value}")
        return value

    def set_holding_registers(self, address, value):
        """ set holding registers """
        print(f"{self.get_nowtime} [server] set_holding_registers[address:{address}]: {[value]}")
        return self.__server.data_bank.set_holding_registers(address, [value])

