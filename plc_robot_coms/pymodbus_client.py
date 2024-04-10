#!/usr/bin/env python3
# -*- coding: utf-8 -+-
# client class using pymodbus default port 5020
#
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException, ParameterException

class ModbusClient:
    def __init__(self, host='localhost', port=5020):
        self.host = host
        self.port = port
        self.client = ModbusTcpClient(self.host, self.port)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.client.close()

    def read_holding_registers(self, start_address, count):
        try:
            response = self.client.read_holding_registers(start_address, count, unit=0x01)
            return response.registers
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None

    def read_input_registers(self, start_address, count):
        try:
            response = self.client.read_input_registers(start_address, count, unit=0x01)
            return response.registers
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None

    def read_coils(self, start_address, count):
        try:
            response = self.client.read_coils(start_address, count, unit=0x01)
            return response.bits
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None

    def read_discrete_inputs(self, start_address, count):
        try:
            response = self.client.read_discrete_inputs(start_address, count, unit=0x01)
            return response.bits
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None
            
    def write_coil(self, addr, boolValue):
        try:
            rr = self.client.write_coil(addr, boolValue)          # example True or False
            return rr
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None	

    def write_coils(self, addr, boolValueList):
        try:
            rr = self.client.write_coils(addr, boolValueList)          # example boolValueList=[True,False]
            return rr
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None	
    
    def write_holding_register(self, addr, value): 
        try:
            rr = self.client.write_register(addr, value)
            return rr
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None		

    def write_holding_registers(self, addr, valuesList):
        try:
            rr = self.client.write_registers(addr, valuesList)          # example set starting at 9 values = [ 1, 3, 5, 8 ]
            return rr
        except ModbusIOException as e:
            print(f"Modbus IO error: {str(e)}")
            return None
        except ParameterException as e:
            print(f"Parameter exception: {str(e)}")
            return None	
