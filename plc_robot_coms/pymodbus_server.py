#!/usr/bin/env python3
# -*- coding: utf-8 -+-
# example of a server thread as a slave modbus tcp device reading its i/o pin7 and saome false periodic updates
#
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException, ParameterException
import threading

# Define the Modbus slave context
holding_registers = ModbusSequentialDataBlock(0, [0] * 10)
input_registers = ModbusSequentialDataBlock(0, [0] * 10)
coils = ModbusSequentialDataBlock(0, [False] * 10)
discrete_inputs = ModbusSequentialDataBlock(0, [False] * 10)
slave_context = ModbusSlaveContext(hr=holding_registers, ir=input_registers, co=coils, di=discrete_inputs)
context = ModbusServerContext(slaves=slave_context, single=False)

S_PORT=5020

# physical i/o on raspbery pi
import board
import RPi.GPIO as GPIO
import time

# GPIO pin scheme
GPIO.setmode(GPIO.BCM)                                                           # BCM channel, ex GPIO#
switch1 = 7; GPIO.setup(switch1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

# Define the Modbus server and start it
def run_server():
    try:
        server = StartTcpServer(context, address=("localhost", S_PORT))
        print("Modbus server started")
        server.serve_forever()
    except Exception as e:
        print(f"Error starting server: {str(e)}")

# Read holding registers
def read_holding_registers(start_address, count):
    try:
        client = ModbusTcpClient('localhost', port=S_PORT)
        response = client.read_holding_registers(start_address, count, unit=0x01)
        client.close()
        return response.registers
    except ModbusIOException as e:
        print(f"Modbus IO error: {str(e)}")
        return None
    except ParameterException as e:
        print(f"Parameter exception: {str(e)}")
        return None

# Write holding registers
def write_holding_registers(start_address, values):
    try:
        client = ModbusTcpClient('localhost', port=S_PORT)
        response = client.write_registers(start_address, values, unit=0x01)
        client.close()
        return response
    except ModbusIOException as e:
        print(f"Modbus IO error: {str(e)}")
        return None
    except ParameterException as e:
        print(f"Parameter exception: {str(e)}")
        return None

# Read coils
def read_coils(start_address, count):
    try:
        client = ModbusTcpClient('localhost', port=S_PORT)
        response = client.read_coils(start_address, count, unit=0x01)
        client.close()
        return response.bits
    except ModbusIOException as e:
        print(f"Modbus IO error: {str(e)}")
        return None
    except ParameterException as e:
        print(f"Parameter exception: {str(e)}")
        return None

# Write coils
def write_coils(start_address, values):
    try:
        client = ModbusTcpClient('localhost', port=S_PORT)
        response = client.write_coils(start_address, values, unit=0x01)
        client.close()
        return response
    except ModbusIOException as e:
        print(f"Modbus IO error: {str(e)}")
        return None
    except ParameterException as e:
        print(f"Parameter exception: {str(e)}")
        return None

def start():
    print(f"[LISTENING] Modbus TCP Server is listening on {S_PORT}")
    # Start the Modbus server in a new thread
    server_thread = threading.Thread(target=run_server)
    server_thread.start()
    while True:
        discrete_inputs[0] = GPIO.input(switch1)
        coils[3] = True
        coils[5] = True
        input_registers[0] = 123        
        holding_registers[0] = 6776
        time.sleep(5)
        coils[3] = False
        coils[5] = False
        input_registers[0] = 321        
        holding_registers[0] = 7667
        time.sleep(5)

if __name__ == '__main__':
    start()